from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Set, Tuple

import numpy as np
from scipy.interpolate import CubicSpline, make_interp_spline

from src.models import CarPose, Cone, Path2D

YELLOW = 0  # right track boundary
BLUE = 1  # left track boundary

HALF_TRACK_WIDTH = 1.5  # nominal W/2 [m], used when no blue/yellow gate is visible to measure it
MIN_GATE_WIDTH = 0.5  # a blue/yellow pair closer than this is a detection error, not a gate [m]
MAX_GATE_WIDTH = 5.0  # a blue/yellow pair farther apart than this is not a gate [m]
MAX_GATE_ALIGNMENT = 0.8  # |cos| between a gate and a boundary above this means a diagonal pair, not a gate
BOUNDARY_SAMPLE_STEP = 0.5  # spacing of virtual centreline samples along a fitted boundary [m]
MIN_WAYPOINT_GAP = 0.3  # waypoints closer than this to the previous one (or the car) are merged [m]
TARGET_LENGTH = 8.0  # shorter paths are extended along the track direction to this length [m]
MAX_LENGTH = 9.5  # longer paths are truncated to this length (hard limit is 10 m) [m]
STEP = 0.25  # spacing of the returned path points [m] (limit is 0.5 m)
DENSE_STEP = 0.02  # spline evaluation resolution used before arc-length resampling [m]


@dataclass
class _Waypoint:
    """A centreline point the path must pass through, with the local track direction there."""

    point: np.ndarray
    tangent: np.ndarray  # unit vector


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


def _left_normal(t: np.ndarray) -> np.ndarray:
    return np.array([-t[1], t[0]])


def _right_normal(t: np.ndarray) -> np.ndarray:
    return np.array([t[1], -t[0]])


def _gate_direction(blue: np.ndarray, yellow: np.ndarray) -> np.ndarray:
    """Track direction through a gate: blue must be on the left, so rotate (yellow -> blue) by -90 deg."""
    return _unit(_right_normal(blue - yellow))


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Strategy (see README.md for the full write-up):
    - Blue and yellow visible: pair each cone with its nearest opposite-colour cone ("gate"),
      take the gate midpoints as centreline waypoints. Cones that cannot be paired are shifted
      inward by the measured half-width.
    - One colour only: order the cones from the car, fit a boundary curve through them
      (line / quadratic / cubic for 2 / 3 / 4+ cones) and offset it inward by W/2 = 1.5 m.
    - No cones: drive straight ahead along the current yaw.
    The car pose and the waypoints are joined by a clamped cubic spline (start tangent = car yaw,
    end tangent = local track direction), extended or truncated to 7.75-9.5 m and resampled
    every 0.25 m.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.
        """
        blue = self._unique_points([c for c in self.cones if c.color == BLUE])
        yellow = self._unique_points([c for c in self.cones if c.color == YELLOW])

        if len(blue) and len(yellow):
            waypoints = self._waypoints_from_both_sides(blue, yellow)
        elif len(blue):
            waypoints = self._waypoints_from_one_side(blue, is_left=True)
        elif len(yellow):
            waypoints = self._waypoints_from_one_side(yellow, is_left=False)
        else:
            waypoints = []

        return self._build_path(self._clean_waypoints(waypoints))

    # ------------------------------------------------------------------ car pose helpers

    def _position(self) -> np.ndarray:
        return np.array([self.car_pose.x, self.car_pose.y])

    def _heading(self) -> np.ndarray:
        return np.array([math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw)])

    # ------------------------------------------------------------------ cone ordering

    @staticmethod
    def _unique_points(cones: List[Cone]) -> np.ndarray:
        """Cone positions as an (N, 2) array with exact/near duplicates removed."""
        pts: List[np.ndarray] = []
        for c in cones:
            p = np.array([c.x, c.y], dtype=float)
            if all(np.linalg.norm(p - q) > 1e-3 for q in pts):
                pts.append(p)
        return np.array(pts).reshape(-1, 2)

    def _order_by_distance(self, pts: np.ndarray) -> np.ndarray:
        """Greedy nearest-neighbour chain starting at the car: cone 1 is closest to the car,
        cone k+1 is the remaining cone closest to cone k."""
        remaining = list(range(len(pts)))
        current = self._position()
        order: List[int] = []
        while remaining:
            nxt = min(remaining, key=lambda i: float(np.linalg.norm(pts[i] - current)))
            order.append(nxt)
            remaining.remove(nxt)
            current = pts[nxt]
        return pts[order]

    @staticmethod
    def _order_along(pts: np.ndarray, direction: np.ndarray) -> np.ndarray:
        """Sort points by their projection onto a direction (progress along the track)."""
        return pts[np.argsort(pts @ direction, kind="stable")]

    # ------------------------------------------------------------------ boundary fitting

    @staticmethod
    def _fit_boundary(pts: np.ndarray):
        """Parametric spline through ordered cones, parameterised by chord length.

        2 cones -> straight line, 3 cones -> quadratic, 4+ cones -> cubic. Returns the spline
        (callable on the parameter, values are (x, y)) and the parameter of each cone.
        """
        t = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
        spline = make_interp_spline(t, pts, k=min(3, len(pts) - 1))
        return spline, t

    def _boundary_tangents(self, pts: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        """Unit boundary direction at every (ordered) cone; `fallback` if only one cone."""
        if len(pts) < 2:
            return np.array([fallback] * len(pts)).reshape(-1, 2)
        spline, t = self._fit_boundary(pts)
        return np.array([_unit(d) for d in spline.derivative()(t)])

    # ------------------------------------------------------------------ waypoint generation

    def _waypoints_from_one_side(self, pts: np.ndarray, is_left: bool) -> List[_Waypoint]:
        """Virtual centreline: shift the single visible boundary inward by the nominal half-width.

        Blue (left boundary) is shifted to its right, yellow (right boundary) to its left.
        """
        inward = _right_normal if is_left else _left_normal

        if len(pts) == 1:
            # One cone gives no boundary direction: assume the boundary runs parallel to the car.
            heading = self._heading()
            return [_Waypoint(pts[0] + HALF_TRACK_WIDTH * inward(heading), heading)]

        pts = self._order_by_distance(pts)
        spline, t = self._fit_boundary(pts)
        n_samples = max(2, int(math.ceil(t[-1] / BOUNDARY_SAMPLE_STEP)) + 1)
        params = np.linspace(0.0, t[-1], n_samples)
        samples = spline(params)
        tangents = [_unit(d) for d in spline.derivative()(params)]
        return [_Waypoint(p + HALF_TRACK_WIDTH * inward(tan), tan) for p, tan in zip(samples, tangents)]

    def _waypoints_from_both_sides(self, blue: np.ndarray, yellow: np.ndarray) -> List[_Waypoint]:
        """Gate midpoints between paired blue/yellow cones, plus inward-shifted unpaired cones."""
        # Provisional track direction from the closest blue/yellow pair; it orders both boundaries.
        dist = np.linalg.norm(blue[:, None, :] - yellow[None, :, :], axis=2)
        dist[dist < MIN_GATE_WIDTH] = np.inf
        if np.isfinite(dist).any():
            bi, yi = np.unravel_index(int(np.argmin(dist)), dist.shape)
            track_dir = _gate_direction(blue[bi], yellow[yi])
        else:
            track_dir = self._heading()

        blue = self._order_along(blue, track_dir)
        yellow = self._order_along(yellow, track_dir)
        dist = np.linalg.norm(blue[:, None, :] - yellow[None, :, :], axis=2)
        blue_tan = self._boundary_tangents(blue, track_dir)
        yellow_tan = self._boundary_tangents(yellow, track_dir)

        # Every cone proposes a gate with its nearest opposite-colour cone.
        candidates = [(i, int(np.argmin(dist[i]))) for i in range(len(blue))]
        candidates += [(int(np.argmin(dist[:, j])), j) for j in range(len(yellow))]
        gates: Set[Tuple[int, int]] = {
            (i, j) for i, j in candidates if self._is_gate(blue[i], yellow[j], blue_tan[i], yellow_tan[j])
        }

        widths = [float(dist[i, j]) for i, j in gates]
        half_width = 0.5 * float(np.median(widths)) if widths else HALF_TRACK_WIDTH

        waypoints = [
            _Waypoint(0.5 * (blue[i] + yellow[j]), _gate_direction(blue[i], yellow[j])) for i, j in sorted(gates)
        ]
        paired_blue = {i for i, _ in gates}
        paired_yellow = {j for _, j in gates}
        for i in range(len(blue)):
            if i not in paired_blue:
                waypoints.append(_Waypoint(blue[i] + half_width * _right_normal(blue_tan[i]), blue_tan[i]))
        for j in range(len(yellow)):
            if j not in paired_yellow:
                waypoints.append(_Waypoint(yellow[j] + half_width * _left_normal(yellow_tan[j]), yellow_tan[j]))

        waypoints.sort(key=lambda w: float(w.point @ track_dir))
        return waypoints

    @staticmethod
    def _is_gate(blue: np.ndarray, yellow: np.ndarray, blue_tan: np.ndarray, yellow_tan: np.ndarray) -> bool:
        """A blue/yellow pair is a gate if it is narrow enough and roughly across the track
        (not a diagonal between cones that are far apart along the track)."""
        across = blue - yellow
        width = float(np.linalg.norm(across))
        if not MIN_GATE_WIDTH <= width <= MAX_GATE_WIDTH:
            return False
        across = across / width
        return abs(float(across @ blue_tan)) < MAX_GATE_ALIGNMENT and abs(float(across @ yellow_tan)) < MAX_GATE_ALIGNMENT

    def _clean_waypoints(self, waypoints: List[_Waypoint]) -> List[_Waypoint]:
        """Drop waypoints the car has already passed and merge ones that are too close together."""
        car = self._position()
        ahead = [w for w in waypoints if float((w.point - car) @ w.tangent) > 0.0]
        if ahead:
            waypoints = ahead

        cleaned: List[_Waypoint] = []
        previous = car
        for w in waypoints:
            if np.linalg.norm(w.point - previous) >= MIN_WAYPOINT_GAP:
                cleaned.append(w)
                previous = w.point
        if not cleaned and waypoints:
            # Everything sits on top of the car: keep only the track direction.
            cleaned = [_Waypoint(car + MIN_WAYPOINT_GAP * waypoints[-1].tangent, waypoints[-1].tangent)]
        return cleaned

    # ------------------------------------------------------------------ path construction

    def _build_path(self, waypoints: List[_Waypoint]) -> Path2D:
        """Clamped cubic spline car -> waypoints, straight extension, arc-length resampling."""
        start = self._position()
        heading = self._heading()

        if waypoints:
            knots = np.vstack([start] + [w.point for w in waypoints])
            end_tangent = waypoints[-1].tangent
            t = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(knots, axis=0), axis=1))))
            # Chord-length parameter ~ arc length, so unit tangents are the right derivative scale.
            spline = CubicSpline(t, knots, axis=0, bc_type=((1, heading), (1, end_tangent)))
            dense = spline(np.linspace(0.0, t[-1], max(2, int(t[-1] / DENSE_STEP) + 1)))
        else:
            # No cones: dead-reckon straight ahead.
            end_tangent = heading
            dense = start[None, :]

        arc = self._arc_length(dense)
        if arc[-1] < TARGET_LENGTH:
            extra = np.arange(DENSE_STEP, TARGET_LENGTH - arc[-1] + DENSE_STEP, DENSE_STEP)
            dense = np.vstack([dense, dense[-1] + extra[:, None] * end_tangent])
            arc = self._arc_length(dense)

        return self._resample(dense, arc, min(arc[-1], MAX_LENGTH))

    @staticmethod
    def _arc_length(pts: np.ndarray) -> np.ndarray:
        return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))

    @staticmethod
    def _resample(pts: np.ndarray, arc: np.ndarray, length: float) -> Path2D:
        """Points every STEP metres of arc length from the car up to `length`."""
        s = np.arange(0.0, length + 1e-9, STEP)
        xs = np.interp(s, arc, pts[:, 0])
        ys = np.interp(s, arc, pts[:, 1])
        return [(float(x), float(y)) for x, y in zip(xs, ys)]
