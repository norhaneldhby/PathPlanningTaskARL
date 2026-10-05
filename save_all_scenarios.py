"""Render every scenario headlessly and save it to screenshots/scenario_<N>.png.

Usage (from the repository root):
    python save_all_scenarios.py               # all scenarios
    python save_all_scenarios.py 2 21 23       # selected scenarios
"""
from __future__ import annotations

import os
import sys
import warnings

import matplotlib

matplotlib.use("Agg")  # must happen before pyplot is imported (PathTester imports it)
# PathTester.run() calls plt.show(), which only warns under the non-interactive Agg backend.
warnings.filterwarnings("ignore", message=".*non-interactive.*")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src.scenarios import get_scenario_names, make_scenario  # noqa: E402
from src.tester import PathTester  # noqa: E402

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "screenshots")


def path_stats(path) -> tuple:
    pts = np.asarray(path)
    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return float(steps.sum()), float(steps.max())


def save_scenario(name: str) -> str:
    cones, car_pose = make_scenario(name)
    path = PathTester(cones=cones, car_pose=car_pose).run()  # plt.show() is a no-op under Agg

    length, max_step = path_stats(path)
    ax = plt.gca()
    ax.set_title(f"Scenario {name}: path length {length:.2f} m, max step {max_step:.2f} m")

    out = os.path.join(OUTPUT_DIR, f"scenario_{name}.png")
    plt.savefig(out, dpi=110, bbox_inches="tight")
    plt.close("all")

    ok = 5.0 <= length <= 10.0 and max_step <= 0.5
    print(f"[{'OK ' if ok else 'BAD'}] scenario {name:>2}: {len(path):3d} pts, "
          f"length {length:5.2f} m, max step {max_step:.2f} m -> {os.path.relpath(out)}")
    return out


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    names = sys.argv[1:] or get_scenario_names()
    for name in names:
        save_scenario(name)


if __name__ == "__main__":
    main()
