"""Run repeated resets and shortcut controls through ordinary YAM joint actions."""

import argparse
from importlib.metadata import version
import json
import os
from pathlib import Path
import time

import numpy as np

from inspect_robots import Scene
from inspect_robots.types import Action
from inspect_robots_yam.packing import pack
from yam_arena import station
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan
from yam_arena.inspect_embodiment import yam_arena_embodiment


def run_case(embodiment, mode):
    start = time.monotonic()
    task = embodiment.task.module
    os.environ["TRAY_DEMO"] = mode
    embodiment.reset(Scene(id=mode, instruction=task.INSTRUCTION), seed=0)
    plans = {} if mode == "idle" else {
        arm: ArmPlan(points, station.BASES[arm]) for arm, points in task.demo().items()
    }
    duration = max((plan.duration for plan in plans.values()), default=3.0)
    max_steps = min(1200, round((duration + 2) * 20))
    max_score = 0.
    max_rise = 0.
    start_relative_z = None
    for step in range(1, max_steps + 1):
        targets = [plans[arm].action7(step / 20)[0] if arm in plans else np.r_[READY, 1.]
                   for arm in ("left", "right")]
        result = embodiment.step(Action(data=pack(*targets)))
        max_score = max(max_score, float(result.info["score"]))
        diagnostic = task.DIAGNOSTICS
        relative_z = diagnostic["inner_xyz"][2] - diagnostic["outer_xyz"][2]
        if start_relative_z is None:
            start_relative_z = relative_z
        max_rise = max(max_rise, relative_z - start_relative_z)
        if result.terminated:
            break
    return {"case": mode, "steps": step, "seconds": round(time.monotonic() - start, 2),
            "maximum_score": max_score, "max_relative_rise_mm": 1000 * max_rise, **diagnostic}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cases", nargs="+", choices=["full", "idle", "straight", "together", "slide", "one_pad"],
                        default=["idle", "straight", "together", "one_pad", "full", "full"])
    args = parser.parse_args()
    embodiment = yam_arena_embodiment(task=str(Path(__file__).with_name("task.py")), object_jitter=0,
                                      control_hz=20, cam_width=320, cam_height=176)
    summary = {"seed": 0, "versions": {pkg: version(pkg) for pkg in
               ("isaacsim", "isaaclab_arena", "inspect-robots", "yam-arena")}, "cases": []}
    for case in args.cases:
        result = run_case(embodiment, case)
        summary["cases"].append(result)
        print(json.dumps(result), flush=True)
        args.output.write_text(json.dumps(summary, indent=2) + "\n")
    embodiment.close()


if __name__ == "__main__":
    main()
