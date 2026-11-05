#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Step 1 deliverables (steps.md §3.5): reference .npz, validation plot and log.

Computes the minimum-curvature raceline with the same code and scenario files as the
ROS plugin MinCurvPlanner, validates it (steps.md §3.4(6)) and writes

    <out>/reference_<track>.npz             arrays for the MPC / policy (ReferenceTrack.from_npz)
    <out>/reference_<track>.png             validation plot
    <out>/reference_<track>_validation.log  check results

Exit code 1 if a check fails. Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/make_reference.py
"""

import argparse
import sys
from pathlib import Path

from plannertrack_py.raceline import centerline_from_config, compute_raceline
from plannertrack_py.reference import load_raceline_limits, plot_reference, validate_reference

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl", help="folder under src/scenarios")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--track", default=None, help="name in the output files (default: scenario)")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "references")
    ap.add_argument("--low-speed", type=float, default=2.5, help="lowest valid MPC speed (mpc.model.MIN_VALID_SPEED) [m/s]")
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    agents, planner = params / "agents.yaml", params / "min_curv_planner.yaml"
    track = args.track or args.scenario

    raceline = compute_raceline(agents, args.agent, planner)
    limits = load_raceline_limits(agents, args.agent, planner)
    report = validate_reference(raceline, limits, low_speed=args.low_speed)

    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.out / f"reference_{track}"
    raceline.save_npz(stem.with_suffix(".npz"))
    plot_reference(raceline, stem.with_suffix(".png"), lim=limits,
                   centerline=centerline_from_config(planner),
                   title=f"{track}: {raceline.length:.2f} m, lap {raceline.lap_time:.2f} s, "
                         f"v_max {raceline.vx.max():.2f} m/s")
    header = (f"reference_{track}: {len(raceline)} points, length {raceline.length:.3f} m, "
              f"lap {raceline.lap_time:.3f} s\nlimits: {limits}\n")
    log = header + report.format() + "\n"
    Path(f"{stem}_validation.log").write_text(log)
    print(log, end="")
    print(f"written: {stem}.npz, {stem}.png, {stem}_validation.log")
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
