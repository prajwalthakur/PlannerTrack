#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""The baseline experiment (project_docs/learning_based_controls/results/baseline.md).

Fixed-weight MPC (θ0) on the nominal car and on perturbed cars, with the nominal MPC model and with the
oracle (MPC given the true parameters), 8 start points evenly spaced around the lap, one lap each; an
episode ends at the first failure flag (off_track, too_slow, heading_out, diverged). All configurations
run as ONE batched rollout per MPC track margin (one compiled solver; the margin is a stage parameter).

Configurations (plant perturbation, MPC model):
    nominal car                     nominal
    μ × 0.9                         nominal, oracle
    heavier × 1.2 (Perturbation.heavier: mass, I_z × 1.2; c_f, c_r, max_accel × 1/1.2)   nominal, oracle
    μ × 0.8                         nominal, oracle

Writes to <out>/baseline_suite/:
    table.md        the comparison table (laps completed, smallest margin, per track margin) + details
    table.csv       one row per configuration and track margin
    per_car.csv     one row per car (all metrics of closed_loop.compute_metrics)

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/baseline_suite.py                    # margins 0 and 3 cm
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/baseline_suite.py --track-margins 0.03

Evaluate other weights with the same suite (learned θ, Steps 6-7); results go to baseline_suite_<tag>/:

    python3 .../baseline_suite.py --theta-file rl_framework/data/train_static/static/theta.json --tag static
    python3 .../baseline_suite.py --theta 50 2.9 2.0 5.0 0.01 6.8 --tag manual
"""

import argparse
import json
import csv
import dataclasses
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from plannertrack_py import ReferenceTrack, compute_raceline
from plannertrack_py.batch_plant import BatchPlant
from diff_weight_mpc.closed_loop import compute_metrics, evenly_spaced_starts, run_episodes
from diff_weight_mpc.mpc.controller import MpcController
from diff_weight_mpc.learning.policy_network import WeightPolicy
from diff_weight_mpc.mpc.ocp import OcpConfig
from plannertrack_py.plant import Perturbation

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent

# (name, plant perturbation, MPC gets the true parameters)
CONFIGS = [
    ("nominal car", Perturbation(), False),
    ("μ × 0.9, nominal model", Perturbation(mu_scale=0.9), False),
    ("μ × 0.9, oracle", Perturbation(mu_scale=0.9), True),
    ("heavier × 1.2, nominal model", Perturbation.heavier(1.2), False),
    ("heavier × 1.2, oracle", Perturbation.heavier(1.2), True),
    ("μ × 0.8, nominal model", Perturbation(mu_scale=0.8), False),
    ("μ × 0.8, oracle", Perturbation(mu_scale=0.8), True),
]


def summarize(rows):
    """Aggregate the per-car metrics of one configuration (8 starts)."""
    done = [r for r in rows if r["completed"]]
    return {
        "laps": f"{len(done)}/{len(rows)}",
        "laps_completed": len(done),
        "cars": len(rows),
        "margin_min_cm": 100 * min(r["margin_min"] for r in rows),
        "n_rms_median_cm": 100 * float(np.median([r["n_rms"] for r in rows])),
        "n_max_cm": 100 * max(r["n_max"] for r in rows),
        "dv_rms_median": float(np.median([r["dv_rms"] for r in rows])),
        "lap_time_median": float(np.median([r["lap_time"] for r in done])) if done else np.nan,
        "loss_mean": float(np.mean([r["loss_mean"] for r in rows])),
        "end_reasons": "+".join(sorted({r["end_reason"] for r in rows})),
        "solver_failures": int(sum(r["failures"] for r in rows)),
    }


def load_theta_file(path, key="theta"):
    """θ (6,) from a train_mpc.py theta.json (``key``: "theta" = final, "best_theta" = best batch)."""
    value = json.loads(Path(path).read_text()).get(key)
    if value is None:
        raise ValueError(f"{path} has no {key!r} (best_theta is null when no training window qualified)")
    if len(value) != 6:
        raise ValueError(f"{path}: {key!r} has {len(value)} entries, expected 6")
    return value


def resolve_theta(args, base_cfg):
    """The weights to evaluate: --theta, else --theta-file, else the baseline θ0. Returns numpy (6,)."""
    if args.theta is not None:
        return np.array(args.theta, dtype=float)
    if args.theta_file is not None:
        return np.asarray(load_theta_file(args.theta_file, args.theta_key), dtype=float)
    return np.array(base_cfg.theta0, dtype=float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--starts", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--track-margins", type=float, nargs="+", default=[0.0, 0.03],
                    help="MPC-only track tightening(s) to compare [m]")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "closed_loop")
    weights = ap.add_mutually_exclusive_group()
    weights.add_argument("--theta", type=float, nargs=6, metavar=("q_n", "q_psi", "q_v", "q_alat", "r_j", "r_omega"),
                         help="weights to evaluate instead of θ0")
    weights.add_argument("--theta-file", type=Path, help="theta.json written by train_mpc.py")
    weights.add_argument("--policy-file", type=Path, help="policy.pt written by train_mpc.py --model policy (Step 7)")
    ap.add_argument("--theta-key", default="theta", choices=("theta", "best_theta"))
    ap.add_argument("--tag", default=None, help="output folder suffix (baseline_suite_<tag>); keeps the baseline")
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    starts = evenly_spaced_starts(ref, args.starts)
    cars = [(c, s0) for c in CONFIGS for s0 in starts]
    plant = BatchPlant.from_scenario(params, n_cars=len(cars), agent_name=args.agent, reference=ref,
                                     perturbations=[c[1] for c, _ in cars])
    base_cfg = OcpConfig()
    policy = None if args.policy_file is None else WeightPolicy.load(args.policy_file)
    theta = None if policy is not None else resolve_theta(args, base_cfg)
    mpc = MpcController.create_controller(plant, cfg=base_cfg)
    vehicle = [p.true_params if oracle else p.nominal_params
               for p, ((_, _, oracle), _) in zip(plant.plants, cars)]

    table, per_car_rows = [], []
    for margin in args.track_margins:
        mpc.cfg = dataclasses.replace(base_cfg, track_margin=margin)  # a stage parameter: no recompile
        log = run_episodes(plant, mpc, theta=theta, theta_fn=policy, s0=[s0 for _, s0 in cars],
                           seed=args.seed, vehicle_params=vehicle)
        per_car, summary = compute_metrics(log, plant)
        for j, (name, pert, oracle) in enumerate(CONFIGS):
            rows = per_car[j * len(starts):(j + 1) * len(starts)]
            table.append({"config": name, "track_margin_cm": 100 * margin, **summarize(rows)})
            for r in rows:
                per_car_rows.append({"config": name, "track_margin_cm": 100 * margin,
                                     **{k: (str(v) if isinstance(v, Perturbation) else v) for k, v in r.items()}})
        print(f"track margin {100 * margin:g} cm: {summary['completed']}/{summary['cars']} laps, "
              f"{summary['wall_time_per_car_ms']:.2f} ms per car per call")

    out = args.out / ("baseline_suite" if args.tag is None else f"baseline_suite_{args.tag}")
    out.mkdir(parents=True, exist_ok=True)
    for path, rows in ((out / "table.csv", table), (out / "per_car.csv", per_car_rows)):
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)

    margins = args.track_margins
    by = {(r["config"], r["track_margin_cm"]): r for r in table}
    head = " | ".join(f"laps, {100 * m:g} cm" for m in margins)
    head2 = " | ".join(f"smallest margin, {100 * m:g} cm" for m in margins)
    weights = f"policy {args.policy_file}" if policy is not None else f"θ = {tuple(float(t) for t in theta)}"
    lines = [f"{weights}, slack_track = {base_cfg.slack_track}, {args.starts} starts, one lap, "
             "an episode ends at the first failure flag", "",
             f"| run | {head} | {head2} |", "|---" * (1 + 2 * len(margins)) + "|"]
    for name, _, _ in CONFIGS:
        laps = " | ".join(by[(name, 100 * m)]["laps"] for m in margins)
        marg = " | ".join(f"{by[(name, 100 * m)]['margin_min_cm']:+.1f} cm" for m in margins)
        lines.append(f"| {name} | {laps} | {marg} |")
    lines += ["", "Details per track margin:", "",
              "| run | track margin | laps | median RMS \\|n\\| [cm] | max \\|n\\| [cm] | median RMS \\|dv\\| [m/s] | "
              "median lap [s] | mean loss | solver failures |", "|---|---|---|---|---|---|---|---|---|"]
    for r in table:
        lines.append(f"| {r['config']} | {r['track_margin_cm']:g} cm | {r['laps']} | {r['n_rms_median_cm']:.1f} | "
                     f"{r['n_max_cm']:.1f} | {r['dv_rms_median']:.3f} | {r['lap_time_median']:.2f} | "
                     f"{r['loss_mean']:.3f} | {r['solver_failures']} |")
    text = "\n".join(lines) + "\n"
    (out / "table.md").write_text(text)
    print(text, end="")
    print(f"written: {out}/")


if __name__ == "__main__":
    main()
