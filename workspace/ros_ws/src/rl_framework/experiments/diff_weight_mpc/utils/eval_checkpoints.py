#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Closed-loop trend of a static Diff-MPC training run (steps.md 6.9): every checkpoint's θ on fixed starts.

train_mpc.py logs the TRAINING lap of every update (random start, θ changing every lap: noisy and not the
evaluation metric). This script replays θ0 (as update 0) and every checkpoint's θ (saved every 5 updates) on the
baseline suite's fixed starts (8 evenly spaced, one lap, 3 cm track margin) for a few configurations, so the
evaluation metrics can be followed from before training, through training, to the end.

Per checkpoint and configuration: laps completed, smallest margin, median RMS |n|, max |n|, median RMS |v − v_ref|,
median lap time, and the mean task loss with the run's own training loss weights (theta.json config).

Outputs: <run>/checkpoint_eval.csv, <run>/checkpoint_eval.png, and <runs parent>/checkpoint_eval_runs.png (the
nominal car of every run evaluated, one line per run).

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/eval_checkpoints.py softplus_lr1e-1 exp_lr3e-2
"""

import argparse
import csv
import dataclasses
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from baseline_suite import CONFIGS, summarize  # noqa: E402  (same folder)
from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from plannertrack_py.batch_plant import BatchPlant  # noqa: E402
from diff_weight_mpc.closed_loop import compute_metrics, evenly_spaced_starts, run_episodes  # noqa: E402
from diff_weight_mpc.learning.loss import LossConfig  # noqa: E402
from diff_weight_mpc.learning.policy_network import WeightPolicy  # noqa: E402
from diff_weight_mpc.mpc.controller import MpcController  # noqa: E402
from diff_weight_mpc.mpc.ocp import THETA0, OcpConfig  # noqa: E402
from train_mpc import GRID_COLOR, REF_COLOR, SERIES, _to_theta  # noqa: E402  (same folder)

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent
DEFAULT_CONFIGS = ("nominal car", "μ × 0.9, nominal model", "heavier × 1.2, nominal model")
METRICS = [  # (column, axis label)
    ("loss_mean", "mean loss (run's weights)"),
    ("n_rms_median_cm", "median RMS |n| [cm]"),
    ("margin_min_cm", "smallest margin [cm]"),
    ("dv_rms_median", "median RMS |v − v_ref| [m/s]"),
    ("lap_time_median", "median lap time [s]"),
    ("n_max_cm", "max |n| [cm]"),
]


def checkpoint_thetas(run_dir):
    """[(update, weights)]: the run's θ0 as update 0 (theta.json; ocp.THETA0 for older runs without it; a policy
    starts by outputting exactly θ0), then every checkpoint in update order. weights = θ (6,) for a static run (from
    w and the weight map), a WeightPolicy for a policy run."""
    theta0 = json.loads((run_dir / "theta.json").read_text()).get("config", {}).get("theta0", THETA0)
    points = [(0, np.array(theta0, dtype=float))]
    for path in sorted((run_dir / "checkpoints").glob("update_*.pt")):
        ckpt = torch.load(path, weights_only=False)  # our own files (model, optimizer state, history)
        if ckpt.get("model") == "policy":
            weights = WeightPolicy(**ckpt["policy_config"])
            weights.load_state_dict(ckpt["state_dict"])
            weights.eval()
        else:
            with torch.no_grad():
                weights = _to_theta(ckpt["w"], ckpt["weight_param"]).numpy()
        points.append((int(ckpt["update"]), weights))
    return points


def resolve_run(run, runs_dir):
    """A run tag under ``runs_dir``, or a path containing "/" relative to rl_framework/data (e.g.
    train_policy/<tag>), or an absolute path."""
    path = Path(run)
    if path.is_absolute():
        return path
    return RL_FRAMEWORK / "data" / path if "/" in run else runs_dir / run


def run_loss_config(run_dir):
    """The LossConfig the run trained with (theta.json "config"), so the evaluation loss is the training loss."""
    config = json.loads((run_dir / "theta.json").read_text())["config"]["loss_cfg"]
    return LossConfig(**config)


def build(args):
    """Plant (the chosen configurations × the fixed starts, one batch), controller, the configurations."""
    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    configs = [c for c in CONFIGS if c[0] in args.configs]
    starts = evenly_spaced_starts(ref, args.starts)
    cars = [(c, s0) for c in configs for s0 in starts]
    plant = BatchPlant.from_scenario(params, n_cars=len(cars), agent_name=args.agent, reference=ref,
                                     perturbations=[c[1] for c, _ in cars])
    mpc = MpcController.create_controller(plant, cfg=dataclasses.replace(OcpConfig(), track_margin=args.track_margin))
    vehicle = [p.true_params if oracle else p.nominal_params for p, ((_, _, oracle), _) in zip(plant.plants, cars)]
    return plant, mpc, configs, starts, cars, vehicle


def evaluate(plant, mpc, configs, starts, cars, vehicle, weights, loss_cfg, seed):
    """One θ (6,) or WeightPolicy on every configuration × start: rows of summarize(...) per configuration."""
    is_policy = isinstance(weights, torch.nn.Module)
    log = run_episodes(plant, mpc, theta=None if is_policy else weights, theta_fn=weights if is_policy else None,
                       s0=[s0 for _, s0 in cars], seed=seed, vehicle_params=vehicle, loss_cfg=loss_cfg)
    per_car, _ = compute_metrics(log, plant)
    n = len(starts)
    return {name: summarize(per_car[j * n:(j + 1) * n]) for j, (name, _, _) in enumerate(configs)}


def plot_run(rows, configs, path, title):
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.5), sharex=True)
    for ax, (column, label) in zip(axes.ravel(), METRICS):
        for i, (name, _, _) in enumerate(configs):
            pts = [(r["update"], r[column]) for r in rows if r["config"] == name]
            ax.plot(*zip(*pts), "-o", lw=2, ms=4, color=SERIES[i], label=name)
        if column == "margin_min_cm":
            ax.axhline(0.0, color=REF_COLOR, lw=1, ls=":")
        ax.set_title(label, fontsize=9)
        ax.grid(True, color=GRID_COLOR, lw=0.6)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    for ax in axes[-1]:
        ax.set_xlabel("update (0 = θ0)")
    axes[0, 0].legend(frameon=False, fontsize=8)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_runs(all_rows, config, path):
    """The chosen configuration of every run, one line per run (loss left out: runs may use different weights)."""
    fig, axes = plt.subplots(1, 5, figsize=(18, 3.6), sharex=True)
    for ax, (column, label) in zip(axes, METRICS[1:]):
        for i, (run, rows) in enumerate(all_rows.items()):
            pts = [(r["update"], r[column]) for r in rows if r["config"] == config]
            ax.plot(*zip(*pts), "-o", lw=2, ms=3, color=SERIES[i], label=run)
        if column == "margin_min_cm":
            ax.axhline(0.0, color=REF_COLOR, lw=1, ls=":")
        ax.set_title(label, fontsize=9)
        ax.set_xlabel("update (0 = θ0)")
        ax.grid(True, color=GRID_COLOR, lw=0.6)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    axes[0].legend(frameon=False, fontsize=8)
    fig.suptitle(f"{config}: evaluation along training (fixed starts)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run tags under --runs-dir (train_mpc.py --tag), or paths relative to "
                    "rl_framework/data with a '/' (e.g. train_policy/<tag>)")
    ap.add_argument("--runs-dir", type=Path, default=RL_FRAMEWORK / "data" / "train_static")
    ap.add_argument("--configs", nargs="+", default=list(DEFAULT_CONFIGS), choices=[c[0] for c in CONFIGS])
    ap.add_argument("--starts", type=int, default=8)
    ap.add_argument("--track-margin", type=float, default=0.03)
    ap.add_argument("--every", type=int, default=1, help="evaluate every n-th checkpoint (θ0 and the last always)")
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--compare-out", type=Path, default=None,
                    help="comparison plot of several runs (default <runs-dir>/checkpoint_eval_runs.png)")
    args = ap.parse_args()

    plant, mpc, configs, starts, cars, vehicle = build(args)
    lap_obs = np.concatenate(plant.reference.policy_observation(
        np.linspace(0.0, plant.reference.length, 200, endpoint=False)), axis=1)
    all_rows = {}
    for run in args.runs:
        run_dir = resolve_run(run, args.runs_dir)
        loss_cfg = run_loss_config(run_dir)
        points = checkpoint_thetas(run_dir)
        points = [p for i, p in enumerate(points) if i % args.every == 0 or i == len(points) - 1]
        rows = []
        for update, weights in points:
            if isinstance(weights, torch.nn.Module):  # a policy: record its mean θ over the lap
                with torch.no_grad():
                    theta = weights(lap_obs).numpy().mean(axis=0)
            else:
                theta = weights
            for name, summary in evaluate(plant, mpc, configs, starts, cars, vehicle, weights, loss_cfg,
                                          args.seed).items():
                rows.append({"update": update, "config": name, **summary,
                             **{f"theta_{i}": float(v) for i, v in enumerate(theta)}})
            nominal = next(r for r in rows[::-1] if r["config"] == configs[0][0])
            print(f"{run} update {update:4d}: {configs[0][0]}: laps {nominal['laps']}, RMS |n| "
                  f"{nominal['n_rms_median_cm']:.1f} cm, margin {nominal['margin_min_cm']:+.1f} cm, RMS |dv| "
                  f"{nominal['dv_rms_median']:.3f}, lap {nominal['lap_time_median']:.2f} s, loss "
                  f"{nominal['loss_mean']:.3f}", flush=True)
        with open(run_dir / "checkpoint_eval.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        plot_run(rows, configs, run_dir / "checkpoint_eval.png",
                 f"{run}: θ0 and every checkpoint on {args.starts} fixed starts, track margin "
                 f"{100 * args.track_margin:g} cm")
        all_rows[run] = rows
    if len(all_rows) > 1:
        plot_runs(all_rows, configs[0][0], args.compare_out or args.runs_dir / "checkpoint_eval_runs.png")


if __name__ == "__main__":
    main()
