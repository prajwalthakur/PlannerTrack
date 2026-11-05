#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""θ along the lap for Diff-WMPC policies (steps.md §10, 7.6; the paper's Fig. 5 as line plots).

For every s on the lap, the policy is evaluated at the observation it sees there, θ(s) = π(o(s)) (5 speeds and 5
curvatures ahead, Reference.policy_observation): deterministic, the weights the controller would get on the
reference line. One panel per weight (log scale), the reference speed and curvature on top for context, a static
θ (theta.json) as a dashed reference line.

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/policy_profile.py \\
        --policies train_policy/wmpc_main/policy.pt --static train_static/theta0_sp1e-1_lat1_v1_200/theta.json
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from diff_weight_mpc.learning.policy_network import WeightPolicy  # noqa: E402
from diff_weight_mpc.mpc.ocp import THETA_NAMES  # noqa: E402
from train_mpc import GRID_COLOR, REF_COLOR, SERIES  # noqa: E402  (same folder)

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent


def data_path(p):
    """A path relative to rl_framework/data (or absolute)."""
    p = Path(p)
    return p if p.is_absolute() else RL_FRAMEWORK / "data" / p


def profile(policy, reference, s):
    """θ(s) (len(s), 6) = π(o(s)) along the lap."""
    obs = np.concatenate(reference.policy_observation(s), axis=1)
    with torch.no_grad():
        return policy(obs).numpy()


def style(ax):
    ax.grid(True, color=GRID_COLOR, lw=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="+", required=True, help="policy.pt files (relative to rl_framework/data)")
    ap.add_argument("--labels", nargs="+", default=None)
    ap.add_argument("--static", default=None, help="theta.json of a static run, drawn as a dashed line")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "train_policy" / "policy_profile.png")
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    s = np.linspace(0.0, ref.length, 400, endpoint=False)
    labels = args.labels or [Path(p).parent.name for p in args.policies]
    profiles = {label: profile(WeightPolicy.load(data_path(p)), ref, s) for label, p in zip(labels, args.policies)}
    static = None if args.static is None else np.array(json.loads(data_path(args.static).read_text())["theta"])

    r = ref.sample(s)
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    ax_v, ax_k = axes[0]
    ax_v.plot(s, r["vx"], color=REF_COLOR, lw=2)
    ax_v.set_title("reference speed [m/s]", fontsize=9)
    ax_k.plot(s, r["kappa"], color=REF_COLOR, lw=2)
    ax_k.axhline(0.0, color=REF_COLOR, lw=0.8, ls=":")
    ax_k.set_title("reference curvature [1/m] (< 0: right turn)", fontsize=9)
    for i, (ax, name) in enumerate(zip(axes[1:].ravel(), THETA_NAMES)):
        for j, (label, th) in enumerate(profiles.items()):
            ax.plot(s, th[:, i], lw=2, color=SERIES[j], label=label)
        if static is not None:
            ax.axhline(static[i], color=REF_COLOR, lw=1.2, ls="--", label="static θ")
        ax.set_yscale("log")
        plain = matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}")
        ax.yaxis.set_major_formatter(plain)
        ax.yaxis.set_minor_formatter(plain)
        ax.set_title(name, fontsize=9)
    for ax in axes.ravel():
        style(ax)
    for ax in axes[-1]:
        ax.set_xlabel("arc length s [m] (θ evaluated at the observation ahead of s)")
    axes[1, 0].legend(frameon=False, fontsize=8)
    fig.suptitle("Policy weights along the lap, θ(s) = π(o(s))", fontsize=10)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=130)
    plt.close(fig)
    for label, th in profiles.items():
        ratio = th.max(axis=0) / th.min(axis=0)
        print(f"{label}: max/min along the lap " + ", ".join(f"{n} {v:.3g}" for n, v in zip(THETA_NAMES, ratio)))
    print(f"written: {args.out}")


if __name__ == "__main__":
    main()
