#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Static weights vs the Diff-WMPC policy as an animated GIF (the RViz replay of replay_compare.py, drawn offline).

Both controllers drive one lap in the Python simulator from the same start (replay_compare.record: the plant, MPC
and evaluation of baseline_suite.py). Three views: the whole track, a true-scale view that follows the cars (a few cm
of lateral error are invisible on the whole track), and the lateral error n over time.

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/replay_gif.py
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/replay_gif.py --s0 3.42 --speed 0.25
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from baseline_suite import CONFIGS  # noqa: E402  (same folder)
from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from plannertrack_py.batch_plant import BatchPlant  # noqa: E402
from diff_weight_mpc.learning.policy_network import WeightPolicy  # noqa: E402
from diff_weight_mpc.mpc.controller import MpcController  # noqa: E402
from diff_weight_mpc.mpc.ocp import OcpConfig  # noqa: E402
from replay_compare import record, static_theta  # noqa: E402  (same folder)
from train_mpc import GRID_COLOR, REF_COLOR, SERIES  # noqa: E402  (same folder)

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent
DATA = RL_FRAMEWORK / "data"
INK = "#1f1f1d"
MUTED = "#6b6b66"


def at(run, t):
    """x, y, yaw of a run at time t, interpolated between simulation steps (held at the last step)."""
    f = min(t / run["dt"], len(run["x"]) - 1.0)
    k = min(int(f), len(run["x"]) - 2)
    a = f - k
    return tuple(run[c][k] + a * (run[c][k + 1] - run[c][k]) for c in ("x", "y", "yaw"))


def car_outline(x, y, yaw, length, width):
    """Closed outline of the body rectangle, centred on the simulated position (as the RViz car)."""
    c, s = np.cos(yaw), np.sin(yaw)
    pts = np.array([[length / 2, width / 2], [length / 2, -width / 2], [-length / 2, -width / 2],
                    [-length / 2, width / 2], [length / 2, width / 2]])
    return x + pts[:, 0] * c - pts[:, 1] * s, y + pts[:, 0] * s + pts[:, 1] * c


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--s0", type=float, default=1.711, help="start arc length [m] (as replay_compare.py)")
    ap.add_argument("--speed", type=float, default=0.25, help="playback speed (1 = real time)")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--window", type=float, default=1.0, help="half-width of the following view [m]")
    ap.add_argument("--hold", type=float, default=1.5, help="seconds the last frame is held")
    ap.add_argument("--config", default="nominal car", choices=[c[0] for c in CONFIGS])
    ap.add_argument("--static-run", default="train_static/theta0_sp1e-1_lat1_v1_400")
    ap.add_argument("--static-update", type=int, default=310)
    ap.add_argument("--policy-file", type=Path,
                    default=DATA / "train_policy" / "wmpc_hidden_none" / "policy_update140.pt")
    ap.add_argument("--out", type=Path, default=DATA / "conclusion_policy_vs_static" / "static_vs_policy.gif")
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    agent = yaml.safe_load((params / "agents.yaml").read_text())["agents"][args.agent]
    length, width = agent["geometry_params"]["length"], agent["geometry_params"]["width"]
    reference = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    _, perturbation, oracle = next(c for c in CONFIGS if c[0] == args.config)
    plant = BatchPlant.from_scenario(params, n_cars=1, agent_name=args.agent, reference=reference,
                                     perturbations=[perturbation])
    mpc = MpcController.create_controller(plant, cfg=OcpConfig())
    vehicle = [plant.plants[0].true_params if oracle else plant.plants[0].nominal_params]
    policy = WeightPolicy.load(args.policy_file)
    policy.eval()
    runs = [
        {"name": f"Static weights (update {args.static_update})", "color": SERIES[0],
         **record(plant, mpc, vehicle, args.s0, args.seed, theta=static_theta(args.static_run, args.static_update))},
        {"name": f"Diff-WMPC policy ({args.policy_file.stem.replace('policy_update', 'update ')})", "color": SERIES[1],
         **record(plant, mpc, vehicle, args.s0, args.seed, theta=None, theta_fn=policy)},
    ]
    for run in runs:
        run["t"] = np.arange(len(run["x"])) * run["dt"]
        print(f"{run['name']}: {len(run['x'])} steps, RMS |n| {100 * np.sqrt((run['n'] ** 2).mean()):.2f} cm, "
              f"max |n| {100 * np.abs(run['n']).max():.1f} cm, end {run['end_reason']}", flush=True)
    duration = max(r["t"][-1] for r in runs)

    # the track
    s = np.linspace(0.0, reference.length, 1200)
    r = reference.sample(s)
    normal = np.c_[-np.sin(r["psi"]), np.cos(r["psi"])]
    xy = np.c_[r["x"], r["y"]]
    left, right = xy + normal * r["w_left"][:, None], xy - normal * r["w_right"][:, None]

    fig = plt.figure(figsize=(10.0, 6.6), dpi=100)
    grid = fig.add_gridspec(2, 2, height_ratios=(2.2, 1.0), hspace=0.32, wspace=0.12,
                            left=0.06, right=0.98, top=0.83, bottom=0.08)
    ax_track, ax_zoom, ax_n = fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, :])
    for ax, title in ((ax_track, "Whole track"), (ax_zoom, f"Following the cars, true scale (±{args.window:g} m)")):
        ax.plot(*left.T, color=INK, lw=1.2)
        ax.plot(*right.T, color=INK, lw=1.2)
        ax.plot(*xy.T, color=REF_COLOR, lw=0.8, ls="--", alpha=0.8)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.set_title(title, loc="left", fontsize=10, color=INK)
    pad = 0.3
    ax_track.set_xlim(min(left[:, 0].min(), right[:, 0].min()) - pad, max(left[:, 0].max(), right[:, 0].max()) + pad)
    ax_track.set_ylim(min(left[:, 1].min(), right[:, 1].min()) - pad, max(left[:, 1].max(), right[:, 1].max()) + pad)

    ax_n.axhline(0.0, color=REF_COLOR, lw=0.8, ls="--")
    ax_n.set_xlim(0.0, duration)
    n_max = 100 * max(np.abs(rn["n"]).max() for rn in runs)
    ax_n.set_ylim(-1.15 * n_max, 1.15 * n_max)
    ax_n.set_xlabel("time [s]", fontsize=9, color=MUTED)
    ax_n.set_title("Lateral error from the raceline, n [cm]", loc="left", fontsize=10, color=INK)
    ax_n.grid(color=GRID_COLOR, lw=0.6)
    ax_n.tick_params(labelsize=8, colors=MUTED)
    for spine in ("top", "right"):
        ax_n.spines[spine].set_visible(False)

    artists = []
    for run in runs:
        c = run["color"]
        artists.append({
            "trail_track": ax_track.plot([], [], color=c, lw=1.6)[0],
            "dot_track": ax_track.plot([], [], "o", color=c, ms=6, mec="white", mew=0.8)[0],
            "trail_zoom": ax_zoom.plot([], [], color=c, lw=2.0)[0],
            "body": ax_zoom.fill([], [], color=c, alpha=0.35, ec=c, lw=1.5)[0],
            "n": ax_n.plot([], [], color=c, lw=1.8, label=run["name"])[0],
        })
    ax_n.legend(loc="upper right", fontsize=8, frameon=False, ncol=2)
    title = fig.text(0.06, 0.95, "", fontsize=12, color=INK, weight="bold")
    readout = fig.text(0.06, 0.905, "", fontsize=9.5, color=MUTED)

    frame_dt = args.speed / args.fps  # simulated seconds per frame
    n_frames = int(np.ceil(duration / frame_dt)) + 1 + int(args.hold * args.fps)

    def draw(i):
        t = min(i * frame_dt, duration)
        centre = []
        for run, a in zip(runs, artists):
            x, y, yaw = at(run, t)
            k = int(np.searchsorted(run["t"], t, side="right"))
            tx, ty = np.r_[run["x"][:k], x], np.r_[run["y"][:k], y]
            a["trail_track"].set_data(tx, ty)
            a["dot_track"].set_data([x], [y])
            a["trail_zoom"].set_data(tx, ty)
            a["body"].set_xy(np.c_[car_outline(x, y, yaw, length, width)])
            a["n"].set_data(run["t"][:k], 100 * run["n"][:k])
            centre.append((x, y))
        cx, cy = np.mean(centre, axis=0)
        ax_zoom.set_xlim(cx - args.window, cx + args.window)
        ax_zoom.set_ylim(cy - args.window, cy + args.window)
        title.set_text(f"Static weights vs Diff-WMPC policy, {args.config}, one lap from s = {args.s0:g} m")
        parts = []
        for run in runs:
            k = min(int(round(t / run["dt"])), len(run["n"]) - 1)
            worst = 100 * np.abs(run["n"][:k + 1]).max()
            parts.append(f"{run['name'].split(' (')[0]}: n {100 * run['n'][k]:+.1f} cm, max so far {worst:.1f} cm")
        readout.set_text(f"t = {t:4.2f} s  (playback {args.speed:g}×)     " + "     ".join(parts))
        return []

    args.out.parent.mkdir(parents=True, exist_ok=True)
    FuncAnimation(fig, draw, frames=n_frames, blit=False).save(args.out, writer=PillowWriter(fps=args.fps))
    print(f"written: {args.out} ({n_frames} frames, {args.out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
