#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Steps 4.4 / 4.5 / 4.7: the MPC with fixed weights in closed loop, one lap per car.

B cars start evenly spaced around the lap (or at --s0) and each drives one episode (one lap, or until a
failure flag or the time limit) with the plant's control rate (0.02 s) and SQP_RTI. Perturbation options
change the true car (plant only); --oracle gives the MPC the true parameters instead of the nominal ones.
Writes to <out>/<tag>/:

    log.npz         per-step arrays (EpisodeLog)
    metrics.csv     one row per car (tracking, safety, smoothness, loss, solver)
    summary.log     the summary and the per-car table
    *.png           track map, n(s) and v(s), box plots, input distributions, solve time

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/closed_loop_baseline.py                  # θ0, nominal
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/closed_loop_baseline.py --paper-theta
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/closed_loop_baseline.py --mu-scale 0.9 [--oracle]
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/closed_loop_baseline.py --heavier 1.2 [--oracle]
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from plannertrack_py.batch_plant import BatchPlant  # noqa: E402
from diff_weight_mpc.closed_loop import (  # noqa: E402
    compute_metrics,
    evenly_spaced_starts,
    observed_signals,
    run_episodes,
)
from diff_weight_mpc.mpc.controller import MpcController  # noqa: E402
from diff_weight_mpc.mpc.ocp import PAPER_THETA0, OcpConfig  # noqa: E402
from plannertrack_py.plant import PERTURBABLE, VX, Perturbation  # noqa: E402

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent

# chart palette (categorical order, fixed; neutral ink for limits and references)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
REF_COLOR, LIMIT_COLOR, GRID_COLOR = "#52514e", "#898781", "#e1e0d9"
DIVERGING = LinearSegmentedColormap.from_list("n_error", ["#1c5cab", "#f0efec", "#c33a39"])


def series_color(i):
    return SERIES[i % len(SERIES)]


def style(ax):
    ax.grid(True, color=GRID_COLOR, lw=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def write_csv(path, rows):
    keys = list(rows[0])
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (str(v) if isinstance(v, Perturbation) else v) for k, v in r.items()})


def summary_text(tag, summary, per_car, theta, oracle):
    lines = [f"{tag}", f"theta = {tuple(round(float(t), 4) for t in theta)}   oracle = {oracle}"]
    lines += [f"  {k}: {v}" for k, v in summary.items()]
    lines.append("")
    lines.append(" car    s0  end        lap[s]  |n| rms   max   |dv| rms  margin min @ s   "
                 "jerk>8 cmd/obs  ω>0.25 cmd/obs  ax obs min/max   loss    fail")
    for r in per_car:
        lines.append(f" {r['car']:3d} {r['s0']:5.2f}  {r['end_reason']:<10s} {r['lap_time']:6.2f}  {r['n_rms']:7.3f} "
                     f"{r['n_max']:5.3f}  {r['dv_rms']:7.3f}  {r['margin_min']:6.3f} @ {r['margin_min_s']:5.1f}   "
                     f"{r['jerk_cmd_above_th']:5.2f}/{r['jerk_obs_above_th']:4.2f}     "
                     f"{r['steer_rate_cmd_above_th']:5.2f}/{r['steer_rate_obs_above_th']:4.2f}     "
                     f"{r['ax_obs_min']:6.2f}/{r['ax_obs_max']:5.2f}   {r['loss_mean']:6.3g}  "
                     f"{r['failures']:3d}")
    return "\n".join(lines) + "\n"


def _worst_moment(log):
    """(car, step) of the smallest margin over all active steps."""
    m = np.where(log.active, log.margin, np.inf)
    k, i = np.unravel_index(np.argmin(m), m.shape)
    return int(i), int(k)


def _draw_track(ax, ref, half_width, lw=1.0):
    """Track edges (solid) and the car-side limits (dotted): the car's CENTRE must stay inside the dotted
    lines, i.e. half the car width (plus nothing) inside the edges; off_track is defined on them."""
    rl = ref.raceline
    xs, ys = np.append(rl.x, rl.x[0]), np.append(rl.y, rl.y[0])
    psi = np.append(rl.psi, rl.psi[0])
    nx_, ny_ = -np.sin(psi), np.cos(psi)  # left normal
    for w, sign in ((np.append(rl.w_left, rl.w_left[0]), 1), (np.append(rl.w_right, rl.w_right[0]), -1)):
        ax.plot(xs + sign * w * nx_, ys + sign * w * ny_, color=LIMIT_COLOR, lw=lw,
                label="track edge" if sign > 0 else None)
        wc = w - half_width
        ax.plot(xs + sign * wc * nx_, ys + sign * wc * ny_, color=LIMIT_COLOR, lw=0.8 * lw, ls=":",
                label="car-side limit (centre must stay inside)" if sign > 0 else None)
    ax.plot(xs, ys, color=REF_COLOR, lw=0.8, ls="--", label="reference")


def _footprint(ax, x, y, psi, length, width, **kw):
    c, s = np.cos(psi), np.sin(psi)
    corners = np.array([[length / 2, width / 2], [length / 2, -width / 2], [-length / 2, -width / 2],
                        [-length / 2, width / 2], [length / 2, width / 2]])
    pts = corners @ np.array([[c, s], [-s, c]]) + [x, y]
    ax.plot(pts[:, 0], pts[:, 1], **kw)


def plot_track(log, ref, per_car, path, title, half_width, car_length=0.58, zoom=0.6):
    """Left: the whole lap, paths coloured by the lateral error, off-track points marked. Right: zoom
    on the moment of the smallest margin, with the car's footprint there (a violation of a few mm is
    invisible at the scale of the whole track)."""
    fig, (ax, az) = plt.subplots(1, 2, figsize=(13, 8.5), constrained_layout=True,
                                 gridspec_kw={"width_ratios": [1.0, 1.15]})
    lim = max(0.05, float(np.nanmax(np.abs(log.n[log.active]))))
    norm = TwoSlopeNorm(vmin=-lim, vcenter=0.0, vmax=lim)
    wi, wk = _worst_moment(log)
    wx, wy, wpsi = log.state[wk, wi, 0], log.state[wk, wi, 1], log.state[wk, wi, 2]
    worst_margin = float(log.margin[wk, wi])
    for a, lw, size in ((ax, 1.0, 4), (az, 1.5, 16)):
        _draw_track(a, ref, half_width, lw=lw)
        sc = None
        for i in range(log.active.shape[1]):
            m = log.active[:, i]
            sc = a.scatter(log.state[m, i, 0], log.state[m, i, 1], c=log.n[m, i], cmap=DIVERGING, norm=norm,
                           s=size, linewidths=0)
        off = log.active & (log.margin < 0)
        if off.any():
            a.scatter(log.state[..., 0][off], log.state[..., 1][off], marker="x", s=3 * size, color="#1a1a19",
                      lw=1.2, label=f"off track ({int(off.any(axis=0).sum())} cars)")
        a.set_aspect("equal")
        a.set_xlabel("x [m]")
        a.set_ylabel("y [m]")
        style(a)
    side = "outside" if worst_margin < 0 else "inside"
    ax.scatter([wx], [wy], s=220, facecolors="none", edgecolors=REF_COLOR, lw=1.5,
               label=f"smallest margin: car {wi}, {abs(100 * worst_margin):.1f} cm {side} (s = {log.s[wk, wi]:.1f} m)")
    _footprint(az, wx, wy, wpsi, car_length, 2 * half_width, color=REF_COLOR, lw=1.8,
               label=f"car {wi} at its smallest margin ({100 * worst_margin:+.1f} cm)")
    az.set_xlim(wx - zoom, wx + zoom)
    az.set_ylim(wy - zoom, wy + zoom)
    az.set_title(f"zoom: car {wi}, side {abs(100 * worst_margin):.1f} cm {side} the track edge", fontsize=9,
                 loc="left")
    fig.colorbar(sc, ax=az, shrink=0.7, label="lateral error n [m] (> 0 left)")
    ax.legend(loc="upper left", fontsize=7, frameon=False)
    az.legend(loc="upper left", fontsize=7, frameon=False)
    ax.set_title(title.replace(", median RMS", ",\nmedian RMS"), fontsize=9, loc="left")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _break_at_wrap(s, *ys, length):
    """Time-ordered series with NaN inserted where s wraps at the start line (no line across the lap)."""
    cut = np.flatnonzero(np.diff(s) < -0.5 * length) + 1
    return [np.insert(np.asarray(a, dtype=float), cut, np.nan) for a in (s, *ys)]


def plot_along_s(log, plant, path, title):
    ref = plant.reference
    s_grid = np.linspace(0, ref.length, 400)
    lim = np.array([plant.plants[0].track_limits(s) for s in s_grid])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
    ax = axes[0]
    ax.plot(s_grid, lim[:, 0], color=LIMIT_COLOR, lw=1, ls="--", label="track bound (car side)")
    ax.plot(s_grid, lim[:, 1], color=LIMIT_COLOR, lw=1, ls="--")
    ax.axhline(0, color=REF_COLOR, lw=0.8)
    ax2 = axes[1]
    ax2.plot(s_grid, ref.sample(s_grid)["vx"], color=REF_COLOR, lw=2, label="v_ref")
    for i in range(log.active.shape[1]):
        m = log.active[:, i]
        s, n, v = _break_at_wrap(log.s[m, i], log.n[m, i], log.state[m, i, VX], length=ref.length)
        ax.plot(s, n, color=series_color(i), lw=1.5, label=f"car {i}")
        ax2.plot(s, v, color=series_color(i), lw=1.2)
    ax.set_ylabel("lateral error n [m]")
    ax.legend(loc="upper left", fontsize=7, frameon=False, ncol=5)
    ax2.set_ylabel("speed vx [m/s]")
    ax2.set_xlabel("arc length s [m]")
    ax2.legend(loc="upper left", fontsize=8, frameon=False)
    axes[0].set_title(title, fontsize=10, loc="left")
    for a in axes:
        style(a)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_boxes(log, path, title):
    B = log.active.shape[1]
    n = [np.abs(log.n[log.active[:, i], i]) for i in range(B)]
    dv = [np.abs(log.dv[log.active[:, i], i]) for i in range(B)]
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), constrained_layout=True)
    for ax, data, label in ((axes[0], n, "|lateral deviation| [m]"), (axes[1], dv, "|v − v_ref| [m/s]")):
        bp = ax.boxplot(data, vert=False, labels=[f"car {i}" for i in range(B)], widths=0.6, whis=(0, 100),
                        patch_artist=True, medianprops=dict(color=REF_COLOR))
        for i, box in enumerate(bp["boxes"]):
            box.set_facecolor(series_color(i))
            box.set_alpha(0.55)
        ax.set_xlabel(label)
        style(ax)
    axes[0].set_title(title + "  (whiskers: min / max)", fontsize=10, loc="left")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_inputs(log, plant, path, title, jerk_th=8.0, steer_rate_th=0.25):
    """Distributions of jerk and steering rate: commanded (the MPC's input) and observed (from the car's
    states, observed_signals) on the same bins."""
    u = log.u[log.active]
    obs = [observed_signals(log.state[log.active[:, i], i], log.dt) for i in range(log.active.shape[1])]
    observed = {"jerk": np.concatenate([o["jerk"] for o in obs]),
                "steer_rate": np.concatenate([o["steer_rate"] for o in obs])}
    lim = plant.limits
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), constrained_layout=True)
    for ax, col, key, th, bound, label in (
            (axes[0], 0, "jerk", jerk_th, lim.max_jerk, "jerk [m/s³]"),
            (axes[1], 1, "steer_rate", steer_rate_th, lim.max_steer_rate, "steering rate [rad/s]")):
        both = np.concatenate([u[:, col], observed[key]])
        bins = np.linspace(np.percentile(both, 0.5), np.percentile(both, 99.5), 60)
        ax.hist(u[:, col], bins=bins, color=SERIES[0], alpha=0.6, label="commanded (MPC input)")
        ax.hist(observed[key], bins=bins, histtype="step", lw=1.8, color=SERIES[1],
                label="observed (from the car's states)")
        for sign in (-1, 1):
            ax.axvline(sign * th, color=SERIES[1], lw=1.5, ls="--", label="loss threshold" if sign > 0 else None)
            ax.axvline(sign * bound, color=LIMIT_COLOR, lw=1, ls=":", label="input bound" if sign > 0 else None)
        ax.set_xlabel(label)
        ax.set_ylabel("steps")
        ax.legend(fontsize=8, frameon=False)
        style(ax)
    axes[0].set_title(title, fontsize=10, loc="left")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_solve_time(log, path, title):
    fig, ax = plt.subplots(figsize=(10, 3), constrained_layout=True)
    ax.plot(log.t, 1e3 * log.wall_time, color=SERIES[0], lw=1)
    ax.axhline(1e3 * log.dt, color=LIMIT_COLOR, ls="--", lw=1, label=f"control interval {1e3 * log.dt:.0f} ms")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("controller call [ms]\n(all cars)")
    ax.legend(fontsize=8, frameon=False)
    ax.set_title(title, fontsize=10, loc="left")
    style(ax)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--cars", type=int, default=8, help="cars, evenly spaced starts")
    ap.add_argument("--s0", type=float, nargs="+", default=None, help="explicit start arc lengths")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--theta", type=float, nargs=6, default=None, help="q_n q_psi q_v q_alat r_j r_omega")
    ap.add_argument("--paper-theta", action="store_true", help="the paper's θ0 (Table I)")
    for name in (*PERTURBABLE, "max_accel"):
        ap.add_argument(f"--{name}-scale", dest=f"{name}_scale", type=float, default=1.0,
                        help=f"true {name} / nominal (plant only)")
    ap.add_argument("--heavier", type=float, default=None,
                    help="Perturbation.heavier(s): mass, Iz x s; cf, cr, max_accel x 1/s (overrides those)")
    ap.add_argument("--oracle", action="store_true", help="MPC gets the true parameters")
    ap.add_argument("--track-margin", type=float, default=OcpConfig().track_margin,
                    help="MPC-only tightening of the track limits [m] (plant flags use the real limits)")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "closed_loop")
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    if args.heavier is not None:
        pert = Perturbation.heavier(args.heavier, mu_scale=args.mu_scale)
    else:
        pert = Perturbation(**{f"{n}_scale": getattr(args, f"{n}_scale") for n in (*PERTURBABLE, "max_accel")})
    s0 = np.array(args.s0) if args.s0 else evenly_spaced_starts(ref, args.cars)
    B = len(s0)
    plant = BatchPlant.from_scenario(params, n_cars=B, agent_name=args.agent, reference=ref,
                                     perturbations=[pert] * B)
    theta = np.array(PAPER_THETA0 if args.paper_theta else (args.theta or OcpConfig().theta0), dtype=float)
    cfg = OcpConfig(track_margin=args.track_margin)
    mpc = MpcController.create_controller(plant, cfg=cfg, log_residuals=True)
    vehicle = [p.true_params for p in plant.plants] if args.oracle else None

    log = run_episodes(plant, mpc, theta=theta, s0=s0, seed=args.seed, laps=args.laps, vehicle_params=vehicle)
    per_car, summary = compute_metrics(log, plant)

    tag = args.tag or ("paper_theta" if args.paper_theta else "theta0") + (
        "" if pert.is_nominal else "_" + "_".join(f"{k.replace('_scale', '')}{v:g}" for k, v in
                                                  pert.__dict__.items() if v != 1.0)) + ("_oracle" if args.oracle else "") + (
        "" if args.track_margin == OcpConfig().track_margin else f"_tm{100 * args.track_margin:g}cm")
    out = args.out / tag
    out.mkdir(parents=True, exist_ok=True)
    log.save_npz(out / "log.npz")
    write_csv(out / "metrics.csv", per_car)
    text = summary_text(tag, summary, per_car, theta, args.oracle) + f"track_margin (MPC only) = {cfg.track_margin} m\n"
    (out / "summary.log").write_text(text)
    title = (f"{tag}: {summary['completed']}/{summary['cars']} laps, median lap {summary['lap_time_median']:.2f} s "
             f"(ref {summary['reference_lap_time']:.2f} s), median RMS |n| {summary['n_rms_median']:.3f} m")
    geometry = plant.plants[0]._agent.get("geometry_params", {})
    plot_track(log, ref, per_car, out / "track.png", title, half_width=0.5 * plant.limits.vehicle_width,
               car_length=float(geometry.get("length", 0.58)))
    plot_along_s(log, plant, out / "along_s.png", title)
    plot_boxes(log, out / "deviation_boxes.png", title)
    plot_inputs(log, plant, out / "inputs.png", title)
    plot_solve_time(log, out / "solve_time.png", title)
    print(text, end="")
    print(f"written: {out}/")


if __name__ == "__main__":
    main()
