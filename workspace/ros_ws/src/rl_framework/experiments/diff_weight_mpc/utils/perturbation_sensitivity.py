#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Step 4.8: which plant perturbations change the closed loop enough to study adaptation (Step 8)?

Grid: mu scale x "heavier car" scale s (Perturbation.heavier: mass, I_z x s; c_f, c_r, max_accel x 1/s).
Every grid point is driven from ``--starts`` evenly spaced start points, once with the nominal MPC model
and once with the oracle (MPC given the true parameters; the drive limit is not a model parameter, so the
oracle keeps the nominal bound). All cars run in one batched rollout, one lap each, with fixed θ0.

off_track does NOT end a lap here (end_on = too_slow, heading_out, diverged): a car that grazes the edge
finishes the lap and the violation is measured (time off track, depth), so RMS errors are comparable.

Per grid point and model: mean ± std over the start points of RMS |n|, RMS |dv|, the node-1 loss, lap time,
time off track and violation depth. A perturbation is "clear" (steps.md 4.8) if its nominal-model RMS |n| or
RMS |dv| changes against the unperturbed car consistently: every grid point uses the same start points, so
the change is taken per start point (paired) and is clear if |mean change| > 2 x its standard error. The
spread between start points (where a car starts, e.g. in a braking zone) does not enter. The oracle gap
(nominal model - oracle) is the room adaptation can have. Writes to <out>/sensitivity/:
results.csv, summary.log, heatmaps.png.
"""

import argparse
import csv
import itertools
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from plannertrack_py.batch_plant import BatchPlant  # noqa: E402
from diff_weight_mpc.closed_loop import compute_metrics, evenly_spaced_starts, run_episodes  # noqa: E402
from diff_weight_mpc.mpc.controller import MpcController  # noqa: E402
from diff_weight_mpc.mpc.ocp import OcpConfig  # noqa: E402
from plannertrack_py.plant import Perturbation  # noqa: E402

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent
METRICS = ("n_rms", "dv_rms", "loss_mean", "lap_time", "time_off_track", "violation_max")
SEQUENTIAL = "Blues"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--mu", type=float, nargs="+", default=[0.8, 0.9, 1.0])
    ap.add_argument("--heavier", type=float, nargs="+", default=[0.8, 1.0, 1.2])
    ap.add_argument("--starts", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--track-margin", type=float, default=OcpConfig().track_margin,
                    help="MPC-only tightening of the track limits [m]")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "closed_loop")
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    starts = evenly_spaced_starts(ref, args.starts)
    grid = list(itertools.product(args.mu, args.heavier))
    rows = [(mu, h, model, s0) for (mu, h) in grid for model in ("nominal", "oracle") for s0 in starts]
    perts = [Perturbation.heavier(h, mu_scale=mu) for mu, h, _, _ in rows]
    plant = BatchPlant.from_scenario(params, n_cars=len(rows), agent_name=args.agent, reference=ref,
                                     perturbations=perts)
    mpc = MpcController.create_controller(plant, cfg=OcpConfig(track_margin=args.track_margin))
    vehicle = [p.true_params if model == "oracle" else p.nominal_params
               for p, (_, _, model, _) in zip(plant.plants, rows)]
    print(f"{len(grid)} grid points x 2 models x {len(starts)} starts = {len(rows)} cars")
    log = run_episodes(plant, mpc, theta=np.array(OcpConfig().theta0), s0=[r[3] for r in rows], seed=args.seed,
                       vehicle_params=vehicle, end_on=("too_slow", "heading_out", "diverged"))
    per_car, summary = compute_metrics(log, plant)

    # aggregate over start points (lists in start order, for paired comparisons)
    agg = {}
    for (mu, h, model, _), r in zip(rows, per_car):
        a = agg.setdefault((mu, h, model), {m: [] for m in (*METRICS, "completed")})
        for m in METRICS:
            a[m].append(r[m])
        a["completed"].append(r["completed"])
    base = agg[(1.0, 1.0, "nominal")] if (1.0, 1.0, "nominal") in agg else None
    table = []
    for (mu, h) in grid:
        for model in ("nominal", "oracle"):
            a = agg[(mu, h, model)]
            row = {"mu_scale": mu, "heavier": h, "model": model, "laps_completed": int(sum(a["completed"])),
                   "cars": len(a["completed"])}
            for m in METRICS:
                v = np.asarray(a[m], dtype=float)
                row[f"{m}_mean"], row[f"{m}_std"] = float(np.nanmean(v)), float(np.nanstd(v))
            table.append(row)
    for row in table:
        if base is None or row["model"] != "nominal":
            row["clear"], row["delta_n_rms"], row["delta_dv_rms"] = "", "", ""
            continue
        clear = []
        a = agg[(row["mu_scale"], row["heavier"], "nominal")]
        for m in ("n_rms", "dv_rms"):
            d = np.asarray(a[m]) - np.asarray(base[m])  # paired: same start points
            row[f"delta_{m}"] = float(d.mean())
            sem = d.std(ddof=1) / np.sqrt(len(d)) if len(d) > 1 else np.inf
            if np.any(d != 0) and abs(d.mean()) > 2.0 * max(sem, 1e-12):
                clear.append(m)
        row["clear"] = "+".join(clear)
    by = {(r["mu_scale"], r["heavier"], r["model"]): r for r in table}
    for row in table:
        o = by[(row["mu_scale"], row["heavier"], "oracle")]
        for m in ("n_rms", "dv_rms", "loss_mean", "time_off_track"):
            row[f"oracle_gap_{m}"] = row[f"{m}_mean"] - o[f"{m}_mean"] if row["model"] == "nominal" else ""

    out = args.out / ("sensitivity" if args.track_margin == OcpConfig().track_margin
                      else f"sensitivity_tm{100 * args.track_margin:g}cm")
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        w.writerows(table)
    lines = [f"perturbation sensitivity: theta0 {tuple(OcpConfig().theta0)}, {len(starts)} starts per point, "
             f"one lap, off_track measured (not ending the lap), MPC track_margin {args.track_margin} m",
             f"controller: {summary['wall_time_per_car_ms']:.2f} ms per car per call, failures {summary['failures']}",
             "",
             " mu   heavier  model    laps  RMS|n| [m]      RMS|dv| [m/s]   loss          off-track [s]  depth [mm]  "
             "Δ vs nominal car RMS|n| / RMS|dv| (paired)  clear          oracle gap RMS|n| / RMS|dv|"]
    for r in table:
        gap = (f"{r['oracle_gap_n_rms']:+.4f} / {r['oracle_gap_dv_rms']:+.4f}" if r["model"] == "nominal" else "")
        delta = (f"{r['delta_n_rms']:+.4f} / {r['delta_dv_rms']:+.4f}" if r["model"] == "nominal" else "")
        lines.append(f" {r['mu_scale']:.2f}  {r['heavier']:.2f}    {r['model']:<8s} {r['laps_completed']}/{r['cars']}   "
                     f"{r['n_rms_mean']:.4f}±{r['n_rms_std']:.4f}  {r['dv_rms_mean']:.3f}±{r['dv_rms_std']:.3f}  "
                     f"{r['loss_mean_mean']:.3f}±{r['loss_mean_std']:.3f}  {r['time_off_track_mean']:6.3f}         "
                     f"{1e3 * r['violation_max_mean']:5.1f}       {delta:<43s} {r['clear']:<14s} {gap}")
    text = "\n".join(lines) + "\n"
    (out / "summary.log").write_text(text)
    print(text, end="")

    # heatmaps: nominal-model RMS |n|, oracle gap in RMS |n|, time off track (nominal model)
    mus, hs = sorted(set(args.mu)), sorted(set(args.heavier))
    def grid_of(key, model="nominal"):
        return np.array([[by[(mu, h, model)][key] for h in hs] for mu in mus], dtype=float)
    panels = [("n_rms_mean", "RMS |n| [m], nominal model", "nominal"),
              ("n_rms_mean", "RMS |n| [m], oracle", "oracle"),
              ("time_off_track_mean", "time off track [s], nominal model", "nominal"),
              ("dv_rms_mean", "RMS |dv| [m/s], nominal model", "nominal")]
    fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 3.8), constrained_layout=True)
    n_range = (0.0, max(np.nanmax(grid_of("n_rms_mean", m)) for m in ("nominal", "oracle")))
    for ax, (key, title, model) in zip(axes, panels):
        g = grid_of(key, model)
        vmin, vmax = n_range if key == "n_rms_mean" else (None, None)  # nominal vs oracle on one scale
        im = ax.imshow(g, cmap=SEQUENTIAL, origin="lower", aspect="auto", vmin=vmin, vmax=vmax)
        for i in range(len(mus)):
            for j in range(len(hs)):
                ax.text(j, i, f"{g[i, j]:.3f}", ha="center", va="center", fontsize=8,
                        color="white" if g[i, j] > np.nanmean(g) else "#1a1a19")
        ax.set_xticks(range(len(hs)), [f"{h:g}" for h in hs])
        ax.set_yticks(range(len(mus)), [f"{m:g}" for m in mus])
        ax.set_xlabel("heavier car scale")
        ax.set_ylabel("mu scale")
        ax.set_title(title, fontsize=9, loc="left")
        fig.colorbar(im, ax=ax, shrink=0.8)
    fig.savefig(out / "heatmaps.png", dpi=130)
    plt.close(fig)
    print(f"written: {out}/")


if __name__ == "__main__":
    main()
