#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Step 3.6 sanity run: one lap on the reference's feed-forward only, no feedback.

The plant starts on the reference (trimmed) and gets the open-loop inputs of
plannertrack_py.feedforward (jerk and steering rate that follow the steady cornering state of the
NOMINAL model along the timed reference, rate-limited to max_jerk / max_steer_rate so the plant
never clips them). The run ends at the first episode flag (off_track,
too_slow, heading_out, diverged) or after one lap. It shows how long the car stays on the line
without feedback, i.e. how much the MPC has to correct. Writes, per start point,

    <out>/feedforward_<scenario>_s<s0>.png   lateral error, speed, heading errors, steering, acceleration
    <out>/feedforward_<scenario>_s<s0>.npz   the run (states, info fields, schedule)
    <out>/feedforward_<scenario>.log         one summary line per start point

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/feedforward_lap.py --s0 0 5 12
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/feedforward_lap.py --mu-scale 0.9   # mismatch
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/feedforward_lap.py --mass-scale 1.2 --Iz-scale 1.2 \
        --cf-scale 0.833 --cr-scale 0.833   # heavier car, fixed physical tire stiffness
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from plannertrack_py.feedforward import feedforward_schedule, run_open_loop  # noqa: E402
from plannertrack_py.plant import ACC, DELTA, PERTURBABLE, VX, Perturbation, Plant  # noqa: E402

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent

PLANT_COLOR = "#2a78d6"  # categorical slot 1: what the plant did
FF_COLOR = "#eb6834"  # categorical slot 2: reference / feed-forward
LIMIT_COLOR = "#8a8a85"  # bounds and limits (recessive)


def summarize(run, schedule, plant, s0):
    n = np.abs(run["n"])
    first = lambda mask: (f"{run['t'][np.flatnonzero(mask)[0]]:.1f} s" if mask.any() else "never")  # noqa: E731
    k = len(run["t"])
    lag = np.abs(schedule.acc_target[: k + 1] - schedule.acc[: k + 1])
    return (f"s0 {s0:6.2f} m | {run['reason']:11s} after {run['t'][-1]:4.1f} s, {run['progress'][-1]:5.2f} m "
            f"of {plant.reference.length:.2f} | |n| > 5 cm at {first(n > 0.05)}, > 10 cm at {first(n > 0.10)} | "
            f"max |n| {n.max():.3f} m | jerk-limited in {int(np.sum(lag > 1e-9))}/{k} steps "
            f"(max a lag {lag.max():.2f} m/s^2) | "
            f"trims ok {bool(schedule.converged.all())}")


def plot(run, schedule, plant, s0, title, path):
    t, lim = run["t"], plant.limits
    k = len(t)
    t_ff = schedule.t
    n_left, n_right = np.array([plant.track_limits(s) for s in run["s"]]).T

    fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True, constrained_layout=True)
    ax = axes[0]
    ax.plot(t, run["n"], color=PLANT_COLOR, lw=2, label="plant")
    ax.plot(t, n_left, color=LIMIT_COLOR, lw=1, ls="--", label="track bound (car side)")
    ax.plot(t, n_right, color=LIMIT_COLOR, lw=1, ls="--")
    ax.axhline(0.0, color=LIMIT_COLOR, lw=0.8)
    ax.set_ylabel("lateral error n [m]\n(> 0 left)")

    ax = axes[1]
    ax.plot(t, run["states"][:, VX], color=PLANT_COLOR, lw=2, label="plant vx")
    ax.plot(t, run["v_ref"], color=FF_COLOR, lw=2, label="v_ref at the car's s")
    ax.set_ylabel("speed [m/s]")

    ax = axes[2]
    ax.plot(t, np.degrees(run["dpsi"]), color=PLANT_COLOR, lw=2, label="dpsi (body)")
    ax.plot(t, np.degrees(run["dchi"]), color=FF_COLOR, lw=2, label="dchi (course)")
    ax.axhline(0.0, color=LIMIT_COLOR, lw=0.8)
    ax.set_ylabel("heading error [deg]")

    ax = axes[3]
    ax.plot(t_ff[: k + 1], schedule.delta_target[: k + 1], color=FF_COLOR, lw=2, label="steady-state steer")
    ax.plot(t, run["states"][:, DELTA], color=PLANT_COLOR, lw=2, ls="--", label="plant steer (= feed-forward)")
    for sign in (-1, 1):
        ax.axhline(sign * lim.max_steer, color=LIMIT_COLOR, lw=1, ls="--", label="max_steer" if sign > 0 else None)
    ax.set_ylabel("steering [rad]")

    ax = axes[4]
    ax.plot(t_ff[: k + 1], schedule.acc_target[: k + 1], color=FF_COLOR, lw=2, label="steady-state a (a_x,ref - vy r)")
    ax.plot(t, run["states"][:, ACC], color=PLANT_COLOR, lw=2, ls="--", label="plant a (jerk-limited feed-forward)")
    ax.axhline(lim.max_accel, color=LIMIT_COLOR, lw=1, ls="--", label="drive / braking limit")
    ax.axhline(-lim.max_decel, color=LIMIT_COLOR, lw=1, ls="--")
    ax.set_ylabel("acceleration state [m/s^2]")
    ax.set_xlabel("time [s]")

    for ax in axes:
        ax.axvline(t[-1], color=LIMIT_COLOR, lw=1, ls=":")
        ax.grid(True, color="#e6e6e3", lw=0.6)
        ax.legend(loc="upper left", fontsize=8, frameon=False)
    axes[0].set_title(title, fontsize=10, loc="left")
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl", help="folder under src/scenarios")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--s0", type=float, nargs="+", default=[0.0, 5.0, 12.0], help="start arc lengths [m]")
    for name in PERTURBABLE:  # --mass-scale, --mu-scale, --Iz-scale, --cf-scale, --cr-scale
        ap.add_argument(f"--{name}-scale", dest=f"{name}_scale", type=float, default=1.0,
                        help=f"true {name} / nominal (plant only; the feed-forward uses nominal values)")
    ap.add_argument("--dt", type=float, default=0.1, help="input interval = MPC interval [s]")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "feedforward")
    args = ap.parse_args()

    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    # feed-forward from the nominal model (the controller's knowledge); the plant may be perturbed
    nominal = Plant.from_scenario(params, args.agent, reference=ref, dt=args.dt)
    plant = Plant.from_scenario(params, args.agent, reference=ref, dt=args.dt,
                                perturbation=Perturbation(**{f"{n}_scale": getattr(args, f"{n}_scale")
                                                             for n in PERTURBABLE}))
    n_steps = int(np.ceil(1.2 * ref.raceline.lap_time / plant.dt))

    args.out.mkdir(parents=True, exist_ok=True)
    tag = f"feedforward_{args.scenario}"
    if not plant.perturbation.is_nominal:
        tag += "".join(f"_{n}{s:g}" for n, s in plant.perturbation.scales().items() if s != 1.0)
    lines = [f"{tag}: lap {ref.length:.2f} m / {ref.raceline.lap_time:.3f} s, dt {plant.dt} s, "
             f"perturbation {plant.perturbation}"]
    for s0 in args.s0:
        lim = nominal.limits
        schedule = feedforward_schedule(ref, nominal.sim.xdot, nominal.nominal_params.wheelbase,
                                        lim.max_steer, s0, n_steps, plant.dt,
                                        max_jerk=lim.max_jerk, max_steer_rate=lim.max_steer_rate)
        _, info0 = plant.reset(s0=s0)
        run = run_open_loop(plant, schedule)
        line = summarize(run, schedule, plant, s0)
        lines.append(line)
        print(line)
        stem = args.out / f"{tag}_s{s0:g}"
        plot(run, schedule, plant, s0, f"{tag}, s0 = {s0:g} m: {run['reason']} after {run['t'][-1]:.1f} s "
             f"({run['progress'][-1]:.1f} m)", stem.with_suffix(".png"))
        np.savez(stem.with_suffix(".npz"), **{k: v for k, v in run.items() if k != "reason"},
                 reason=run["reason"], ff_t=schedule.t, ff_s=schedule.s, ff_delta=schedule.delta,
                 ff_acc=schedule.acc, ff_delta_target=schedule.delta_target, ff_acc_target=schedule.acc_target,
                 ff_u=schedule.u, trim_converged0=info0["trim_converged"])
    log = args.out / f"{tag}.log"
    log.write_text("\n".join(lines) + "\n")
    print(f"written: {args.out}/{tag}_s*.png, .npz, {log.name}")


if __name__ == "__main__":
    main()
