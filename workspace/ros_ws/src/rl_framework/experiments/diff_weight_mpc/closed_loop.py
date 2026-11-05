# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Closed loop: BatchPlant + MpcController, step by step (steps.md Step 4.4).

:func:`rollout` is the single loop used for evaluation and (Steps 6-7) training. Every iteration is one
control step (the plant's dt, 0.02 s) and yields a :class:`StepData`: B data points, one per car, with
the MPC solution tensors (graph intact) and the stage references of that solve. Training computes the
loss and calls ``backward()`` on each StepData and steps the optimizer every ``accumulate_steps``
(paper Algorithm 1); evaluation records the steps into an :class:`EpisodeLog` (:func:`run_episodes`)
and summarizes it (:func:`compute_metrics`).

Episodes: a car's episode ends when it completes ``laps`` laps, raises a failure flag (off_track,
too_slow, heading_out, diverged) or reaches ``max_episode_steps``. Then the car is reset (new episode)
in both modes; with ``on_done="stop"`` its row is no longer *active* (not a data point any more, and
the rollout ends when no car is active), with ``on_done="reset"`` it keeps producing data points.

Example (evaluation)::

    plant = BatchPlant.from_scenario(PARAMS_DIR, n_cars=8, reference=ref)          # dt 0.02
    mpc = MpcController.create_controller(plant, log_residuals=True)
    log = run_episodes(plant, mpc, s0=evenly_spaced_starts(ref, 8))
    per_car, summary = compute_metrics(log, plant)
"""

from dataclasses import dataclass, field

import numpy as np
import torch

from diff_weight_mpc.learning.loss import LossConfig, node_loss
from plannertrack_py.plant import ACC, DELTA, FAILURE_FLAGS, NU, NX, VX, VY, R

END_REASONS = ("lap_done", *FAILURE_FLAGS, "time_limit")


def evenly_spaced_starts(reference, n_cars, offset=0.0):
    """n_cars arc lengths evenly spaced around the lap (every corner reached at a different phase)."""
    return reference.wrap(offset + reference.length * np.arange(n_cars) / n_cars)


@dataclass
class StepData:
    """One control step = B data points (one per car)."""

    k: int  # control step index of the rollout
    t: float  # time since the start of the rollout [s]
    obs: np.ndarray  # (B, 10) policy observation (5 v_ref, 5 kappa ahead) at the solve, or None
    theta: object  # weights used: torch (B, 6) (graph to the policy, if any) or numpy
    u: np.ndarray  # (B, NU) inputs applied (after the controller's failure fallback)
    mpc_info: dict  # MpcController.step info (status, fallback, x_plan, residuals, ...)
    solution: object  # MpcController.last: ctx, u0, x, u, value tensors (graph to theta), params
    state: np.ndarray  # (B, NX) plant state after the step (before any reset)
    plant_info: dict  # BatchPlant.step info (B,) arrays: s, n, dpsi, dv, flags, ...
    active: np.ndarray  # (B,) bool: the row is a data point (its car was in an episode)
    episode_end: np.ndarray  # (B,) bool: the car's episode ended at this step (then it was reset)
    end_reason: list  # (B,) END_REASONS entry where episode_end, else None
    episode_step: np.ndarray  # (B,) step index within the car's episode (0 = first step)
    episode_s0: np.ndarray  # (B,) arc length at which each car's current episode started


def rollout(
    plant,
    controller,
    theta=None,
    theta_fn=None,
    s0=None,
    seed=None,
    laps=1,
    max_episode_steps=None,
    max_steps=None,
    on_done="stop",
    vehicle_params=None,
    observe=False,
    end_on=FAILURE_FLAGS,
):
    """Drive the batch, yielding one :class:`StepData` per control step.

    Args:
        plant: BatchPlant (B cars, its dt is the control interval).
        controller: MpcController for the same B.
        theta: fixed weights (6,) or (B, 6) (numpy or torch); None = the OCP's theta0.
        theta_fn: ``theta_fn(obs, k) -> (B, 6)`` instead of ``theta`` (a policy); implies ``observe``.
        s0, seed: start arc lengths (B,) / seed of the first reset (as BatchPlant.reset). Later resets
            (new episodes) restart at the car's first s0 if given, else at a random point.
        laps: laps per episode; max_episode_steps: time limit per episode (None: 1.5 reference laps
            per lap); max_steps: stop the rollout after this many steps (None: until no car is active).
        on_done: "stop" (evaluation: a car is a data point for one episode) or "reset" (training).
        vehicle_params: None (nominal model) or a list of B VehicleParams passed to the MPC (oracle rows
            get their car's true parameters).
        observe: compute the policy observation every step.
        end_on: failure flags that end an episode (default all FAILURE_FLAGS). E.g. without
            "off_track" a car that grazes the edge finishes its lap and the violation is measured
            (EpisodeLog.margin < 0) instead of cutting the episode short.
    """
    if on_done not in ("stop", "reset"):
        raise ValueError(f"on_done must be 'stop' or 'reset', got {on_done!r}")
    B =  plant.n_cars
    ref = plant.reference
    dt = plant.dt     # sim-time-step
    if max_episode_steps is None:
        max_episode_steps = int(np.ceil(1.5 * laps * ref.raceline.lap_time / dt)) # max-episode steps allowed, given sim-dt and reference lap time
    first_s0 = None if s0 is None else np.broadcast_to(np.asarray(s0, dtype=float), (B,)).copy()
    observe = observe or theta_fn is not None

    x, info = plant.reset(seed=seed, s0=first_s0)
    s = np.asarray(info["s0"], dtype=float).copy()
    episode_s0 = s.copy()
    controller.reset()
    active = np.ones(B, dtype=bool)
    episode_step = np.zeros(B, dtype=int)
    k = 0
    while active.any() and (max_steps is None or k < max_steps):
        obs = np.concatenate(ref.policy_observation(s), axis=1) if observe else None
        th = theta_fn(obs, k) if theta_fn is not None else theta # A learning based theta or fixed theta parameter.
        u, mpc_info = controller.step(x, s, theta=th, vehicle_params=vehicle_params)
        x, pinfo = plant.step(u)
        state = x.copy()

        done_laps = pinfo["progress"] >= laps * ref.length
        reason = [None] * B
        for i in range(B):
            failed = [f for f in end_on if pinfo[f][i]]
            if failed:
                reason[i] = failed[0]
            elif done_laps[i]:
                reason[i] = "lap_done"
            elif episode_step[i] + 1 >= max_episode_steps:
                reason[i] = "time_limit"
        episode_end = np.array([r is not None for r in reason])

        yield StepData(
            k=k,
            t=(k + 1) * dt,
            obs=obs,
            theta=th,
            u=u,
            mpc_info=mpc_info,
            solution=controller.last,
            state=state,
            plant_info=pinfo,
            active=active.copy(),
            episode_end=episode_end & active,
            end_reason=[r if a else None for r, a in zip(reason, active)],
            episode_step=episode_step.copy(),
            episode_s0=episode_s0.copy(),
        )

        s = np.asarray(pinfo["s"], dtype=float).copy()
        episode_step += 1
        ended = np.flatnonzero(episode_end)
        if ended.size:
            # new episode for these cars: a car that left the track must not keep driving off-track
            x_new, rinfo = plant.reset_cars(ended, s0=None if first_s0 is None else first_s0[ended])
            x[ended] = x_new
            s[ended] = rinfo["s0"]
            episode_s0[ended] = rinfo["s0"]
            controller.reset(ended)
            episode_step[ended] = 0
            if on_done == "stop":
                active[ended] = False
        k += 1


@dataclass
class EpisodeLog:
    """Per-step arrays of an evaluation rollout (one episode per car), (T, B, ...); rows after a car's
    episode ended are masked (``active`` False)."""

    t: np.ndarray  # (T,)
    active: np.ndarray  # (T, B)
    state: np.ndarray  # (T, B, NX)
    u: np.ndarray  # (T, B, NU)
    theta: np.ndarray  # (T, B, 6)
    s: np.ndarray  # (T, B)
    n: np.ndarray
    dpsi: np.ndarray
    dchi: np.ndarray
    dv: np.ndarray
    v_ref: np.ndarray
    progress: np.ndarray
    margin: np.ndarray  # (T, B) distance of the car's side to the nearer track bound [m]
    loss: np.ndarray  # (T, B) task loss at node 1 of the plan (LossConfig)
    status: np.ndarray  # (T, B) acados status
    fallback: np.ndarray  # (T, B)
    cold_start: np.ndarray  # (T, B)
    residuals: np.ndarray  # (T, B, 4) or empty
    wall_time: np.ndarray  # (T,) controller time per call (all cars)
    s0: np.ndarray  # (B,)
    end_step: np.ndarray  # (B,) last active step of each car's episode
    end_reason: list  # (B,)
    dt: float
    lap_length: float
    meta: dict = field(default_factory=dict)

    def save_npz(self, path):
        arrays = {k: v for k, v in self.__dict__.items() if isinstance(v, np.ndarray)}
        np.savez(
            path,
            **arrays,
            end_reason=np.array(self.end_reason),
            dt=self.dt,
            lap_length=self.lap_length,
        )


def _no_grad(theta_fn):
    """theta_fn evaluated without building an autograd graph (evaluation only)."""

    def wrapped(obs, k):
        with torch.no_grad():
            return theta_fn(obs, k)

    return wrapped


def run_episodes(
    plant,
    controller,
    theta=None,
    s0=None,
    seed=None,
    laps=1,
    max_episode_steps=None,
    vehicle_params=None,
    loss_cfg=LossConfig(),
    end_on=FAILURE_FLAGS,
    theta_fn=None,
):
    """One episode per car (``on_done="stop"``), recorded into an :class:`EpisodeLog`. ``theta_fn(obs, k) -> (B, 6)``
    (a weight policy, evaluated without a graph) instead of a fixed ``theta``."""
    B = plant.n_cars
    keys = ("s", "n", "dpsi", "dchi", "dv", "v_ref", "progress")
    rec = {
        k: []
        for k in (
            "t",
            "active",
            "state",
            "u",
            "theta",
            "margin",
            "loss",
            "status",
            "fallback",
            "cold_start",
            "residuals",
            "wall_time",
            *keys,
        )
    }
    end_step = np.full(B, -1)
    end_reason = [None] * B
    s0_used = None
    for sd in rollout(
        plant,
        controller,
        theta=theta,
        theta_fn=None if theta_fn is None else _no_grad(theta_fn),
        s0=s0,
        seed=seed,
        laps=laps,
        max_episode_steps=max_episode_steps,
        on_done="stop",
        vehicle_params=vehicle_params,
        end_on=end_on,
    ):
        if s0_used is None:
            s0_used = sd.episode_s0.copy()
        pi = sd.plant_info
        rec["t"].append(sd.t)
        rec["active"].append(sd.active)
        rec["state"].append(sd.state)
        rec["u"].append(sd.u)
        th = sd.theta
        if th is None:
            th = np.asarray(controller.cfg.theta0, dtype=float)
        th = (
            th.detach().cpu().numpy()
            if isinstance(th, torch.Tensor)
            else np.asarray(th, dtype=float)
        )
        rec["theta"].append(np.broadcast_to(th, (B, th.shape[-1])).copy())
        for key in keys:
            rec[key].append(np.asarray(pi[key], dtype=float))
        limits = np.array([p.track_limits(si) for p, si in zip(plant.plants, pi["s"])])
        rec["margin"].append(np.minimum(limits[:, 0] - pi["n"], pi["n"] - limits[:, 1]))
        loss, _ = node_loss(
            sd.solution.x.detach(), sd.solution.u.detach(), sd.solution.params, loss_cfg
        )
        rec["loss"].append(loss.cpu().numpy())
        mi = sd.mpc_info
        rec["status"].append(mi["status"])
        rec["fallback"].append(mi["fallback"])
        rec["cold_start"].append(mi["cold_start"])
        rec["residuals"].append(mi.get("residuals", np.full((B, 4), np.nan)))
        rec["wall_time"].append(mi["wall_time"])
        for i in np.flatnonzero(sd.episode_end):
            end_step[i], end_reason[i] = sd.k, sd.end_reason[i]
    arr = {k: np.asarray(v) for k, v in rec.items()}
    return EpisodeLog(
        **{k: arr[k] for k in rec},
        s0=s0_used,
        end_step=end_step,
        end_reason=end_reason,
        dt=plant.dt,
        lap_length=plant.reference.length,
        meta={
            "theta0": tuple(controller.cfg.theta0),
            "laps": laps,
            "perturbations": [str(p) for p in plant.perturbations],
        },
    )


# ------------------------------------------------------------------------- metrics


def observed_signals(states, dt):
    """What the car actually did, derived from its logged states (one car, consecutive steps, (T, NX)).

    The commanded inputs (jerk, steering rate) are what the controller asked for; under model mismatch the
    car's motion differs from the MPC's prediction, so the quantities a passenger feels are taken from the
    states instead:
        speed      |(vx, vy)|                                   (T,)
        ax         longitudinal acceleration  Δvx / dt          (T-1,)  (= a + vy·r, not the a state)
        jerk       Δax / dt                                     (T-2,)
        steer_rate Δδ / dt                                      (T-1,)
        alat       lateral acceleration  vx·r                   (T,)
    Finite differences over one control step (dt = 0.02 s); the first samples of an episode are lost
    (the state before its first step is not logged).
    """
    st = np.asarray(states, dtype=float)
    ax = np.diff(st[:, VX]) / dt
    return {
        "speed": np.hypot(st[:, VX], st[:, VY]),
        "ax": ax,
        "jerk": np.diff(ax) / dt,
        "steer_rate": np.diff(st[:, DELTA]) / dt,
        "alat": st[:, VX] * st[:, R],
    }


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else np.nan


def _absmax(x):
    return float(np.max(np.abs(x))) if len(x) else np.nan


def _share_above(x, th):
    return float(np.mean(np.abs(x) > th)) if len(x) else np.nan


def _stats(x):
    a = np.abs(x)
    if a.size == 0:
        return dict(mean=np.nan, p25=np.nan, p75=np.nan, max=np.nan, rms=np.nan)
    return dict(
        mean=float(a.mean()),
        p25=float(np.percentile(a, 25)),
        p75=float(np.percentile(a, 75)),
        max=float(a.max()),
        rms=float(np.sqrt(np.mean(np.asarray(x) ** 2))),
    )


def compute_metrics(log, plant, jerk_th=8.0, steer_rate_th=0.25, margin_warn=0.05):
    """Per-car metrics of an :class:`EpisodeLog` and a summary over the cars.

    Returns ``(per_car: list of dicts, summary: dict)``. Per car: outcome (end_reason, completed,
    lap_time), tracking (|n| and |v - v_ref|: mean, p25, p75, max, rms; from the plant's state), safety
    (min margin and where, time below ``margin_warn``, time and depth off track, max friction-ellipse
    use), smoothness of jerk and steering rate (rms, max, share of time above the paper's loss
    thresholds) twice: ``*_cmd`` from the commanded inputs (what the MPC asked for) and ``*_obs`` from the
    observed states (:func:`observed_signals`: what the car did), plus observed accelerations and speed;
    loss (mean task loss at node 1 of the MPC's prediction), solver (failures, fallbacks, cold starts,
    residuals).
    """
    lim = plant.limits
    per_car = []
    for i in range(log.active.shape[1]):
        m = log.active[:, i]
        steps = int(m.sum())
        st = log.state[m, i]
        u = log.u[m, i]
        obs = observed_signals(st, log.dt)
        n, dv, margin = log.n[m, i], log.dv[m, i], log.margin[m, i]
        reason = log.end_reason[i]
        ellipse = (st[:, ACC] / lim.max_decel) ** 2 + (st[:, VX] * st[:, R] / lim.ay_max) ** 2
        k_min = int(np.argmin(margin)) if steps else 0
        res = log.residuals[m, i]
        row = {
            "car": i,
            "s0": float(log.s0[i]),
            "perturbation": plant.perturbations[i],
            "end_reason": reason,
            "completed": reason == "lap_done",
            "steps": steps,
            "lap_time": steps * log.dt if reason == "lap_done" else np.nan,
            "progress": float(log.progress[m, i][-1]) if steps else 0.0,
            **{f"n_{k}": v for k, v in _stats(n).items()},
            **{f"dv_{k}": v for k, v in _stats(dv).items()},
            "margin_min": float(margin.min()) if steps else np.nan,
            "margin_min_s": float(log.s[m, i][k_min]) if steps else np.nan,
            "time_margin_below": float((margin < margin_warn).sum() * log.dt),
            "time_off_track": float((margin < 0).sum() * log.dt),
            "violation_max": float(max(0.0, -margin.min())) if steps else np.nan,
            "ellipse_max": float(ellipse.max()) if steps else np.nan,
            # commanded (the MPC's input)
            "jerk_cmd_rms": _rms(u[:, 0]),
            "jerk_cmd_max": _absmax(u[:, 0]),
            "jerk_cmd_above_th": _share_above(u[:, 0], jerk_th),
            "steer_rate_cmd_rms": _rms(u[:, 1]),
            "steer_rate_cmd_max": _absmax(u[:, 1]),
            "steer_rate_cmd_above_th": _share_above(u[:, 1], steer_rate_th),
            # observed (derived from the plant's states)
            "jerk_obs_rms": _rms(obs["jerk"]),
            "jerk_obs_max": _absmax(obs["jerk"]),
            "jerk_obs_above_th": _share_above(obs["jerk"], jerk_th),
            "steer_rate_obs_rms": _rms(obs["steer_rate"]),
            "steer_rate_obs_max": _absmax(obs["steer_rate"]),
            "steer_rate_obs_above_th": _share_above(obs["steer_rate"], steer_rate_th),
            "ax_obs_max": float(obs["ax"].max()) if len(obs["ax"]) else np.nan,
            "ax_obs_min": float(obs["ax"].min()) if len(obs["ax"]) else np.nan,
            "alat_obs_max": _absmax(obs["alat"]),
            "speed_max": float(obs["speed"].max()) if steps else np.nan,
            "loss_mean": float(log.loss[m, i].mean()) if steps else np.nan,
            "failures": int((log.status[m, i] != 0).sum()),
            "fallbacks": int(log.fallback[m, i].sum()),
            "cold_starts": int(log.cold_start[m, i].sum()),
            "residual_comp_median": float(np.nanmedian(res[:, 3]))
            if steps and np.isfinite(res).any()
            else np.nan,
            "residual_comp_max": float(np.nanmax(res[:, 3]))
            if steps and np.isfinite(res).any()
            else np.nan,
        }
        per_car.append(row)
    wt = log.wall_time
    summary = {
        "cars": len(per_car),
        "completed": int(sum(r["completed"] for r in per_car)),
        "end_reasons": {
            r: sum(c["end_reason"] == r for c in per_car)
            for r in END_REASONS
            if any(c["end_reason"] == r for c in per_car)
        },
        "lap_time_median": float(np.nanmedian([r["lap_time"] for r in per_car]))
        if any(r["completed"] for r in per_car)
        else np.nan,
        "reference_lap_time": float(plant.reference.raceline.lap_time),
        "n_rms_median": float(np.median([r["n_rms"] for r in per_car])),
        "n_max": float(np.max([r["n_max"] for r in per_car])),
        "dv_rms_median": float(np.median([r["dv_rms"] for r in per_car])),
        "margin_min": float(np.nanmin([r["margin_min"] for r in per_car])),
        "cars_off_track": int(sum(r["time_off_track"] > 0 for r in per_car)),
        "violation_max": float(np.nanmax([r["violation_max"] for r in per_car])),
        "jerk_cmd_above_th_median": float(np.median([r["jerk_cmd_above_th"] for r in per_car])),
        "jerk_obs_above_th_median": float(np.median([r["jerk_obs_above_th"] for r in per_car])),
        "steer_rate_cmd_above_th_median": float(np.median([r["steer_rate_cmd_above_th"] for r in per_car])),
        "steer_rate_obs_above_th_median": float(np.median([r["steer_rate_obs_above_th"] for r in per_car])),
        "loss_mean": float(np.nanmean([r["loss_mean"] for r in per_car])),
        "failures": int(sum(r["failures"] for r in per_car)),
        "fallbacks": int(sum(r["fallbacks"] for r in per_car)),
        "wall_time_mean_ms": float(1e3 * wt.mean()),
        "wall_time_p95_ms": float(1e3 * np.percentile(wt, 95)),
        "wall_time_max_ms": float(1e3 * wt.max()),
        "wall_time_per_car_ms": float(1e3 * wt.mean() / max(len(per_car), 1)),
    }
    return per_car, summary
