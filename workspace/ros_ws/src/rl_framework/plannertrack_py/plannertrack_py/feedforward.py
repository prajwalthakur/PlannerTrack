# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Open-loop feed-forward along the reference (steps.md 3.6; later the MPC's initial guess).

The reference gives position, heading, speed v_ref(s), curvature and a_x,ref(s), but not the
steering angle or the acceleration state the car needs (the planner is a point mass). This module
times the reference (ds/dt = v_ref, as :meth:`ReferenceTrack.stage_references`), computes the
steady cornering state at every stage (:func:`plannertrack_py.plant.steady_cornering`) and turns
it into inputs: constant jerk and steering rate per interval, so a(t) and delta(t) are the
piecewise-linear interpolation of the stage values. With ``max_jerk`` / ``max_steer_rate`` the
schedule is rate-limited (it lags behind and catches up): the reference's a_x is not jerk-limited
(the speed profile switches between full drive and full braking from one stage to the next, up to
~150 m/s^3), and inputs the plant clips would leave a permanent offset in a and delta, because
open-loop inputs are differences of the schedule.

Driving these inputs on the plant with no feedback shows how far the car drifts from the line on
its own, i.e. how much correction the MPC has to add.
"""

from dataclasses import dataclass

import numpy as np

from plannertrack_py.plant import FAILURE_FLAGS, NU, steady_cornering


@dataclass(frozen=True)
class FeedforwardSchedule:
    """Stage values k = 0..N at t_k = k*dt, and the N inputs between them."""

    t: np.ndarray  # (N+1,) stage times [s]
    s: np.ndarray  # (N+1,) arc length, continuous (not wrapped) [m]
    v: np.ndarray  # (N+1,) v_ref [m/s]
    delta: np.ndarray  # (N+1,) steering angle the inputs produce (rate-limited target) [rad]
    acc: np.ndarray  # (N+1,) acceleration state the inputs produce (rate-limited target) [m/s^2]
    delta_target: np.ndarray  # (N+1,) steady steering angle [rad]
    acc_target: np.ndarray  # (N+1,) acceleration state of the steady state, a_x,ref - vy*r [m/s^2]
    converged: np.ndarray  # (N+1,) bool: steady state found (else kinematic guess)
    u: np.ndarray  # (N, NU) [jerk, steer_rate] held over [t_k, t_k+1)
    dt: float


def feedforward_schedule(reference, xdot, wheelbase, max_steer, s0, n_steps, dt,
                         max_jerk=None, max_steer_rate=None):
    """Feed-forward over ``n_steps`` intervals of ``dt`` from arc length ``s0``.

    Args:
        reference: :class:`ReferenceTrack`.
        xdot: derivative with the plant's layout, e.g. a nominal plant's ``Sim.xdot`` or the
            MPC's CasADi model (the controller's knowledge, not the true plant's).
        wheelbase, max_steer: for :func:`steady_cornering`.
        max_jerk, max_steer_rate: input limits; if given, the schedule follows the targets as fast
            as these allow, so every input is inside the limits. None: no limit.
    """
    stages = reference.stage_references(s0, n_steps, dt)
    k = n_steps + 1
    delta, acc, converged = np.empty(k), np.empty(k), np.empty(k, dtype=bool)
    for i in range(k):
        d, _, a, ok = steady_cornering(xdot, float(stages["vx"][i]), float(stages["kappa"][i]),
                                       float(stages["ax"][i]), wheelbase, max_steer)
        delta[i], acc[i], converged[i] = d, a, ok
    acc_ff, delta_ff = _rate_limited(acc, max_jerk, dt), _rate_limited(delta, max_steer_rate, dt)
    u = np.zeros((n_steps, NU))
    u[:, 0] = np.diff(acc_ff) / dt  # jerk
    u[:, 1] = np.diff(delta_ff) / dt  # steering rate
    return FeedforwardSchedule(t=stages["t"], s=stages["s"], v=stages["vx"], delta=delta_ff,
                               acc=acc_ff, delta_target=delta, acc_target=acc,
                               converged=converged, u=u, dt=dt)


def _rate_limited(target, max_rate, dt):
    """Follow ``target`` from target[0] with |change per step| <= max_rate * dt."""
    if max_rate is None:
        return target.copy()
    out = np.empty_like(target)
    out[0] = target[0]
    step = max_rate * dt
    for i in range(1, len(target)):
        out[i] = out[i - 1] + np.clip(target[i] - out[i - 1], -step, step)
    return out


def run_open_loop(plant, schedule, stop_on_termination=True):
    """Drive ``schedule.u`` on ``plant`` from its current state (call ``plant.reset`` first).

    Returns a dict of arrays, one row per step: states (N, nx) and the info fields t, s, n, dpsi,
    dchi, beta, dv, v_ref, progress, plus ``reason``: "lap_done", "schedule_end", or the
    FAILURE_FLAGS that ended the run joined by "+" (e.g. "too_slow+heading_out" for a spin).
    """
    keys = ("t", "s", "n", "dpsi", "dchi", "beta", "dv", "v_ref", "progress")
    rows, states, reason = {k: [] for k in keys}, [], "schedule_end"
    for u in schedule.u:
        x, info = plant.step(u)
        states.append(x)
        for k in keys:
            rows[k].append(info[k])
        if info["lap_done"]:
            reason = "lap_done"
            break
        if stop_on_termination and info["terminated"]:
            reason = "+".join(f for f in FAILURE_FLAGS if info[f])
            break
    out = {k: np.asarray(v) for k, v in rows.items()}
    out["states"] = np.asarray(states)
    out["reason"] = reason
    return out
