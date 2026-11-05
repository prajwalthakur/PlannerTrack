# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Closed loop (steps.md 4.4 / 4.6): rollout, episode handling, metrics, the node-1 loss, the batched
policy observation, and the controller's bookkeeping. One compiled solver per module (~30 s)."""

from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("leap_c.torch")

from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: E402
from plannertrack_py.batch_plant import BatchPlant  # noqa: E402
from diff_weight_mpc.closed_loop import (  # noqa: E402
    EpisodeLog,
    compute_metrics,
    evenly_spaced_starts,
    rollout,
    run_episodes,
)
from diff_weight_mpc.learning.loss import LossConfig, node_loss, threshold_penalty  # noqa: E402
from diff_weight_mpc.mpc.controller import MpcController  # noqa: E402
from diff_weight_mpc.mpc.ocp import THETA0, OcpConfig  # noqa: E402
from plannertrack_py.plant import NU, NX, Perturbation  # noqa: E402

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
B = 2


@pytest.fixture(scope="module")
def ref():
    return ReferenceTrack(compute_raceline(PARAMS / "agents.yaml", "agent_1", PARAMS / "min_curv_planner.yaml"))


@pytest.fixture(scope="module")
def plant(ref):
    return BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref)  # dt 0.02


@pytest.fixture(scope="module")
def mpc(plant, tmp_path_factory):
    return MpcController.create_controller(plant, export_directory=tmp_path_factory.mktemp("acados_cl"),
                                           name="test_closed_loop", log_residuals=True)


# ------------------------------------------------------------------ helpers without the solver


def test_policy_observation_matches_single(ref):
    s0 = np.array([0.0, 5.0, 20.0, ref.length - 0.5])
    v, k = ref.policy_observation(s0)
    assert v.shape == (4, 5) and k.shape == (4, 5)
    for i, s in enumerate(s0):
        vi, ki = ref.policy_lookahead(s)
        np.testing.assert_allclose(v[i], vi, atol=1e-9)
        np.testing.assert_allclose(k[i], ki, atol=1e-9)


def test_evenly_spaced_starts(ref):
    s = evenly_spaced_starts(ref, 4)
    np.testing.assert_allclose(np.diff(s), ref.length / 4)
    assert s[0] == 0.0


def test_threshold_penalty_is_continuous():
    th, kappa = 8.0, 0.15
    x = torch.tensor([th - 1e-6, th, th + 1e-6, -th - 1e-6, 3.0, 20.0], dtype=torch.float64)
    p = threshold_penalty(x, th, kappa)
    assert p[0] == pytest.approx(th**2, rel=1e-6) and p[2] == pytest.approx(th**2, rel=1e-6)
    assert p[3] == pytest.approx(th**2, rel=1e-6)
    assert p[4] == pytest.approx(9.0)
    assert p[5] == pytest.approx(th**2 + np.expm1(kappa * 12.0))


def test_node_loss_by_hand():
    """One car, node 1: lateral offset 0.1 m left of a reference pointing along +x, speed 0.5 above."""
    N = 3
    x = np.zeros((1, N + 1, NX))
    u = np.zeros((1, N, NU))
    x[0, 1, 1] = 0.1        # Y
    x[0, 1, 3] = 5.5        # vx
    u[0, 1] = [10.0, 0.1]   # jerk above its threshold, steering rate below
    params = {"x_ref": np.zeros((1, N + 1, 1)), "y_ref": np.zeros((1, N + 1, 1)),
              "psi_ref": np.zeros((1, N + 1, 1)), "v_ref": np.full((1, N + 1, 1), 5.0)}
    cfg = LossConfig()
    loss, parts = node_loss(x, u, params, cfg)
    assert float(parts["n"][0]) == pytest.approx(0.1) and float(parts["dv"][0]) == pytest.approx(0.5)
    expected = (cfg.alpha * 0.01 + cfg.beta * 0.25 + cfg.gamma * (8.0**2 + np.expm1(0.15 * 2.0))
                + cfg.delta * 0.01)
    assert float(loss[0]) == pytest.approx(expected)


def test_node_loss_keeps_the_graph():
    x = torch.zeros((1, 3, NX), dtype=torch.float64, requires_grad=True)
    u = torch.zeros((1, 2, NU), dtype=torch.float64)
    params = {k: np.zeros((1, 3, 1)) for k in ("x_ref", "y_ref", "psi_ref", "v_ref")}
    loss, _ = node_loss(x + torch.tensor(1.0), u, params)
    loss.sum().backward()
    assert x.grad is not None and torch.any(x.grad != 0)


def test_observed_signals_by_hand():
    """Finite differences of the states: vx = 5 + 2t + t^3 sampled at dt -> ax ~ 2 + 3t^2, jerk ~ 6t."""
    dt = 0.02
    t_ = dt * np.arange(50)
    st = np.zeros((50, NX))
    st[:, 3] = 5 + 2 * t_ + t_**3
    st[:, 5] = 0.4
    st[:, 6] = 0.1 * t_  # steering at 0.1 rad/s
    from diff_weight_mpc.closed_loop import observed_signals
    o = observed_signals(st, dt)
    assert o["ax"].shape == (49,) and o["jerk"].shape == (48,)
    np.testing.assert_allclose(o["steer_rate"], 0.1, atol=1e-12)
    np.testing.assert_allclose(o["jerk"], 6 * (t_[1:-1] + dt / 2) + 0 * t_[1:-1], atol=0.13)
    np.testing.assert_allclose(o["alat"], st[:, 3] * 0.4)


def test_metrics_on_a_synthetic_log(plant):
    """Known per-step values give the expected metrics (no solver involved)."""
    T, dt = 10, 0.02
    active = np.ones((T, B), dtype=bool)
    active[6:, 1] = False  # car 1 stopped after 6 steps
    state = np.zeros((T, B, NX))
    state[..., 3] = 5.0
    u = np.zeros((T, B, NU))
    u[:, :, 0] = 10.0  # jerk above 8 everywhere
    n = np.full((T, B), 0.1)
    margin = np.full((T, B), 0.2)
    margin[3, 0] = -0.01  # car 0 off track for one step
    log = EpisodeLog(t=dt * np.arange(1, T + 1), active=active, state=state, u=u, theta=np.zeros((T, B, 6)),
                     s=np.tile(np.linspace(0, 1, T)[:, None], (1, B)), n=n, dpsi=np.zeros((T, B)),
                     dchi=np.zeros((T, B)), dv=np.full((T, B), -0.2), v_ref=np.full((T, B), 5.2),
                     progress=np.tile(np.linspace(0.1, 1, T)[:, None], (1, B)), margin=margin,
                     loss=np.full((T, B), 0.5), status=np.zeros((T, B), dtype=int),
                     fallback=np.zeros((T, B), dtype=bool), cold_start=np.zeros((T, B), dtype=bool),
                     residuals=np.full((T, B, 4), np.nan), wall_time=np.full(T, 0.004), s0=np.zeros(B),
                     end_step=np.array([9, 5]), end_reason=["lap_done", "off_track"], dt=dt, lap_length=1.0)
    per, summary = compute_metrics(log, plant)
    assert per[0]["completed"] and not per[1]["completed"]
    assert per[0]["steps"] == 10 and per[1]["steps"] == 6
    assert per[0]["lap_time"] == pytest.approx(0.2) and np.isnan(per[1]["lap_time"])
    assert per[0]["n_rms"] == pytest.approx(0.1) and per[0]["dv_rms"] == pytest.approx(0.2)
    assert per[0]["margin_min"] == pytest.approx(-0.01) and per[0]["violation_max"] == pytest.approx(0.01)
    assert per[0]["time_off_track"] == pytest.approx(dt) and per[1]["time_off_track"] == 0.0
    assert per[0]["jerk_cmd_above_th"] == 1.0 and per[0]["steer_rate_cmd_above_th"] == 0.0
    # observed from the states: constant speed and steering -> no acceleration, jerk or steering rate,
    # although the commanded jerk was 10 (the commanded and observed signals are independent)
    assert per[0]["jerk_obs_max"] == 0.0 and per[0]["ax_obs_max"] == 0.0 and per[0]["steer_rate_obs_max"] == 0.0
    assert summary["completed"] == 1 and summary["cars_off_track"] == 1
    assert summary["wall_time_mean_ms"] == pytest.approx(4.0)


# ------------------------------------------------------------------ closed loop with the solver


def test_nominal_lap_from_two_starts(plant, mpc):
    """θ0 drives one lap from two starts (one in the braking zone before the hairpin): no failure flag,
    no solver failure, inside the track everywhere."""
    log = run_episodes(plant, mpc, s0=[0.0, 20.0])
    per, summary = compute_metrics(log, plant)
    assert [r["end_reason"] for r in per] == ["lap_done", "lap_done"]
    assert summary["failures"] == 0 and summary["fallbacks"] == 0
    assert summary["margin_min"] > 0.0
    for r in per:
        assert r["n_rms"] < 0.15 and r["lap_time"] < 1.1 * plant.reference.raceline.lap_time
    assert np.all(log.theta[log.active] == np.array(THETA0))


def test_stop_mode_masks_finished_cars(plant, mpc):
    """A car whose episode ended is no longer a data point; the rollout ends when no car is active."""
    steps = list(rollout(plant, mpc, s0=[0.0, 13.0], max_episode_steps=40, on_done="stop"))
    assert len(steps) == 40  # both cars hit the time limit at step 39
    assert steps[-1].episode_end.all() and steps[-1].end_reason == ["time_limit", "time_limit"]
    assert all(sd.active.all() for sd in steps)
    steps = list(rollout(plant, mpc, s0=[0.0, 13.0], laps=1, max_steps=None,
                         max_episode_steps=None, on_done="stop"))
    ends = [np.flatnonzero(sd.episode_end) for sd in steps]
    first_end = next(k for k, e in enumerate(ends) if e.size)
    finished = ends[first_end][0]
    assert all(not sd.active[finished] for sd in steps[first_end + 1:])  # masked afterwards


def test_reset_mode_keeps_producing_data(plant, mpc):
    """on_done='reset' (training): after an episode ends the car restarts and stays a data point."""
    steps = list(rollout(plant, mpc, s0=[0.0, 13.0], max_episode_steps=15, max_steps=40, on_done="reset",
                         observe=True))
    assert len(steps) == 40
    assert all(sd.active.all() for sd in steps)
    assert sum(sd.episode_end.sum() for sd in steps) == 4  # 2 cars x 2 episode ends (steps 14, 29)
    assert steps[15].episode_step.tolist() == [0, 0]
    np.testing.assert_allclose(steps[15].episode_s0, [0.0, 13.0])  # restarts at the first s0
    assert steps[0].obs.shape == (B, 10)


def test_theta_fn_and_gradient_through_a_step(plant, mpc):
    """A policy-like theta_fn: theta with requires_grad reaches the loss's backward (Algorithm 1)."""
    w = torch.tensor(np.log(np.array(THETA0)), requires_grad=True)

    def theta_fn(obs, k):
        return torch.exp(w).expand(len(obs), -1)

    sd = next(rollout(plant, mpc, theta_fn=theta_fn, s0=[0.0, 13.0], max_steps=1))
    loss, _ = node_loss(sd.solution.x, sd.solution.u, sd.solution.params)
    loss.sum().backward()
    assert w.grad is not None and torch.all(torch.isfinite(w.grad)) and torch.any(w.grad != 0)


def test_same_seed_same_rollout(plant, mpc):
    a = [sd.u.copy() for sd in rollout(plant, mpc, s0=[0.0, 13.0], max_steps=25)]
    b = [sd.u.copy() for sd in rollout(plant, mpc, s0=[0.0, 13.0], max_steps=25)]
    np.testing.assert_array_equal(np.stack(a), np.stack(b))


def test_oracle_rows_and_perturbed_plants(ref, mpc, tmp_path):
    """Per-row vehicle parameters reach the MPC: two μ x 0.9 cars from the same start, one MPC row with the
    nominal model and one with the true parameters (oracle), drive differently. With the 3 cm MPC track
    margin (4.9) both stay on the track at the tight apex (s ~ 18.6 m), where without it the nominal-model
    car left the track by a few millimetres (4.7)."""
    pert = Perturbation(mu_scale=0.9)
    plant = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, perturbations=[pert, pert])
    vehicle = [plant.plants[0].nominal_params, plant.plants[1].true_params]
    log = run_episodes(plant, mpc, s0=[0.0, 0.0], vehicle_params=vehicle,
                       end_on=("too_slow", "heading_out", "diverged"))
    per, _ = compute_metrics(log, plant)
    k = min(per[0]["steps"], per[1]["steps"])
    assert np.abs(log.state[:k, 0] - log.state[:k, 1]).max() > 1e-3  # different models, different driving
    assert all(r["end_reason"] == "lap_done" for r in per)
    assert all(r["time_off_track"] == 0.0 for r in per)


def test_controller_reset_rows_and_fallback(plant, mpc):
    """reset(rows) cold-starts only those rows; a failed row gets its last good plan's input."""
    x, info = plant.reset(s0=[0.0, 13.0])
    mpc.reset()
    for _ in range(3):
        u, mi = mpc.step(x, info["s0"])
    assert not mi["cold_start"].any()
    mpc.reset([1])
    u, mi = mpc.step(x, info["s0"])
    assert mi["cold_start"].tolist() == [False, True]
    # fallback: the last plan's input at the current time (plan step = elapsed // dt)
    mpc._plan_u[0] = np.arange(OcpConfig().N * NU, dtype=float).reshape(-1, NU)
    mpc._plan_t[0] = mpc._t - 2.5 * OcpConfig().dt  # plan computed 2.5 MPC intervals ago
    np.testing.assert_array_equal(mpc._fallback_inputs()[0], mpc._plan_u[0, 2])
