# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Tests for the Diff-MPC training script (experiments/diff_weight_mpc/utils/train_mpc.py): static weights (Step 6, steps.md §9)."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

SCRIPT = Path(__file__).resolve().parents[2] / "experiments" / "diff_weight_mpc" / "utils" / "train_mpc.py"


@pytest.fixture(scope="module")
def ts():
    spec = importlib.util.spec_from_file_location("train_mpc", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---- 6.1 StaticWeights / make_optimizer (fast: no solver)

@pytest.mark.parametrize("param", ["exp", "softplus"])
def test_starts_at_theta0(ts, param):
    m = ts.StaticWeights(ts.THETA0, param)
    np.testing.assert_allclose(m.theta, ts.THETA0, rtol=1e-12)


@pytest.mark.parametrize("param", ["exp", "softplus"])
def test_shape_and_gradient(ts, param):
    m = ts.StaticWeights(ts.THETA0, param)
    th = m(np.zeros((4, 10)), 0)
    assert th.shape == (4, 6) and th.dtype == torch.float64
    th[0].sum().backward()
    theta = np.array(ts.THETA0)
    expected = theta if param == "exp" else -np.expm1(-theta)  # dθ/dw: exp → θ, softplus → sigmoid(w) = 1 − e^(−θ)
    np.testing.assert_allclose(m.w.grad.numpy(), expected, rtol=1e-12)


@pytest.mark.parametrize("param", ["exp", "softplus"])
def test_one_adam_step(ts, param):
    m = ts.StaticWeights(ts.THETA0, param)
    opt = ts.make_optimizer(m, ts.TrainConfig(lr=0.03))
    (-m(None, 0).sum()).backward()  # pushes every θ up
    opt.step()
    ratio = m.theta / np.array(ts.THETA0)
    if param == "exp":
        np.testing.assert_allclose(ratio, np.exp(0.03), rtol=1e-6)  # +3 % for every weight
    else:
        assert ratio[0] < 1.001 and ratio[4] > 1.02  # q_n = 50 barely moves; r_j = 0.01 moves ~3 %


# ---- 6.4 update rule (fast)

def _step(k=0, end=False, active=True, ok=True):
    """A minimal StepData stand-in for one car."""
    from types import SimpleNamespace
    return SimpleNamespace(k=k, episode_end=np.array([end]), active=np.array([active]),
                           mpc_info={"ok": np.array([ok])}, end_reason=["lap_done" if end else None],
                           episode_step=np.array([299]), theta=torch.tensor([[1.0] * 6], dtype=torch.float64))


def test_is_update_step(ts):
    episode = ts.TrainConfig(accumulate_steps="episode")
    assert not ts.is_update_step(_step(end=False), 5, episode)
    assert ts.is_update_step(_step(end=True), 5, episode)
    every10 = ts.TrainConfig(accumulate_steps=10)
    assert [k for k in range(30) if ts.is_update_step(_step(k), k, every10)] == [9, 19, 29]


def test_optimizer_update_averages_clips_and_zeroes(ts):
    m = ts.StaticWeights(ts.THETA0)
    cfg = ts.TrainConfig(lr=0.01, grad_clip=0.1)
    opt = ts.make_optimizer(m, cfg)
    w0 = m.w.detach().clone()
    m.w.grad = torch.tensor([3.0, -3.0, 0.03, -0.03, 0.0, 30.0], dtype=torch.float64)  # summed over 10 points
    raw = ts.optimizer_update(m, opt, cfg, n_points=10)
    np.testing.assert_allclose(raw, [0.3, -0.3, 0.003, -0.003, 0.0, 3.0])  # mean, before clipping
    step = (m.w.detach() - w0).numpy()
    np.testing.assert_allclose(step, -0.01 * np.sign(raw), atol=1e-7)  # Adam's first step: lr·g/(|g| + 1e-8) ≈ lr·sign
    assert m.w.grad is None or not m.w.grad.any()  # zeroed for the next window


def test_optimizer_update_skips_nan(ts):
    m = ts.StaticWeights(ts.THETA0)
    cfg = ts.TrainConfig()
    opt = ts.make_optimizer(m, cfg)
    w0 = m.w.detach().clone()
    m.w.grad = torch.tensor([np.nan, 1, 1, 1, 1, 1], dtype=torch.float64)
    ts.optimizer_update(m, opt, cfg, n_points=1)
    assert torch.equal(m.w, w0)
    assert ts.optimizer_update(m, opt, cfg, n_points=1) is None  # no gradient left: nothing to do


# ---- 6.5 fallback (fast)

@pytest.mark.parametrize("param", ["exp", "softplus"])
def test_feasible_weights_restore_resets_adam(ts, param):
    m = ts.StaticWeights(ts.THETA0, param)
    opt = ts.make_optimizer(m, ts.TrainConfig(lr=0.03))
    feasible = ts.FeasibleWeights(m)
    w0 = m.w.detach().clone()
    (-m(None, 0).sum()).backward()
    opt.step()
    opt.zero_grad()
    assert not torch.equal(m.w, w0) and len(opt.state[m.w]) > 0
    feasible.restore(m, opt)  # nothing recorded since construction: back to θ0
    assert torch.equal(m.w, w0) and len(opt.state.get(m.w, {})) == 0
    np.testing.assert_allclose(m.theta, ts.THETA0, rtol=1e-12)


def test_feasible_weights_keeps_last_clean(ts):
    m = ts.StaticWeights(ts.THETA0)
    feasible = ts.FeasibleWeights(m)
    with torch.no_grad():
        m.w += 0.1
    feasible.record(m)  # a clean window driven by w0 + 0.1
    w_clean = m.w.detach().clone()
    with torch.no_grad():
        m.w += 0.5  # the update whose window then fails
    feasible.restore(m)
    assert torch.equal(m.w, w_clean) and feasible.n_restores == 1


# ---- 6.6 logging and outputs (fast)

def test_training_log_rows_best_and_theta_file(ts, tmp_path):
    m = ts.StaticWeights(ts.THETA0)
    log = ts.TrainingLog(dt=0.02)
    parts = {"loss": 2.0, "n": 0.01, "L_n": 1.0}
    log.add_step(_step(0), None, 1, parts)
    log.add_step(_step(1, end=True), None, 1, {**parts, "loss": 4.0})
    row = log.end_update(1, m, np.array([0.2, 0, 0, 0, 0, 0]), False, grad_clip=0.1)
    assert row["steps"] == 2 and row["points"] == 2 and row["loss"] == 3.0
    assert row["clipped"] == 1 and row["episodes"] == 1 and row["lap_time"] == 6.0
    log.add_step(_step(0, ok=False, end=True), None, 0, {})
    row2 = log.end_update(2, m, None, True)
    assert row2["solver_failures"] == 1 and row2["fallback"] and np.isnan(row2["grad_q_n"])
    assert ts.best_row(log.rows)["update"] == 1  # the failed window never qualifies
    log.write(tmp_path)
    assert (tmp_path / "history.csv").read_text().count("\n") == 3
    ts.save_theta(tmp_path / "theta.json", m, log, ts.TrainConfig())
    import json
    data = json.loads((tmp_path / "theta.json").read_text())
    np.testing.assert_allclose(data["theta"], ts.THETA0, rtol=1e-12)
    assert data["best_update"] == 1 and data["weight_param"] == "exp"


# ---- 6.3 step loss gradient through the real loss (slow: compiles a converged SQP solver, ~40 s)

@pytest.fixture(scope="module")
def sqp_setup(ts, tmp_path_factory):
    from types import SimpleNamespace
    pytest.importorskip("leap_c.torch")
    args = SimpleNamespace(scenario="ground_vehicle_racing_rl", agent="agent_1", out=tmp_path_factory.mktemp("ts"))
    cfg = ts.TrainConfig(nlp_solver="SQP", max_iter=200)
    ref, plant, mpc = ts.build_setup(args, cfg)
    return ref, plant, mpc, cfg


def test_step_loss_gradient_matches_finite_difference(ts, sqp_setup):
    """∂L/∂w from backward() through leap-c and the exp map vs a central difference in w (same state, cold solves)."""
    from types import SimpleNamespace
    ref, plant, mpc, cfg = sqp_setup
    x0, info = plant.reset(seed=0, s0=[5.0])
    s0 = np.asarray(info["s0"], dtype=float)
    obs = np.concatenate(ref.policy_observation(s0), axis=1)

    def loss_at(model, backward=False):
        mpc.reset()
        _, mpc_info = mpc.step(x0, s0, theta=model(obs, 0))
        sd = SimpleNamespace(active=np.array([True]), mpc_info=mpc_info, solution=mpc.last)
        loss_sum, n_valid, _ = ts.step_loss(sd, cfg.loss_cfg)
        assert n_valid == 1
        if backward:
            loss_sum.backward()
        return float(loss_sum.detach())

    m = ts.StaticWeights(ts.THETA0)
    loss_at(m, backward=True)
    grad = m.w.grad.detach().numpy().copy()
    fd = np.empty(6)
    h = 1e-4
    for i in range(6):
        values = []
        for sign in (1, -1):
            p = ts.StaticWeights(ts.THETA0)
            with torch.no_grad():
                p.w[i] += sign * h
            values.append(loss_at(p))
        fd[i] = (values[0] - values[1]) / (2 * h)
    np.testing.assert_allclose(grad, fd, rtol=1e-3, atol=1e-6 * np.abs(fd).max())


def test_theta_file_is_readable_by_baseline_suite(ts, tmp_path):
    """train_mpc's theta.json → baseline_suite.load_theta_file (the evaluation path of 6.9)."""
    spec = importlib.util.spec_from_file_location("baseline_suite", SCRIPT.parent / "baseline_suite.py")
    suite = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(suite)
    m = ts.StaticWeights(ts.THETA0)
    log = ts.TrainingLog()
    log.add_step(_step(0, end=True), None, 1, {"loss": 1.0})
    log.end_update(1, m, np.zeros(6), False)
    ts.save_theta(tmp_path / "theta.json", m, log, ts.TrainConfig())
    np.testing.assert_allclose(suite.load_theta_file(tmp_path / "theta.json"), ts.THETA0, rtol=1e-12)
    # best_theta = the θ that drove the best window: the stand-in steps carry θ = 1
    np.testing.assert_allclose(suite.load_theta_file(tmp_path / "theta.json", "best_theta"), np.ones(6))


# ---- Step 7: the policy branch (7.2)

def _policy_model(ts):
    from diff_weight_mpc.learning.policy_network import WeightPolicy
    scale = np.r_[np.full(5, 8.0), np.full(5, 1.0)]
    probe = np.c_[np.full((3, 5), 5.0), np.zeros((3, 5))]
    return ts.PolicyWeights(WeightPolicy(ts.THETA0, scale), probe)


def test_policy_optimizer_groups_bias_like_static_w(ts):
    m = _policy_model(ts)
    opt = ts.make_optimizer(m, ts.TrainConfig(model="policy", lr=0.1, lr_net=1e-3))
    bias_group, rest_group = opt.param_groups
    assert bias_group["lr"] == 0.1 and bias_group["params"] == [m.policy.head.bias]
    assert rest_group["lr"] == 1e-3 and len(rest_group["params"]) == len(list(m.parameters())) - 1
    np.testing.assert_allclose(m.theta, ts.THETA0, rtol=1e-12)  # lap-mean θ of the fresh policy


def test_feasible_weights_restore_a_policy(ts):
    m = _policy_model(ts)
    opt = ts.make_optimizer(m, ts.TrainConfig(model="policy", lr=0.1, lr_net=1e-3))
    feasible = ts.FeasibleWeights(m)
    before = {k: v.clone() for k, v in m.state_dict().items()}
    for _ in range(2):  # two steps, so every layer moves
        (-m(m.probe_obs, 0).sum()).backward()
        opt.step()
        opt.zero_grad()
    assert not torch.equal(m.policy.body[0].weight, before["policy.body.0.weight"])
    feasible.restore(m, opt)
    assert all(torch.equal(m.state_dict()[k], v) for k, v in before.items())
    assert all(len(opt.state.get(q, {})) == 0 for q in m.parameters())


def test_training_log_theta_statistics(ts):
    log = ts.TrainingLog()
    m = ts.StaticWeights(ts.THETA0)
    for value in (1.0, 2.0, 4.0):  # θ varying over the window (as with a policy)
        step = _step(0)
        step.theta = torch.full((1, 6), value, dtype=torch.float64)
        log.add_step(step, None, 1, {"loss": 1.0})
    row = log.end_update(1, m, None, False)
    assert row["theta_q_n"] == pytest.approx(7.0 / 3) and row["theta_min_q_n"] == 1.0 and row["theta_max_q_n"] == 4.0
    assert row["theta_scale"] == pytest.approx(7.0 / 3)


@pytest.fixture(scope="module")
def rti_setup(ts, tmp_path_factory):
    from types import SimpleNamespace
    pytest.importorskip("leap_c.torch")
    args = SimpleNamespace(scenario="ground_vehicle_racing_rl", agent="agent_1", out=tmp_path_factory.mktemp("rti"))
    return ts.build_setup(args, ts.TrainConfig())  # (ref, plant, mpc): SQP_RTI, compiles ~40 s


def test_policy_with_frozen_network_reproduces_static(ts, rti_setup, tmp_path):
    """The fair-comparison guarantee of 7.5: a policy whose network beyond the last-layer bias is frozen
    (lr_net = 0) trains exactly like the static softplus weights with the same settings."""
    ref, plant, mpc = rti_setup
    common = dict(laps=2, weight_param="softplus", lr=0.1, loss_cfg=ts.balanced_loss_config(
        {"n": 1.0, "v": 1.0, "jerk": 0.0, "steer_rate": 0.0}))
    rows = {}
    for kind in ("static", "policy"):
        cfg = ts.TrainConfig(model=kind, lr_net=0.0, **common)
        out = tmp_path / kind
        out.mkdir()
        rows[kind] = ts.train(ts.build_model(cfg, ref), plant, mpc, cfg, out).rows
    assert len(rows["static"]) == len(rows["policy"]) == 2
    for a, b in zip(rows["static"], rows["policy"]):
        for key in ("loss", "L_n", "L_v", "lap_time", *(f"theta_next_{n}" for n in ts.THETA_NAMES)):
            assert b[key] == pytest.approx(a[key], rel=1e-12, abs=1e-15), key
