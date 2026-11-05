# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""The Diff-WMPC OCP (steps.md Step 4.1): building blocks, and the compiled leap-c solver.

The compiled solver is built once per module (~30 s of acados code generation). The tests solve with
SQP to convergence (the paper's SQP_RTI is the closed-loop setting, 4.3).
"""

from pathlib import Path

import casadi as ca
import numpy as np
import pytest

leap_c_torch = pytest.importorskip("leap_c.torch")
torch = pytest.importorskip("torch")

from plannertrack_py import ReferenceTrack, Sim, compute_raceline, load_agent_yaml  # noqa: E402
from diff_weight_mpc.mpc.initializer import ReferenceInitializer  # noqa: E402
from diff_weight_mpc.mpc.model import NU, NX, dynamics_expr, parameter_values  # noqa: E402
from diff_weight_mpc.mpc.ocp import (  # noqa: E402
    ERROR_NAMES,
    STAGE_REF_NAMES,
    PAPER_THETA0,
    THETA0,
    OcpConfig,
    build_ocp,
    discrete_dynamics,
    error_function,
    path_constraints,
)
from plannertrack_py.plant import PSI, Plant  # noqa: E402

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
CFG = OcpConfig(nlp_solver="SQP")
STARTS = (0.0, 5.48, 12.0, 20.0)


@pytest.fixture(scope="module")
def ref():
    return ReferenceTrack(compute_raceline(PARAMS / "agents.yaml", "agent_1", PARAMS / "min_curv_planner.yaml"))


@pytest.fixture(scope="module")
def plant(ref):
    return Plant.from_scenario(PARAMS, reference=ref, dt=0.02)


@pytest.fixture(scope="module")
def mpc(plant, tmp_path_factory):
    ocp, manager = build_ocp(plant.nominal_params, plant.limits, CFG, name="test_ocp")
    diff_mpc = leap_c_torch.AcadosDiffMpcTorch(
        ocp, manager, export_directory=tmp_path_factory.mktemp("acados_ocp"), n_batch_init=2,
        verbose=False, dtype=torch.float64)
    diff_mpc.diff_mpc_fun.initializer = ReferenceInitializer(
        diff_mpc.diff_mpc_fun.ocp, manager, plant.nominal_params.wheelbase)
    return diff_mpc


def stage_params(plant, s0, x0, cfg=CFG):
    """Stage references for one solve and x0 with its yaw on ψ_ref's branch (steps.md 4.2 will make
    this a module; here as a test helper). Returns (x0, {name: (1, N+1, 1)})."""
    st = plant.reference.stage_references(s0, cfg.N, cfg.dt)
    n_left, n_right = np.array([plant.track_limits(s) for s in st["s"]]).T
    values = {"x_ref": st["x"], "y_ref": st["y"], "psi_ref": st["psi"], "v_ref": st["vx"],
              "alat_ref": st["a_lat"], "n_left": n_left, "n_right": n_right}
    x0 = np.array(x0, dtype=float)
    x0[PSI] = st["psi"][0] + np.angle(np.exp(1j * (x0[PSI] - st["psi"][0])))
    return x0, {k: values[k].reshape(1, -1, 1) for k in STAGE_REF_NAMES}


def plan_errors(x, par):
    """n and e_v of a planned trajectory x (N+1, NX) against the stage references."""
    psi_r = par["psi_ref"][0, :, 0]
    n = (-np.sin(psi_r) * (x[:, 0] - par["x_ref"][0, :, 0]) + np.cos(psi_r) * (x[:, 1] - par["y_ref"][0, :, 0]))
    return n, x[:, 3] - par["v_ref"][0, :, 0]


def solve(mpc, plant, s0, **params):
    x0, _ = plant.reset(s0=s0)
    x0, par = stage_params(plant, s0, x0)
    ctx, u0, x, u, value = mpc(torch.tensor(x0[None]), params={**par, **params})
    return ctx, u0, x.detach().numpy()[0], u.detach().numpy()[0], value, par


# ------------------------------------------------------------------ building blocks (no compile)


def test_error_function_matches_the_plant(plant):
    """The OCP's n, e_psi, e_v at the plant's state and its projection's reference point equal the
    plant's own Frenet info (Plant.step: n, dpsi, dv). n agrees to ~1e-6: the plant projects onto the
    raceline's polyline segment (its direction), the OCP uses the interpolated psi_ref at s."""
    f = error_function()
    for s0 in STARTS:
        plant.reset(s0=s0)
        for _ in range(3):
            x, info = plant.step([2.0, 0.3])
        r = plant.reference.sample(info["s"])
        n_l, n_r = plant.track_limits(info["s"])
        refv = [float(r["x"]), float(r["y"]), float(r["psi"]), float(r["vx"]), float(r["a_lat"]), n_l, n_r]
        e = dict(zip(ERROR_NAMES, np.asarray(f(x, refv)).ravel()))
        assert e["n"] == pytest.approx(info["n"], abs=1e-5)
        assert e["e_psi"] == pytest.approx(info["dpsi"], abs=1e-9)  # both wrapped (plannertrack_py.utils)
        for k in (-1, 1):  # the OCP's heading error does not depend on the yaw's 2π branch
            x_k = np.array(x, dtype=float)
            x_k[PSI] += 2.0 * np.pi * k
            assert float(f(x_k, refv)[1]) == pytest.approx(e["e_psi"], abs=1e-9)
        assert e["e_v"] == pytest.approx(info["dv"], abs=1e-9)
        assert e["e_alat"] == pytest.approx(x[3] * x[5] - float(r["a_lat"]), abs=1e-12)


def test_path_constraints(plant):
    """h <= 0 on the reference; h0 > 0 beyond the left limit, h1 > 0 beyond the right one; a lower
    vehicle mu tightens the friction ellipse (h2 rises) at the same state."""
    lim, vp = plant.limits, plant.nominal_params
    x, r, p = ca.SX.sym("x", NX), ca.SX.sym("r", len(STAGE_REF_NAMES)), ca.SX.sym("p", 5)
    h = ca.Function("h", [x, r, p], [path_constraints(
        x, {k: r[i] for i, k in enumerate(STAGE_REF_NAMES)}, p, lim, vp.mu)])
    s0 = 5.48
    x0, _ = plant.reset(s0=s0)
    rr = plant.reference.sample(s0)
    n_l, n_r = plant.track_limits(s0)
    refv = [float(rr["x"]), float(rr["y"]), float(rr["psi"]), float(rr["vx"]), float(rr["a_lat"]), n_l, n_r]
    nominal = parameter_values(vp)
    h_on = np.asarray(h(x0, refv, nominal)).ravel()
    assert np.all(h_on < 0.0)
    psi = float(rr["psi"])
    for row, shift in ((0, n_l + 0.05), (1, n_r - 0.05)):
        xs = x0.copy()
        xs[0] -= shift * np.sin(psi)
        xs[1] += shift * np.cos(psi)
        assert np.asarray(h(xs, refv, nominal)).ravel()[row] == pytest.approx(0.05, abs=1e-6)
    low_mu = nominal.copy()
    low_mu[1] *= 0.8
    assert np.asarray(h(x0, refv, low_mu)).ravel()[2] > h_on[2]


def test_discrete_dynamics_match_the_plant(plant):
    """The OCP's discretization (RK4, 4 steps per interval) over one interval vs the plant (RK4 at
    1 ms). 0.08 s = 8 plant steps is used instead of 0.075 s (not a multiple of the 10 ms sim step);
    the step size, 20 ms, is the OCP's (0.075 / 4 = 18.75 ms)."""
    vp = plant.nominal_params
    x, u = ca.SX.sym("x", NX), ca.SX.sym("u", NU)
    step = ca.Function("F", [x, u], [discrete_dynamics(lambda xk, uk: dynamics_expr(xk, uk, vp), x, u, 0.08, 4)])
    sim = Sim((PARAMS / "sim.yaml").read_text(), load_agent_yaml(PARAMS / "agents.yaml", "agent_1",
                                                                  "SingleTrackAugDynModel"))
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(20):
        x0 = np.array([0.0, 0.0, rng.uniform(-3, 3), rng.uniform(3.0, 8.0), rng.uniform(-0.2, 0.2),
                       rng.uniform(-1.5, 1.5), rng.uniform(-0.3, 0.3), rng.uniform(-4, 4)])
        u0 = np.array([rng.uniform(-20, 20), rng.uniform(-1.5, 1.5)])
        sim.set_state(x0)
        worst = max(worst, np.abs(np.asarray(step(x0, u0)).ravel() - sim.step(u0, n=8)).max())
    assert worst < 5e-3  # measured 1.6e-3 (yaw rate) on these off-trim states; 2.6: 4.6e-4 near trim


# ------------------------------------------------------------------ compiled solver


# A tracking-heavy theta for correctness checks. The paper's theta0 is scaled for a full-size car: on
# our 1:10 car its jerk weight r_j = 4.3 dominates (j = 30 m/s^3 costs ~1900 per stage vs ~1 for a
# 1 m/s speed error), so the plan brakes too little before corners (a ~ -4 instead of -6.6 m/s^2),
# arrives too fast and runs wide. Re-scaling theta0 is steps.md 4.5.
TRACKING_THETA = np.array(THETA0)  # = OcpConfig's default since 4.5: q_n 50, r_j 0.01


def own_projection(plant, x, s0):
    """(s, n) of every planned point at its OWN projection onto the raceline (not against the stage
    reference point, which a plan that is ahead of or behind schedule has not reached)."""
    st = plant.reference.stage_references(s0, CFG.N, CFG.dt)
    proj = [plant.reference.project(xi[0], xi[1], s_guess=si % plant.reference.length, window=3.0)
            for xi, si in zip(x, st["s"])]
    return np.array([q.s for q in proj]), np.array([q.n for q in proj])


@pytest.mark.parametrize("s0", STARTS)
def test_converges_with_the_paper_weights(mpc, plant, s0):
    """With the paper's theta0, SQP converges (status 0) from a trimmed start on the reference, within
    the actuator limits. (How well theta0 tracks on our car is 4.5.)"""
    ctx, u0, x, u, value, par = solve(mpc, plant, s0, theta=torch.tensor(np.array(PAPER_THETA0)[None]))
    assert ctx.status[0] == 0
    n, e_v = plan_errors(x, par)
    assert abs(n[0]) < 1e-9 and abs(e_v[0]) < 1e-9  # x0 is on the reference
    lim = plant.limits
    assert np.all(np.abs(u[:, 0]) <= lim.max_jerk + 1e-6) and np.all(np.abs(u[:, 1]) <= lim.max_steer_rate + 1e-6)
    assert np.all(np.abs(x[:, 6]) <= lim.max_steer + 1e-6)
    assert np.all(x[:, 7] >= -lim.max_decel - 1e-6) and np.all(x[:, 7] <= lim.max_accel + 1e-6)


@pytest.mark.parametrize("s0", STARTS)
def test_holds_the_line_with_tracking_weights(mpc, plant, s0):
    """With a tracking theta the plan stays on the line for the executed part (first 0.375 s) and inside
    the track over the whole 2.55 s horizon, including the braking zone before the hairpin (s0 = 20)."""
    ctx, u0, x, u, value, par = solve(mpc, plant, s0, theta=torch.tensor(TRACKING_THETA[None]))
    assert ctx.status[0] == 0
    s, n = own_projection(plant, x, s0)
    assert np.abs(n[:6]).max() < 0.05
    n_left, n_right = np.array([plant.track_limits(si) for si in s]).T
    assert np.all(n <= n_left + 0.02) and np.all(n >= n_right - 0.02)  # soft constraints: small slack


def test_theta_changes_the_plan(mpc, plant):
    """The weights reach the solution: a larger q_n pulls the plan onto the line, a larger r_omega
    smooths the steering."""
    theta0 = np.array(PAPER_THETA0)
    _, _, x0_, u0_, _, par = solve(mpc, plant, 12.0, theta=torch.tensor(theta0[None]))
    _, _, x_n, _, _, _ = solve(mpc, plant, 12.0, theta=torch.tensor((theta0 * [20, 1, 1, 1, 1, 1])[None]))
    _, _, _, u_w, _, _ = solve(mpc, plant, 12.0, theta=torch.tensor((theta0 * [1, 1, 1, 1, 1, 50])[None]))
    assert np.abs(plan_errors(x_n, par)[0]).mean() < 0.7 * np.abs(plan_errors(x0_, par)[0]).mean()
    assert np.abs(u_w[:, 1]).mean() < 0.7 * np.abs(u0_[:, 1]).mean()


def test_vehicle_parameters_change_the_plan(mpc, plant):
    """The non-differentiable vehicle parameters reach the dynamics (oracle with the true values)."""
    vehicle = np.tile(parameter_values(plant.nominal_params), (1, CFG.N + 1, 1))
    _, _, x_nom, _, _, _ = solve(mpc, plant, 5.48)
    low_mu = vehicle.copy()
    low_mu[..., 1] *= 0.8
    _, _, x_low, _, _, _ = solve(mpc, plant, 5.48, vehicle=low_mu)
    _, _, x_same, _, _, _ = solve(mpc, plant, 5.48, vehicle=vehicle)
    np.testing.assert_allclose(x_same, x_nom, atol=1e-8)  # nominal values = the default
    assert np.abs(x_low - x_nom).max() > 1e-2


def test_gradient_reaches_theta(mpc, plant):
    """leap-c backpropagates through the solver to theta (correctness vs finite differences is Step 5)."""
    theta = torch.tensor(np.array(CFG.theta0)[None], requires_grad=True)
    ctx, u0, x, u, value, par = solve(mpc, plant, 12.0, theta=theta)
    assert ctx.status[0] == 0
    value.sum().backward()
    assert theta.grad is not None and torch.all(torch.isfinite(theta.grad))
    assert torch.any(theta.grad != 0)


def test_batch_equals_single_solves(mpc, plant):
    """Two cars at different points of the track in one batched call = two single solves."""
    singles, x0s, pars = [], [], []
    for s0 in (5.48, 12.0):
        _, _, x, _, _, par = solve(mpc, plant, s0)
        singles.append(x)
        x0, _ = plant.reset(s0=s0)
        x0, par = stage_params(plant, s0, x0)
        x0s.append(x0)
        pars.append(par)
    batch_par = {k: np.concatenate([p[k] for p in pars]) for k in STAGE_REF_NAMES}
    ctx, _, xb, _, _ = mpc(torch.tensor(np.stack(x0s)), params=batch_par)
    assert list(ctx.status) == [0, 0]
    for i in range(2):
        np.testing.assert_allclose(xb.detach().numpy()[i], singles[i], atol=1e-6)
