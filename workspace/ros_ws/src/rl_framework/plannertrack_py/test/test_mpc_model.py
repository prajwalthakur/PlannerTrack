# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""CasADi prediction model vs the C++ plant (steps.md Steps 2.4, 2.5, 2.7).

The MPC model (diff_weight_mpc.mpc.model) must equal SingleTrackAugDynModel::xdot() wherever the
MPC operates (vx >= MIN_VALID_SPEED), so that the only model mismatch in the experiments is the
deliberate one (mass, mu).
"""

from pathlib import Path

import casadi as ca
import numpy as np
import pytest
import yaml

from plannertrack_py import Sim, load_agent_yaml, load_raceline_limits
from plannertrack_py.vehicle import agent_with_vehicle_params, load_vehicle_params
from diff_weight_mpc.mpc.model import (
    MIN_VALID_SPEED,
    NU,
    NX,
    PARAMETER_NAMES,
    dynamics_expr,
    dynamics_function,
    parameter_values,
)
from plannertrack_py.plant import PERTURBABLE

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
AGENTS, SIM_YAML, PLANNER = PARAMS / "agents.yaml", PARAMS / "sim.yaml", PARAMS / "min_curv_planner.yaml"

X, Y, PSI, VX, VY, R, DELTA, ACC = range(NX)
N_SAMPLES = 1000


@pytest.fixture(scope="module")
def vp():
    return load_vehicle_params(AGENTS, "agent_1")


@pytest.fixture(scope="module")
def f(vp):
    return dynamics_function(vp)


@pytest.fixture(scope="module")
def sim():
    return Sim(SIM_YAML.read_text(), load_agent_yaml(AGENTS, "agent_1", "SingleTrackAugDynModel"))


@pytest.fixture(scope="module")
def limits():
    return load_raceline_limits(AGENTS, "agent_1", PLANNER)


def sample(rng, lim, n, vx_range):
    """Random states/inputs over the MPC's operating range (bounds from the scenario's limits)."""
    x = np.column_stack([
        rng.uniform(-5.0, 5.0, n),                          # X
        rng.uniform(-5.0, 5.0, n),                          # Y
        rng.uniform(-np.pi, np.pi, n),                      # psi
        rng.uniform(*vx_range, n),                          # vx
        rng.uniform(-0.5, 0.5, n),                          # vy
        rng.uniform(-2.0, 2.0, n),                          # yaw rate
        rng.uniform(-lim.max_steer, lim.max_steer, n),      # steer
        rng.uniform(-lim.max_decel, lim.max_accel, n),      # acc
    ])
    u = np.column_stack([
        rng.uniform(-lim.max_jerk, lim.max_jerk, n),        # jerk
        rng.uniform(-lim.max_steer_rate, lim.max_steer_rate, n),  # steer rate
    ])
    return x, u


def casadi_xdot(f, x, u, p):
    return np.asarray(f(x, u, p)).ravel()


# ------------------------------------------------------------------ 2.4 derivatives


def test_derivatives_match_the_plant_above_min_speed(f, sim, vp, limits):
    rng = np.random.default_rng(0)
    xs, us = sample(rng, limits, N_SAMPLES, (MIN_VALID_SPEED, 12.0))
    worst = 0.0
    for x, u in zip(xs, us):
        ref = sim.xdot(x, u)
        err = np.abs(casadi_xdot(f, x, u, parameter_values(vp)) - ref)
        worst = max(worst, float(np.max(err / (1.0 + np.abs(ref)))))
    print(f"worst relative derivative error over {N_SAMPLES} samples: {worst:.2e}")
    assert worst < 1e-10


def test_exact_at_the_min_speed_boundary(f, sim, vp):
    # at vx = 2.5 m/s the plant's blend factor is exactly 1 (fully dynamic)
    x = np.array([0.0, 0.0, 0.4, MIN_VALID_SPEED, 0.1, 0.6, 0.2, -2.0])
    u = np.array([5.0, 0.5])
    np.testing.assert_allclose(casadi_xdot(f, x, u, parameter_values(vp)), sim.xdot(x, u), atol=1e-12)


def test_differs_below_min_speed(f, sim, vp, limits):
    """Documents the model's validity range: below 2.5 m/s the plant blends towards the kinematic
    model and the MPC model does not."""
    rng = np.random.default_rng(1)
    xs, us = sample(rng, limits, 50, (0.7, 2.3))
    diffs = [np.max(np.abs(casadi_xdot(f, x, u, parameter_values(vp)) - sim.xdot(x, u))[[VY, R]])
             for x, u in zip(xs, us)]
    assert min(diffs) > 1e-3


# ------------------------------------------------------------------ 2.5 trajectories


def rk4(fun, x, u, h, n):
    for _ in range(n):
        k1 = fun(x, u)
        k2 = fun(x + h / 2 * k1, u)
        k3 = fun(x + h / 2 * k2, u)
        k4 = fun(x + h * k3, u)
        x = x + h * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6)
    return x


def test_trajectory_matches_the_plant(f, sim, vp):
    """2 s of driving above 3 m/s: RK4 of the CasADi model at the plant's 1 ms sub-step equals
    Sim.step (the trajectory the MPC predicts is the one the plant drives)."""
    model = lambda x, u: casadi_xdot(f, x, u, parameter_values(vp))  # noqa: E731
    x_plant = np.array([0.0, 0.0, 0.3, 6.0, 0.0, 0.0, 0.0, 0.0])
    x_model = x_plant.copy()
    sim.set_state(x_plant)
    rng = np.random.default_rng(2)
    for _ in range(200):  # 200 x 10 ms
        u = np.array([rng.uniform(-10, 10), rng.uniform(-1.0, 1.0)])
        x_plant = sim.step(u)
        x_model = rk4(model, x_model, u, 0.001, 10)
        assert x_plant[VX] > MIN_VALID_SPEED  # stays in the valid range
    np.testing.assert_allclose(x_model, x_plant, rtol=0, atol=1e-9)


# ------------------------------------------------------------------ 2.7 model parameters


def test_parameter_vector_is_the_perturbable_set(vp):
    assert PARAMETER_NAMES == PERTURBABLE == ("mass", "mu", "Iz", "cf", "cr")
    np.testing.assert_array_equal(parameter_values(vp), [vp.mass, vp.mu, vp.Iz, vp.cf, vp.cr])


@pytest.mark.parametrize("scales", [{"mass": 1.2}, {"mu": 0.8}, {"Iz": 1.3}, {"cf": 0.8}, {"cr": 1.25},
                                    {"mass": 1.2, "Iz": 1.2, "cf": 1 / 1.2, "cr": 1 / 1.2},
                                    {"mass": 0.9, "mu": 1.15, "Iz": 0.9, "cf": 1.1, "cr": 0.95}])
def test_parameters_match_a_perturbed_plant(f, vp, limits, scales):
    """Every model parameter (mass, mu, Iz, cf, cr) is wired like the plant's: the model with the
    perturbed parameter vector equals a plant built with the same values (the mismatch experiments and
    the oracle rely on this)."""
    perturbed = vp.with_overrides(**{k: s * getattr(vp, k) for k, s in scales.items()})
    agent = yaml.safe_load(load_agent_yaml(AGENTS, "agent_1", "SingleTrackAugDynModel"))
    plant = Sim(SIM_YAML.read_text(), yaml.safe_dump(agent_with_vehicle_params(agent, perturbed)))
    rng = np.random.default_rng(3)
    xs, us = sample(rng, limits, 100, (MIN_VALID_SPEED, 12.0))
    for x, u in zip(xs, us):
        ref = plant.xdot(x, u)
        np.testing.assert_allclose(casadi_xdot(f, x, u, parameter_values(perturbed)), ref,
                                   rtol=1e-10, atol=1e-10)
    # and the nominal model does NOT match the perturbed plant (the mismatch is real)
    x, u = xs[0], us[0]
    assert not np.allclose(casadi_xdot(f, x, u, parameter_values(vp)), plant.xdot(x, u))


def test_single_overrides_and_unknown_names(vp):
    x, u = ca.SX.sym("x", NX), ca.SX.sym("u", NU)
    iz = ca.SX.sym("Iz")
    g = ca.Function("g", [x, u, iz], [dynamics_expr(x, u, vp, Iz=iz)])
    f_full = dynamics_function(vp)
    xv, uv = np.array([0.0, 0.0, 0.1, 6.0, 0.1, 0.5, 0.1, 1.0]), np.array([2.0, 0.3])
    np.testing.assert_allclose(np.asarray(g(xv, uv, 2 * vp.Iz)),
                               np.asarray(f_full(xv, uv, parameter_values(vp.with_overrides(Iz=2 * vp.Iz)))))
    with pytest.raises(ValueError):
        dynamics_expr(x, u, vp, lf=0.2)


def test_constant_parameters_equal_the_symbolic_ones(vp):
    # without p the params' values are baked in: same result as passing them as the parameter vector
    x, u = ca.SX.sym("x", NX), ca.SX.sym("u", NU)
    const = ca.Function("c", [x, u], [dynamics_expr(x, u, vp)])
    sym = dynamics_function(vp)
    xv = np.array([1.0, 2.0, 0.3, 7.0, 0.2, 0.8, -0.1, 3.0])
    uv = np.array([-4.0, 0.7])
    np.testing.assert_array_equal(np.asarray(const(xv, uv)), np.asarray(sym(xv, uv, parameter_values(vp))))


# ------------------------------------------------------------------ for acados


def test_derivatives_for_acados_are_finite(vp, limits):
    """acados uses the Jacobian (and, with the exact Hessian, second derivatives) of the model:
    they must be finite over the operating range (no kinks left)."""
    x, u = ca.SX.sym("x", NX), ca.SX.sym("u", NU)
    p = ca.SX.sym("p", len(PARAMETER_NAMES))
    xdot = dynamics_expr(x, u, vp, p=p)
    w = ca.vertcat(x, u)
    lam = ca.SX.sym("lam", NX)  # adjoint weights, as in the Lagrangian Hessian
    jac = ca.Function("J", [x, u, p], [ca.jacobian(xdot, w)])
    hess = ca.Function("H", [x, u, p, lam], [ca.hessian(ca.dot(lam, xdot), w)[0]])
    rng = np.random.default_rng(4)
    xs, us = sample(rng, limits, 200, (MIN_VALID_SPEED, 12.0))
    for xv, uv in zip(xs, us):
        assert np.all(np.isfinite(np.asarray(jac(xv, uv, parameter_values(vp)))))
        assert np.all(np.isfinite(np.asarray(hess(xv, uv, parameter_values(vp), rng.normal(size=NX)))))
