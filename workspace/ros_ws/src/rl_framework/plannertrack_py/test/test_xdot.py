# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Sim.xdot (steps.md Step 2.1): the plant's continuous-time derivative, exposed for the MPC parity test."""

import numpy as np
import pytest

from conftest import SIM_YAML, aug_agent, dump, kinematic_agent, single_track_agent
from plannertrack_py import Sim


def rk4_step(f, x, u, h, n):
    """The simulator's integrator: n RK4 sub-steps of size h (project_utils IntegratorClass)."""
    for _ in range(n):
        k1 = f(x, u)
        k2 = f(x + (h / 2.0) * k1, u)
        k3 = f(x + (h / 2.0) * k2, u)
        k4 = f(x + h * k3, u)
        x = x + h * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0)
    return x


@pytest.mark.parametrize("make_agent", [aug_agent, single_track_agent, kinematic_agent])
def test_xdot_is_what_the_simulator_integrates(make_agent):
    """Integrating xdot with the simulator's own scheme (RK4, 10 x 1 ms per 10 ms step)
    reproduces Sim.step: xdot is exactly the plant's dynamics function."""
    sim = Sim(SIM_YAML, dump(make_agent()))
    rng = np.random.default_rng(1)
    for _ in range(20):
        x0 = sim.get_state() + rng.uniform(-0.2, 0.2, sim.nx)
        x0[3] = rng.uniform(0.5, 8.0)  # vx: kinematic, blended and dynamic regimes
        u = rng.uniform(-1.0, 1.0, sim.nu)
        sim.set_state(x0)
        stepped = sim.step(u)
        integrated = rk4_step(sim.xdot, x0, u, 0.001, 10)
        np.testing.assert_allclose(integrated, stepped, rtol=0, atol=1e-12)


def test_xdot_does_not_change_the_state(aug_sim):
    before = aug_sim.get_state()
    aug_sim.xdot(before + 1.0, np.array([5.0, 0.3]))
    np.testing.assert_array_equal(aug_sim.get_state(), before)


def test_xdot_straight_line_values(aug_sim):
    # x = [X, Y, psi, vx, vy, r, delta, a], u = [jerk, steer_rate]
    x = np.array([0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 1.5])
    d = aug_sim.xdot(x, np.array([2.0, 0.3]))
    assert d[0] == pytest.approx(4.0)  # X_dot = vx
    assert d[3] == pytest.approx(1.5)  # vx_dot = a (vy = r = 0)
    assert d[6] == pytest.approx(0.3)  # steer_dot = steer_rate
    assert d[7] == pytest.approx(2.0)  # a_dot = jerk


def test_xdot_wrong_sizes_raise(aug_sim):
    with pytest.raises(ValueError, match="state"):
        aug_sim.xdot(np.zeros(7), np.zeros(2))
    with pytest.raises(ValueError, match="input"):
        aug_sim.xdot(np.zeros(8), np.zeros(3))
