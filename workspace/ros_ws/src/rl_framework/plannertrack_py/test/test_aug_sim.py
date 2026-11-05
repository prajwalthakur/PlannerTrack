# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""SingleTrackAugDynModel: acceleration as a state, jerk as the input.

The vehicle physics is a copy of SingleTrackDynStateModel; the parity tests
below fail if the two copies drift apart.
"""

import numpy as np
import pytest

from conftest import SIM_YAML, dump, aug_agent, single_track_agent
from plannertrack_py import CONTROL_NAMES, STATE_NAMES, Sim, with_dynamics_plugin

X, Y, YAW, VX, VY, R, STEER, ACC = range(8)
JERK, SV = range(2)


def aug_state(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, r=0.0, steer=0.0, acc=0.0):
    return np.array([x, y, yaw, vx, vy, r, steer, acc])


def test_dimensions_and_names(aug_sim):
    assert (aug_sim.nx, aug_sim.nu, aug_sim.dt) == (8, 2, 0.01)
    assert aug_sim.dynamics_plugin == "SingleTrackAugDynModel"
    assert STATE_NAMES["SingleTrackAugDynModel"][ACC] == "acc"
    assert CONTROL_NAMES["SingleTrackAugDynModel"] == ("jerk", "steer_rate")


def test_initial_state_includes_acc():
    sim = Sim(SIM_YAML, dump(aug_agent(initAcc=0.7)))
    np.testing.assert_array_equal(sim.get_state(), aug_state(x=1.0, y=-2.0, yaw=0.5, acc=0.7))


def test_reset_restores_initial_state(aug_sim):
    initial = aug_sim.get_state()
    aug_sim.step(np.array([2.0, 0.2]), n=50)
    assert not np.array_equal(aug_sim.get_state(), initial)
    aug_sim.reset()
    np.testing.assert_array_equal(aug_sim.get_state(), initial)


def test_wrong_state_count_raises():
    with pytest.raises(ValueError, match="numStates must be 8"):
        Sim(SIM_YAML, dump(aug_agent(numStates=7)))


def test_jerk_integrates_acceleration_and_speed(aug_sim):
    a0, j, v0, t = 0.5, 1.0, 1.0, 2.0
    aug_sim.set_state(aug_state(vx=v0, acc=a0))
    s = aug_sim.step(np.array([j, 0.0]), n=200)  # 2 s
    assert s[ACC] == pytest.approx(a0 + j * t, abs=1e-9)
    assert s[VX] == pytest.approx(v0 + a0 * t + 0.5 * j * t**2, abs=1e-9)
    assert s[X] == pytest.approx(v0 * t + 0.5 * a0 * t**2 + j * t**3 / 6.0, abs=1e-9)


def test_steering_rate_integrates_steer(aug_sim):
    aug_sim.set_state(aug_state())
    s = aug_sim.step(np.array([0.0, 0.3]), n=100)
    assert s[STEER] == pytest.approx(0.3, abs=1e-9)


@pytest.mark.parametrize("vx", [0.5, 2.0, 5.0])  # kinematic, blended, dynamic regime
def test_matches_single_track_model_with_constant_acc(vx):
    """Zero jerk keeps acc constant: the augmented model must reproduce
    SingleTrackDynStateModel driven by that acc (same physics, same RK4)."""
    base = Sim(SIM_YAML, dump(single_track_agent()))
    aug = Sim(SIM_YAML, dump(aug_agent()))
    rng = np.random.default_rng(int(vx * 10))
    acc = 0.8
    state7 = np.array([0.0, 0.0, 0.3, vx, 0.05, 0.1, 0.05])
    base.set_state(state7)
    aug.set_state(np.append(state7, acc))
    for sv in rng.uniform(-0.5, 0.5, 100):
        sb = base.step(np.array([acc, sv]), n=5)
        sa = aug.step(np.array([0.0, sv]), n=5)
        np.testing.assert_allclose(sa[:7], sb, rtol=0, atol=1e-10)
        assert sa[ACC] == acc


def test_with_dynamics_plugin_converts_single_track_agent():
    agent = single_track_agent()
    aug = with_dynamics_plugin(agent, "SingleTrackAugDynModel")
    assert aug["dynamics_plugin"] == "SingleTrackAugDynModel"
    assert aug["dynamics_params"]["numStates"] == 8
    assert agent["dynamics_params"]["numStates"] == 7  # input not modified
    with pytest.raises(ValueError):
        kin = dict(agent, dynamics_plugin="BicycleKinematicModel")
        with_dynamics_plugin(kin, "SingleTrackAugDynModel")
