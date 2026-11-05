# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Stage-1 checks for plannertrack_py.Sim: API contract, determinism, physics sanity."""

from pathlib import Path

import numpy as np
import pytest

from conftest import SIM_YAML, WHEELBASE, dump, kinematic_agent, single_track_agent
from plannertrack_py import CONTROL_NAMES, STATE_NAMES, Sim, make_sim

_RACING_PARAMS = Path("scenarios/ground_vehicle_racing_rl/params")
# Nearest ancestor holding the scenarios package, so the test survives folder moves.
SCENARIO_PARAMS = next(
    (p / _RACING_PARAMS for p in Path(__file__).resolve().parents if (p / _RACING_PARAMS).exists()),
    None,
)

# Single-track layout: x, y, yaw, vx, vy, yaw_rate, steer
X, Y, YAW, VX, VY, R, STEER = range(7)


def st_state(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, r=0.0, steer=0.0):
    return np.array([x, y, yaw, vx, vy, r, steer])


# ---------------------------------------------------------------------------
# API contract
# ---------------------------------------------------------------------------


def test_dimensions_match_plugin_layout(st_sim, kin_sim):
    assert (st_sim.nx, st_sim.nu, st_sim.dt) == (7, 2, 0.01)
    assert st_sim.dynamics_plugin == "SingleTrackDynStateModel"
    assert len(STATE_NAMES[st_sim.dynamics_plugin]) == st_sim.nx
    assert len(CONTROL_NAMES[st_sim.dynamics_plugin]) == st_sim.nu

    assert (kin_sim.nx, kin_sim.nu) == (5, 2)
    assert len(STATE_NAMES[kin_sim.dynamics_plugin]) == kin_sim.nx


def test_initial_state_comes_from_yaml(st_sim):
    np.testing.assert_array_equal(st_sim.get_state(), st_state(x=1.0, y=-2.0, yaw=0.5))


def test_reset_restores_initial_state(st_sim):
    initial = st_sim.get_state()
    st_sim.step(np.array([1.0, 0.2]), n=50)
    assert not np.array_equal(st_sim.get_state(), initial)
    st_sim.reset()
    np.testing.assert_array_equal(st_sim.get_state(), initial)


def test_set_state_round_trip(st_sim):
    x = st_state(x=3.0, y=4.0, yaw=-1.0, vx=2.0, vy=0.1, r=0.3, steer=0.05)
    st_sim.set_state(x)
    np.testing.assert_array_equal(st_sim.get_state(), x)


def test_get_state_returns_a_copy(st_sim):
    s = st_sim.get_state()
    s[:] = 99.0
    assert st_sim.get_state()[X] == 1.0


@pytest.mark.parametrize("bad_u", [np.zeros(1), np.zeros(3)])
def test_wrong_input_size_raises(st_sim, bad_u):
    with pytest.raises(ValueError, match="input"):
        st_sim.step(bad_u)


def test_wrong_state_size_raises(st_sim):
    with pytest.raises(ValueError, match="state"):
        st_sim.set_state(np.zeros(5))


def test_non_positive_n_raises(st_sim):
    with pytest.raises(ValueError, match="n must be"):
        st_sim.step(np.zeros(2), n=0)


def test_missing_key_raises():
    agent = single_track_agent()
    del agent["dynamics_params"]["numControlInputs"]
    with pytest.raises(ValueError, match="numControlInputs"):
        Sim(SIM_YAML, dump(agent))


def test_unknown_plugin_raises():
    agent = single_track_agent()
    agent["dynamics_plugin"] = "NoSuchModel"
    with pytest.raises(RuntimeError):
        Sim(SIM_YAML, dump(agent))


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------


def _rollout(sim, inputs):
    sim.reset()
    return np.stack([sim.step(u) for u in inputs])


def test_two_instances_are_bitwise_identical():
    rng = np.random.default_rng(0)
    inputs = np.column_stack([rng.uniform(-2, 3, 300), rng.uniform(-1, 1, 300)])
    a = Sim(SIM_YAML, dump(single_track_agent()))
    b = Sim(SIM_YAML, dump(single_track_agent()))
    np.testing.assert_array_equal(_rollout(a, inputs), _rollout(b, inputs))
    # and repeatable on the same instance after reset
    np.testing.assert_array_equal(_rollout(a, inputs), _rollout(a, inputs))


def test_step_n_equals_n_single_steps(st_sim):
    u = np.array([1.5, 0.4])
    st_sim.reset()
    batched = st_sim.step(u, n=25)
    st_sim.reset()
    for _ in range(25):
        single = st_sim.step(u)
    np.testing.assert_array_equal(batched, single)


# ---------------------------------------------------------------------------
# Physics sanity
# ---------------------------------------------------------------------------


def test_straight_line_at_constant_speed(st_sim):
    st_sim.set_state(st_state(vx=2.0))
    s = st_sim.step(np.zeros(2), n=100)  # 1 s
    assert s[X] == pytest.approx(2.0, abs=1e-9)
    assert s[[Y, YAW, VY, R, STEER]] == pytest.approx(np.zeros(5), abs=1e-12)
    assert s[VX] == pytest.approx(2.0, abs=1e-12)


def test_acceleration_integrates_speed(st_sim):
    st_sim.set_state(st_state(vx=1.0))
    s = st_sim.step(np.array([0.5, 0.0]), n=200)  # 2 s at 0.5 m/s^2
    assert s[VX] == pytest.approx(2.0, abs=1e-9)
    assert s[X] == pytest.approx(1.0 * 2.0 + 0.5 * 0.5 * 2.0**2, abs=1e-9)


def test_steering_rate_integrates_steer(st_sim):
    st_sim.set_state(st_state())
    s = st_sim.step(np.array([0.0, 0.3]), n=100)  # 1 s at 0.3 rad/s
    assert s[STEER] == pytest.approx(0.3, abs=1e-9)


def test_low_speed_turn_matches_kinematic_curvature(st_sim):
    """Below 0.5 m/s the model is fully kinematic: yaw_rate / vx -> tan(steer) / L."""
    steer = 0.2
    st_sim.set_state(st_state(vx=0.3, steer=steer))
    s = st_sim.step(np.zeros(2), n=200)  # 2 s >> 0.1 s manifold relaxation
    assert s[VX] < 0.5  # still in the kinematic regime
    assert s[R] / s[VX] == pytest.approx(np.tan(steer) / WHEELBASE, rel=1e-3)
    assert s[VY] == pytest.approx(0.17 * s[R], rel=1e-3)  # vy = lr * yaw_rate


def test_high_speed_turn_understeers(st_sim):
    """Above 2.5 m/s the dynamic tire model applies; with cr*lr > cf*lf the car
    understeers, so yaw rate stays below the kinematic value."""
    steer, vx = 0.1, 5.0
    st_sim.set_state(st_state(vx=vx, steer=steer))
    s = st_sim.step(np.zeros(2), n=100)
    kinematic_r = s[VX] * np.tan(steer) / WHEELBASE
    assert 0.0 < s[R] < kinematic_r


def test_kinematic_plugin_turn_rate_is_exact(kin_sim):
    steer, vx = 0.25, 1.5
    kin_sim.set_state(np.array([0.0, 0.0, 0.0, vx, steer]))
    s = kin_sim.step(np.zeros(2), n=100)  # 1 s, constant v and steer
    assert s[2] == pytest.approx(vx * np.tan(steer) / WHEELBASE * 1.0, rel=1e-9)


# ---------------------------------------------------------------------------
# Real scenario files
# ---------------------------------------------------------------------------


@pytest.mark.skipif(SCENARIO_PARAMS is None, reason="scenario params not found")
def test_make_sim_from_racing_scenario():
    # default plant for learning: the augmented single-track model
    sim = make_sim(SCENARIO_PARAMS / "sim.yaml", SCENARIO_PARAMS / "agents.yaml", "agent_1")
    assert sim.dynamics_plugin == "SingleTrackAugDynModel"
    assert sim.nx == 8
    sim.step(np.array([1.0, 0.0]), n=10)  # [jerk, steer_rate]
    assert np.all(np.isfinite(sim.get_state()))


@pytest.mark.skipif(SCENARIO_PARAMS is None, reason="scenario params not found")
def test_rl_scenario_declares_the_augmented_plant():
    # ground_vehicle_racing_rl/agents.yaml itself runs SingleTrackAugDynModel
    sim = make_sim(SCENARIO_PARAMS / "sim.yaml", SCENARIO_PARAMS / "agents.yaml", "agent_1",
                   dynamics_plugin=None)
    assert sim.dynamics_plugin == "SingleTrackAugDynModel"
    assert sim.nx == 8


@pytest.mark.skipif(SCENARIO_PARAMS is None, reason="scenario params not found")
def test_make_sim_can_convert_back_to_the_acceleration_plant():
    sim = make_sim(SCENARIO_PARAMS / "sim.yaml", SCENARIO_PARAMS / "agents.yaml", "agent_1",
                   dynamics_plugin="SingleTrackDynStateModel")
    assert sim.dynamics_plugin == "SingleTrackDynStateModel"
    assert sim.nx == 7
