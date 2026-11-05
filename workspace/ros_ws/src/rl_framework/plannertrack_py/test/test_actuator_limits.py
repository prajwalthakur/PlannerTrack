# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""clip_control_in_sim in SingleTrackAugDynModel (steps.md Step 3.3): input clipping and end stops."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from plannertrack_py import Sim

from conftest import SIM_YAML, aug_agent, dump

LIMITS = {"max_jerk": 50.0, "max_steer_rate": 3.2, "max_steer": 0.4189, "max_accel": 9.51,
          "max_decel": 8.74, "max_speed": 20.0}
STEER, ACC = 6, 7
X0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0])
RL_AGENTS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params" / "agents.yaml"


def make(clip=True, limits=LIMITS):
    agent = aug_agent(clip_control_in_sim=clip)
    if limits is not None:
        agent["vehicle_limits"] = dict(limits)
    sim = Sim(SIM_YAML, dump(agent))
    sim.set_state(X0)
    return sim


def test_off_by_default_and_when_false():
    for agent in (aug_agent(), aug_agent(clip_control_in_sim=False)):
        agent["vehicle_limits"] = dict(LIMITS)
        sim = Sim(SIM_YAML, dump(agent))
        sim.set_state(X0)
        x = sim.step(np.array([100.0, 0.0]))  # 2x max_jerk for 10 ms
        assert x[ACC] == pytest.approx(1.0)


def test_inputs_are_clipped():
    sim = make()
    x = sim.step(np.array([100.0, -10.0]))  # 10 ms
    assert x[ACC] == pytest.approx(LIMITS["max_jerk"] * 0.01)
    assert x[STEER] == pytest.approx(-LIMITS["max_steer_rate"] * 0.01)


def test_steer_end_stop_is_exact_and_releases():
    sim = make()
    x = sim.step(np.array([0.0, 3.2]), n=50)  # 0.5 s at max rate: would reach 1.6 rad
    assert x[STEER] == LIMITS["max_steer"]  # held exactly at the stop (projection after each sub-step)
    x = sim.step(np.array([0.0, -1.0]))  # turning back works immediately
    assert x[STEER] == pytest.approx(LIMITS["max_steer"] - 0.01, abs=1e-12)
    x = sim.step(np.array([0.0, -3.2]), n=100)
    assert x[STEER] == -LIMITS["max_steer"]


def test_acc_end_stops():
    sim = make()
    x = sim.step(np.array([50.0, 0.0]), n=40)  # +20 m/s^2 requested
    assert x[ACC] == LIMITS["max_accel"]
    x = sim.step(np.array([-50.0, 0.0]), n=60)
    assert x[ACC] == -LIMITS["max_decel"]
    x = sim.step(np.array([10.0, 0.0]))
    assert x[ACC] == pytest.approx(-LIMITS["max_decel"] + 0.1, abs=1e-12)


def test_xdot_applies_the_same_limits():
    sim = make()
    at_stop = X0.copy()
    at_stop[STEER], at_stop[ACC] = LIMITS["max_steer"], -LIMITS["max_decel"]
    f = sim.xdot(at_stop, np.array([-20.0, 1.0]))  # both push outward: stopped
    assert f[STEER] == 0.0 and f[ACC] == 0.0
    f = sim.xdot(at_stop, np.array([20.0, -1.0]))  # both pull inward: free
    assert f[STEER] == -1.0 and f[ACC] == 20.0
    f = sim.xdot(X0, np.array([80.0, 5.0]))  # inside the limits: input clipped
    assert f[ACC] == LIMITS["max_jerk"] and f[STEER] == LIMITS["max_steer_rate"]


def test_identical_to_unclipped_plant_inside_the_limits():
    """Within the limits the clipped plant runs the same RK4 numerics: bit-identical states."""
    clipped, free = make(clip=True), make(clip=False)
    rng = np.random.default_rng(0)
    for _ in range(100):
        u = np.array([rng.uniform(-20, 20), rng.uniform(-1.0, 1.0)])
        np.testing.assert_array_equal(clipped.step(u), free.step(u))
    assert abs(clipped.get_state()[STEER]) < LIMITS["max_steer"]


@pytest.mark.parametrize("limits", [None, {k: v for k, v in LIMITS.items() if k != "max_jerk"},
                                    {**LIMITS, "max_decel": -8.74}])
def test_invalid_config_raises(limits):
    with pytest.raises((ValueError, RuntimeError)):
        make(clip=True, limits=limits)


def test_rl_scenario_enables_it():
    agent = yaml.safe_load(RL_AGENTS.read_text())["agents"]["agent_1"]
    assert agent["dynamics_params"]["clip_control_in_sim"] is True
