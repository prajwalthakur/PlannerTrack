# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""SingleTrackAugDynModel command_mode: acceleration (pid_controller in ground_vehicle_racing).

u[0] is an acceleration command a_cmd; the acc state follows it through a first-order
actuator, jerk = (a_cmd - acc) / accelTimeConstant.
"""

from pathlib import Path

import numpy as np
import pytest
import yaml

from plannertrack_py import Sim

from conftest import SIM_YAML, aug_agent, dump
from test_actuator_limits import LIMITS

VX, STEER, ACC = 3, 6, 7
TAU = 0.05
X0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0])
RACING_AGENTS = (Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing"
                 / "params" / "agents.yaml")


def make(clip=False, **overrides):
    agent = aug_agent(command_mode="acceleration", accelTimeConstant=TAU,
                      clip_control_in_sim=clip, **overrides)
    agent["vehicle_limits"] = dict(LIMITS)
    sim = Sim(SIM_YAML, dump(agent))
    sim.set_state(X0)
    return sim


def test_acc_follows_command_with_first_order_lag():
    sim, a_cmd, t = make(), 2.0, 0.2
    x = sim.step(np.array([a_cmd, 0.0]), n=20)
    assert x[ACC] == pytest.approx(a_cmd * (1.0 - np.exp(-t / TAU)), abs=1e-9)
    x = sim.step(np.array([a_cmd, 0.0]), n=200)
    assert x[ACC] == pytest.approx(a_cmd, abs=1e-9)


def test_xdot_is_actuator_rate():
    sim = make()
    state = X0.copy()
    state[ACC] = 1.0
    f = sim.xdot(state, np.array([3.0, 0.4]))
    assert f[ACC] == pytest.approx((3.0 - 1.0) / TAU)
    assert f[STEER] == 0.4


def test_commanding_current_acc_holds_it():
    sim = make()
    state = X0.copy()
    state[ACC] = 0.7
    sim.set_state(state)
    x = sim.step(np.array([0.7, 0.0]), n=50)
    assert x[ACC] == 0.7


def test_clipping_limits_command_and_jerk():
    sim = make(clip=True)
    x = sim.step(np.array([100.0, 0.0]))  # (100 - 0) / TAU >> max_jerk for 10 ms
    assert x[ACC] == pytest.approx(LIMITS["max_jerk"] * 0.01)
    x = sim.step(np.array([100.0, 0.0]), n=300)  # a_cmd clipped to max_accel
    assert x[ACC] == pytest.approx(LIMITS["max_accel"], abs=1e-6)
    f = sim.xdot(X0, np.array([-100.0, 0.0]))
    assert f[ACC] == -LIMITS["max_jerk"]


def test_jerk_mode_is_default():
    sim = Sim(SIM_YAML, dump(aug_agent()))
    sim.set_state(X0)
    assert sim.xdot(X0, np.array([2.0, 0.0]))[ACC] == 2.0


@pytest.mark.parametrize("overrides", [{"command_mode": "torque"},
                                       {"command_mode": "acceleration"},
                                       {"command_mode": "acceleration", "accelTimeConstant": 0.0}])
def test_invalid_config_raises(overrides):
    with pytest.raises((ValueError, RuntimeError)):
        Sim(SIM_YAML, dump(aug_agent(**overrides)))


def test_racing_scenario_uses_it():
    params = yaml.safe_load(RACING_AGENTS.read_text())["agents"]["agent_1"]
    assert params["dynamics_plugin"] == "SingleTrackAugDynModel"
    assert params["dynamics_params"]["command_mode"] == "acceleration"
