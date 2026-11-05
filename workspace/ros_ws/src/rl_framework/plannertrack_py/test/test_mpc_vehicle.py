# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""plannertrack_py.vehicle (steps.md Step 2.2): one source of vehicle parameters."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from conftest import SIM_YAML, aug_agent, dump, kinematic_agent
from plannertrack_py import STATE_NAMES, CONTROL_NAMES, Sim
from plannertrack_py.vehicle import (
    VehicleParams,
    agent_with_vehicle_params,
    load_vehicle_params,
    vehicle_params_from_agent,
)

AGENTS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params" / "agents.yaml"


def test_loads_the_scenario_values():
    p = load_vehicle_params(AGENTS, "agent_1")
    assert p == VehicleParams(mass=3.47, Iz=0.04712, lf=0.17, lr=0.17, cf=4.718, cr=5.4562,
                              mu=1.0489, hcg=0.074)
    assert p.wheelbase == pytest.approx(0.34)


def test_missing_mu_and_hcg_default_like_the_cpp_plugin():
    # the C++ plugin uses mu = 1, hcg = 0 when the keys are absent: the plant built without
    # them must behave exactly like the plant built with those values written out
    agent = aug_agent()  # conftest agent has neither mu nor hcg
    p = vehicle_params_from_agent(agent)
    assert (p.mu, p.hcg) == (1.0, 0.0)
    explicit = agent_with_vehicle_params(agent, p)
    x = np.array([0.0, 0.0, 0.2, 5.0, 0.1, 0.3, 0.05, -2.0])
    u = np.array([1.0, 0.2])
    np.testing.assert_array_equal(Sim(SIM_YAML, dump(agent)).xdot(x, u),
                                  Sim(SIM_YAML, dump(explicit)).xdot(x, u))


def test_round_trip_and_perturbed_plant():
    agent = aug_agent(mu=1.0489, hcg=0.074)
    nominal = vehicle_params_from_agent(agent)
    heavy = nominal.with_overrides(mass=1.2 * nominal.mass, mu=0.9 * nominal.mu)
    heavy_agent = agent_with_vehicle_params(agent, heavy)
    assert vehicle_params_from_agent(heavy_agent) == heavy
    assert agent["dynamics_params"]["mass"] == nominal.mass  # input not modified
    # the perturbed plant really is different (lateral dynamics depend on mass and mu)
    x = np.array([0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.1, 0.0])
    u = np.zeros(2)
    assert not np.allclose(Sim(SIM_YAML, dump(agent)).xdot(x, u),
                           Sim(SIM_YAML, dump(heavy_agent)).xdot(x, u))


def test_rejects_inconsistent_or_unsupported_agents():
    with pytest.raises(ValueError, match="vehWheelBase"):
        vehicle_params_from_agent(aug_agent(vehWheelBase=0.30))
    with pytest.raises(ValueError, match="single-track"):
        vehicle_params_from_agent(kinematic_agent())
    agent = aug_agent()
    del agent["dynamics_params"]["Iz"]
    with pytest.raises(ValueError, match="Iz"):
        vehicle_params_from_agent(agent)


def test_model_layout_matches_the_plugin():
    # the CasADi model's layout must be the plugin's layout
    from diff_weight_mpc.mpc import model

    assert (model.NX, model.NU) == (8, 2)
    assert model.STATE_NAMES == STATE_NAMES["SingleTrackAugDynModel"]
    assert model.INPUT_NAMES == CONTROL_NAMES["SingleTrackAugDynModel"]
