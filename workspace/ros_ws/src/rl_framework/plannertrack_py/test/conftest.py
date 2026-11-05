# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Self-contained configs, so tests don't change when a scenario is retuned."""

import sys
from pathlib import Path

import pytest
import yaml

# The Diff-WMPC code (diff_weight_mpc: MPC, loss, policy, closed loop) lives in experiments/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "experiments"))

SIM_YAML = "simTimeStep: 0.01\n"

# F1TENTH-scale values, as in scenarios/*/params/agents.yaml.
WHEELBASE = 0.34

_COMMON_PLUGINS = {
    "geometry_plugin": "RectangularGeometry",
    "geometry_params": {"length": 0.58, "width": 0.31},
    "collision_plugin": "EllipseCollisionFootPrint",
    "collision_params": {"major_axis_length": 0.60, "minor_axis_length": 0.34},
    "sensor_plugin": "LidarSensorModel",
    "sensor_params": {
        "num_beams": 36,
        "angle_min": -3.14159265,
        "angle_max": 3.14159265,
        "range_min": 0.05,
        "range_max": 10.0,
    },
}

_INIT = {
    "initXPose": 1.0,
    "initYPose": -2.0,
    "initYaw": 0.5,
    "initVx": 0.0,
    "initSf": 0.0,
    "initAcc": 0.0,
    "initSv": 0.0,
}


def single_track_agent(**overrides):
    params = {
        "numStates": 7,
        "numControlInputs": 2,
        **_INIT,
        "initYawRate": 0.0,
        "initVy": 0.0,
        "mass": 3.47,
        "Iz": 0.04712,
        "lf": 0.17,
        "lr": 0.17,
        "cf": 4.718,
        "cr": 5.4562,
        "vehWheelBase": WHEELBASE,
        "steeringTimeConstant": 0.1,
        "integration": {"integrationStepSize": 0.001},
    }
    params.update(overrides)
    return {"dynamics_plugin": "SingleTrackDynStateModel", "dynamics_params": params, **_COMMON_PLUGINS}


def aug_agent(**overrides):
    """SingleTrackAugDynModel with the same vehicle parameters as single_track_agent()."""
    agent = single_track_agent(**{"numStates": 8, **overrides})
    agent["dynamics_plugin"] = "SingleTrackAugDynModel"
    return agent


def kinematic_agent(**overrides):
    params = {
        "numStates": 5,
        "numControlInputs": 2,
        **_INIT,
        "vehWheelBase": WHEELBASE,
        "steeringTimeConstant": 0.1,
        "integration": {"integrationStepSize": 0.001},
    }
    params.update(overrides)
    return {"dynamics_plugin": "BicycleKinematicModel", "dynamics_params": params, **_COMMON_PLUGINS}


def dump(agent):
    return yaml.safe_dump(agent)


@pytest.fixture
def st_sim():
    from plannertrack_py import Sim

    return Sim(SIM_YAML, dump(single_track_agent()))


@pytest.fixture
def aug_sim():
    from plannertrack_py import Sim

    return Sim(SIM_YAML, dump(aug_agent()))


@pytest.fixture
def kin_sim():
    from plannertrack_py import Sim

    return Sim(SIM_YAML, dump(kinematic_agent()))
