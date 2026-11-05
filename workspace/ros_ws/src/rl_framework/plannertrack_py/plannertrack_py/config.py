# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Build a :class:`Sim` from the same YAML files ``agent_sim`` uses."""

from pathlib import Path

import yaml

from plannertrack_py._core import Sim

# State/control layouts per dynamics plugin (StateToVector / VectorToInput in
# motion_model_ground_vehicles). Keep in sync if a plugin's layout changes.
STATE_NAMES = {
    "SingleTrackAugDynModel": ("x", "y", "yaw", "vx", "vy", "yaw_rate", "steer", "acc"),
    "SingleTrackDynStateModel": ("x", "y", "yaw", "vx", "vy", "yaw_rate", "steer"),
    "BicycleKinematicModel": ("x", "y", "yaw", "vx", "steer"),
}
CONTROL_NAMES = {
    "SingleTrackAugDynModel": ("jerk", "steer_rate"),
    "SingleTrackDynStateModel": ("acc", "steer_rate"),
    "BicycleKinematicModel": ("acc", "steer_rate"),
}


# Plant used for learning: the single-track model with the
# acceleration as a state, driven by jerk.
DEFAULT_DYNAMICS_PLUGIN = "SingleTrackAugDynModel"

# Plugins that share the single-track parameters and can be swapped for each other.
_SINGLE_TRACK_STATES = {"SingleTrackDynStateModel": 7, "SingleTrackAugDynModel": 8}


def with_dynamics_plugin(agent, plugin):
    """Copy of an agent node (dict) using ``plugin`` for its dynamics.

    Only swaps between the single-track plugins, which take the same vehicle parameters;
    ``numStates`` is set for the target. For SingleTrackAugDynModel ``initAcc`` becomes the
    initial acceleration state and the initial jerk is ``initJerk`` (default 0).
    """
    current = agent["dynamics_plugin"]
    if current == plugin:
        return agent
    if current not in _SINGLE_TRACK_STATES or plugin not in _SINGLE_TRACK_STATES:
        raise ValueError(f"cannot convert dynamics plugin {current!r} to {plugin!r}")
    out = dict(agent)
    out["dynamics_plugin"] = plugin
    out["dynamics_params"] = dict(agent["dynamics_params"], numStates=_SINGLE_TRACK_STATES[plugin])
    return out


def load_agent_yaml(agents_yaml_path, agent_name, dynamics_plugin=None):
    """Return one agent's node from an ``agents.yaml`` as YAML text.

    Pass the result to :class:`Sim` as ``agent_yaml``. With ``dynamics_plugin`` the
    agent's single-track plugin is swapped (see :func:`with_dynamics_plugin`).
    """
    agents = yaml.safe_load(Path(agents_yaml_path).read_text())["agents"]
    if agent_name not in agents:
        raise KeyError(f"{agent_name!r} not in {agents_yaml_path} (have: {list(agents)})")
    agent = agents[agent_name]
    if dynamics_plugin is not None:
        agent = with_dynamics_plugin(agent, dynamics_plugin)
    return yaml.safe_dump(agent)


def make_sim(sim_yaml_path, agents_yaml_path, agent_name, dynamics_plugin=DEFAULT_DYNAMICS_PLUGIN):
    """Build a :class:`Sim` for ``agent_name`` from scenario ``sim.yaml``/``agents.yaml`` files.

    By default the plant is ``SingleTrackAugDynModel`` (state [..., steer, acc], input
    [jerk, steer_rate]), built from the agent's single-track parameters even if the scenario
    runs ``SingleTrackDynStateModel`` for its ROS controllers. ``dynamics_plugin=None``
    keeps the plugin named in ``agents.yaml``.
    """
    return Sim(Path(sim_yaml_path).read_text(),
               load_agent_yaml(agents_yaml_path, agent_name, dynamics_plugin))
