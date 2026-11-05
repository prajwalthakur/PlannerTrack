# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Vehicle parameters of the single-track plant: one source for the plant and the MPC.

Reads exactly the keys ``SingleTrackDynStateModel`` / ``SingleTrackAugDynModel`` read from an
agent's ``dynamics_params`` in ``agents.yaml``, with the same defaults, so the MPC model can never
silently use different numbers than the simulator. The reverse direction,
:func:`agent_with_vehicle_params`, builds an agent node (for :class:`plannertrack_py.Sim`) from a
:class:`VehicleParams`, e.g. with perturbed mass or mu for the mismatch experiments.
"""

import copy
import dataclasses
from dataclasses import dataclass
from pathlib import Path

import yaml

SINGLE_TRACK_PLUGINS = ("SingleTrackAugDynModel", "SingleTrackDynStateModel")

# dynamics_params key -> VehicleParams field
_KEYS = {"mass": "mass", "Iz": "Iz", "lf": "lf", "lr": "lr", "cf": "cf", "cr": "cr",
         "mu": "mu", "hcg": "hcg"}
# Optional keys and their defaults in the C++ plugins (initialze()).
_DEFAULTS = {"mu": 1.0, "hcg": 0.0}


@dataclass(frozen=True)
class VehicleParams:
    """Physical parameters of the single-track model (C++ names in brackets)."""

    mass: float  # [mass] kg
    Iz: float  # [Iz] yaw inertia, kg m^2
    lf: float  # [lf] CG to front axle, m
    lr: float  # [lr] CG to rear axle, m
    cf: float  # [cf] load-normalized front cornering stiffness C_Sf, 1/rad
    cr: float  # [cr] load-normalized rear cornering stiffness C_Sr, 1/rad
    mu: float = 1.0  # [mu] tire-road friction coefficient
    hcg: float = 0.0  # [hcg] CG height (longitudinal load transfer), m

    @property
    def wheelbase(self):
        """lf + lr: the wheelbase the plant's dynamics use [m]."""
        return self.lf + self.lr

    def with_overrides(self, **changes):
        """Copy with some fields replaced, e.g. ``params.with_overrides(mass=4.0, mu=0.9)``."""
        return dataclasses.replace(self, **changes)


def vehicle_params_from_agent(agent):
    """:class:`VehicleParams` from one agent's node of ``agents.yaml`` (a dict).

    Raises ValueError for non-single-track plugins, missing keys, or if ``vehWheelBase``
    disagrees with ``lf + lr`` (the plant uses lf + lr, the raceline's curvature limit uses
    vehWheelBase).
    """
    plugin = agent.get("dynamics_plugin")
    if plugin not in SINGLE_TRACK_PLUGINS:
        raise ValueError(f"not a single-track plugin: {plugin!r} (expected one of {SINGLE_TRACK_PLUGINS})")
    dyn = agent["dynamics_params"]
    values = {}
    for key, field in _KEYS.items():
        if key in dyn:
            values[field] = float(dyn[key])
        elif key in _DEFAULTS:
            values[field] = _DEFAULTS[key]
        else:
            raise ValueError(f"dynamics_params has no '{key}'")
    params = VehicleParams(**values)
    if "vehWheelBase" in dyn and abs(float(dyn["vehWheelBase"]) - params.wheelbase) > 1e-9:
        raise ValueError(f"vehWheelBase {dyn['vehWheelBase']} != lf + lr = {params.wheelbase}")
    return params


def load_vehicle_params(agents_yaml_path, agent_name):
    """:class:`VehicleParams` of ``agent_name`` in an ``agents.yaml`` file."""
    agents = yaml.safe_load(Path(agents_yaml_path).read_text())["agents"]
    if agent_name not in agents:
        raise KeyError(f"{agent_name!r} not in {agents_yaml_path} (have: {list(agents)})")
    return vehicle_params_from_agent(agents[agent_name])


def agent_with_vehicle_params(agent, params):
    """Copy of an agent node (dict) whose dynamics_params carry ``params``.

    Use it to build a plant with perturbed parameters::

        nominal = vehicle_params_from_agent(agent)
        heavy = agent_with_vehicle_params(agent, nominal.with_overrides(mass=1.2 * nominal.mass))
        sim = Sim(sim_yaml, yaml.safe_dump(heavy))
    """
    vehicle_params_from_agent(agent)  # validates the plugin and keys
    out = copy.deepcopy(agent)
    for key, field in _KEYS.items():
        out["dynamics_params"][key] = float(getattr(params, field))
    return out
