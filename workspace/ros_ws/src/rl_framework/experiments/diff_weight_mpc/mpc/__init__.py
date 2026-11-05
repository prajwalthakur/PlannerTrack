# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""MPC for the Diff-WMPC experiments (steps.md Steps 2 and 4).

- :mod:`plannertrack_py.vehicle`: vehicle parameters shared by the plant and the MPC.
- :mod:`diff_weight_mpc.mpc.model`: the prediction model in CasADi (imports casadi; import it
  explicitly, it is not loaded here).
"""

from plannertrack_py.vehicle import (
    SINGLE_TRACK_PLUGINS,
    VehicleParams,
    agent_with_vehicle_params,
    load_vehicle_params,
    vehicle_params_from_agent,
)

__all__ = [
    "SINGLE_TRACK_PLUGINS",
    "VehicleParams",
    "agent_with_vehicle_params",
    "load_vehicle_params",
    "vehicle_params_from_agent",
]
