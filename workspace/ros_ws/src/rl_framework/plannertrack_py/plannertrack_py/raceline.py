# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Minimum-curvature raceline (``planning/min_curv_planner``) from Python.

Same C++ code and the same scenario files as the ROS plugin ``MinCurvPlanner``,
so the reference used for training is the one the controllers follow in the
simulator. Derivations: ``project_docs/learning_based_controls/min_curv_*.md``.

Example::

    from plannertrack_py.raceline import compute_raceline
    params = "src/scenarios/ground_vehicle_racing_rl/params"
    rl = compute_raceline(f"{params}/agents.yaml", "agent_1", f"{params}/min_curv_planner.yaml")
    rl.s, rl.x, rl.y, rl.psi, rl.kappa, rl.vx, rl.ax, rl.a_lat
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from plannertrack_py import _core
from plannertrack_py.config import load_agent_yaml

PLANNER_BLOCK = "min_curv_planner"


@dataclass(frozen=True)
class Raceline:
    """Closed raceline sampled along arc length (closing point not repeated).

    Heading ``psi`` is 0 along +x; ``kappa`` > 0 is a left turn; widths are the
    distances from the raceline to the right/left track bound.
    """

    s: np.ndarray  # arc length from the start point [m]
    x: np.ndarray  # [m]
    y: np.ndarray  # [m]
    psi: np.ndarray  # heading [rad]
    kappa: np.ndarray  # curvature [rad/m]
    vx: np.ndarray  # speed [m/s]
    ax: np.ndarray  # longitudinal acceleration [m/s^2]
    w_right: np.ndarray  # distance to right bound [m]
    w_left: np.ndarray  # distance to left bound [m]
    length: float  # closed-loop length [m]
    lap_time: float  # estimated lap time [s]
    iqp_iterations: int
    iqp_converged: bool
    iqp_curv_error_max: float  # linearization error of the last QP [rad/m]

    def __len__(self):
        return len(self.s)

    @property
    def xy(self):
        """(N, 2) positions."""
        return np.column_stack((self.x, self.y))

    @property
    def a_lat(self):
        """Lateral acceleration of the reference, vx^2 * kappa [m/s^2]."""
        return self.vx**2 * self.kappa

    def save_npz(self, path):
        """Write all arrays and scalars to a .npz file (steps.md §3.5 reference file)."""
        np.savez(path, **{k: getattr(self, k) for k in self.__dataclass_fields__}, a_lat=self.a_lat)


def load_planner_config(planner_yaml_path):
    """The ``min_curv_planner`` block of a planner config (ros__parameters wrapper optional)."""
    root = yaml.safe_load(Path(planner_yaml_path).read_text())
    node = root.get("/**", {}).get("ros__parameters", root) if isinstance(root, dict) else {}
    if PLANNER_BLOCK not in node:
        raise KeyError(f"no '{PLANNER_BLOCK}' block in {planner_yaml_path}")
    return node[PLANNER_BLOCK]


def load_centerline(json_path, key="centerline_waypoints"):
    """(N, 4) array [x, y, w_right, w_left] from a global_waypoints.json."""
    return _core.load_track_json(str(json_path), key)


def centerline_from_config(planner_yaml_path):
    """Load the centerline the planner config points to (path relative to the config, as in the plugin)."""
    cfg = load_planner_config(planner_yaml_path)
    path = Path(cfg["centerline_file"])
    if not path.is_absolute():
        path = Path(planner_yaml_path).parent / path
    return load_centerline(path, cfg.get("centerline_key", "centerline_waypoints"))


def compute_raceline(agents_yaml_path, agent_name, planner_yaml_path, centerline=None):
    """Raceline and speed profile, exactly as ``MinCurvPlanner`` computes it.

    Vehicle parameters come from ``agent_name`` in ``agents_yaml_path``, tuning from the
    ``min_curv_planner`` block of ``planner_yaml_path``. The centerline is the JSON file
    the planner config points to, unless an (N, 4) ``centerline`` array is given.
    """
    if centerline is None:
        centerline = centerline_from_config(planner_yaml_path)
    result = _core.compute_raceline(
        np.asarray(centerline, dtype=float),
        load_agent_yaml(agents_yaml_path, agent_name),
        yaml.safe_dump(load_planner_config(planner_yaml_path)),
    )
    return Raceline(**result)


def optimize_raceline(reference, agents_yaml_path, agent_name, planner_yaml_path):
    """Like :func:`compute_raceline` on an already prepared (N, 4) reference line (no preprocessing)."""
    result = _core.optimize_raceline(
        np.asarray(reference, dtype=float),
        load_agent_yaml(agents_yaml_path, agent_name),
        yaml.safe_dump(load_planner_config(planner_yaml_path)),
    )
    return Raceline(**result)
