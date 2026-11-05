# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Raceline bindings against the racing scenario and the tph fixture of min_curv_planner."""

from pathlib import Path

import numpy as np
import pytest

from plannertrack_py import Raceline, compute_raceline, load_centerline, optimize_raceline
from plannertrack_py.raceline import centerline_from_config, load_planner_config

SRC = Path(__file__).resolve().parents[3]
PARAMS = SRC / "scenarios" / "ground_vehicle_racing_rl" / "params"
AGENTS = PARAMS / "agents.yaml"
PLANNER = PARAMS / "min_curv_planner.yaml"
JSON = SRC / "scenarios" / "ground_vehicle_racing_rl" / "map" / "global_waypoints.json"
TPH_DATA = SRC / "planning" / "min_curv_planner" / "test" / "data"


@pytest.fixture(scope="module")
def raceline():
    return compute_raceline(AGENTS, "agent_1", PLANNER)


def test_load_centerline_shape():
    c = load_centerline(JSON)
    assert c.ndim == 2 and c.shape[1] == 4 and c.shape[0] > 100
    assert np.all(c[:, 2:] > 0.0)


def test_config_points_to_the_scenario_centerline():
    assert load_planner_config(PLANNER)["centerline_key"] == "centerline_waypoints"
    np.testing.assert_array_equal(centerline_from_config(PLANNER), load_centerline(JSON))


def test_raceline_is_consistent(raceline):
    assert isinstance(raceline, Raceline)
    n = len(raceline)
    assert n > 100
    for name in ("s", "x", "y", "psi", "kappa", "vx", "ax", "w_right", "w_left"):
        assert getattr(raceline, name).shape == (n,), name
    assert raceline.iqp_converged
    assert np.all(np.diff(raceline.s) > 0.0)
    assert raceline.s[-1] < raceline.length
    assert raceline.xy.shape == (n, 2)
    np.testing.assert_allclose(raceline.a_lat, raceline.vx**2 * raceline.kappa)


def test_raceline_respects_limits(raceline):
    from plannertrack_py import load_raceline_limits

    lim = load_raceline_limits(AGENTS, "agent_1", PLANNER)
    assert lim.ay_max == pytest.approx(0.82 * 1.0489 * 9.81)  # friction_margin * mu * g
    assert raceline.vx.max() <= lim.v_max + 1e-9
    assert np.abs(raceline.a_lat).max() <= lim.ay_max * (1.0 + 1e-6)
    half_width = 0.5 * (0.31 + 2 * 0.10)
    assert raceline.w_right.min() >= half_width - 0.05
    assert raceline.w_left.min() >= half_width - 0.05


def test_matches_cpp_pipeline_on_scenario(raceline):
    # Same numbers as the C++ test ComputeRaceline.ScenarioTrackEndToEnd / the ROS plugin.
    assert len(raceline) == 137
    assert raceline.length == pytest.approx(27.3763, abs=1e-3)
    assert raceline.lap_time == pytest.approx(5.992, abs=1e-3)  # RL scenario: friction_margin 0.82, max_decel 8.64
    assert raceline.iqp_iterations == 5


def test_optimize_matches_tph_fixture_geometry():
    # tph reference line through Python -> same raceline geometry as tph (corrected
    # objective). Speeds are compared in the C++ test TphParity.PipelineMatchesTphRaceline
    # with the fixture's own acceleration limits, which differ from the RL scenario's.
    reference = np.loadtxt(TPH_DATA / "tph_reftrack.csv", delimiter=",")
    expected = np.loadtxt(TPH_DATA / "tph_raceline.csv", delimiter=",")
    rl = optimize_raceline(reference, AGENTS, "agent_1", PLANNER)
    assert len(rl) == len(expected)
    np.testing.assert_allclose(rl.xy, expected[:, 1:3], atol=1e-6)
    np.testing.assert_allclose(rl.kappa, expected[:, 4], atol=1e-6)


def test_save_npz(raceline, tmp_path):
    path = tmp_path / "reference_racing.npz"
    raceline.save_npz(path)
    data = np.load(path)
    np.testing.assert_array_equal(data["kappa"], raceline.kappa)
    assert float(data["lap_time"]) == raceline.lap_time
    assert "a_lat" in data


def test_bad_centerline_shape_raises():
    with pytest.raises(ValueError):
        compute_raceline(AGENTS, "agent_1", PLANNER, centerline=np.zeros((10, 3)))
