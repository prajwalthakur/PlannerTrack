# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Reference queries, validation and plot on the racing-scenario raceline."""

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from plannertrack_py import (
    ReferenceTrack,
    compute_raceline,
    load_raceline_limits,
    plot_reference,
    validate_reference,
)

SRC = Path(__file__).resolve().parents[3]
PARAMS = SRC / "scenarios" / "ground_vehicle_racing_rl" / "params"
AGENTS = PARAMS / "agents.yaml"
PLANNER = PARAMS / "min_curv_planner.yaml"


@pytest.fixture(scope="module")
def raceline():
    return compute_raceline(AGENTS, "agent_1", PLANNER)


@pytest.fixture(scope="module")
def ref(raceline):
    return ReferenceTrack(raceline)


@pytest.fixture(scope="module")
def limits():
    return load_raceline_limits(AGENTS, "agent_1", PLANNER)


def test_limits_come_from_the_cpp_params(limits):
    # agents.yaml: max_steer 0.6981 (40 deg), vehWheelBase 0.34, mu 1.0489; planner: margin 0.82
    assert limits.curvature_limit == pytest.approx(math.tan(0.6981) / 0.34)
    assert limits.ay_max == pytest.approx(0.82 * 1.0489 * 9.81)
    assert limits.optimization_width == pytest.approx(0.31 + 2 * 0.10)


# ------------------------------------------------------------------ queries


def test_sample_reproduces_the_raceline_points(ref, raceline):
    v = ref.sample(raceline.s)
    np.testing.assert_allclose(v["x"], raceline.x, atol=1e-12)
    np.testing.assert_allclose(v["kappa"], raceline.kappa, atol=1e-12)
    np.testing.assert_allclose(np.angle(np.exp(1j * (v["psi"] - raceline.psi))), 0.0, atol=1e-12)


def test_sample_is_continuous_across_start_finish(ref):
    eps = 1e-6
    a, b = ref.sample([ref.length - eps]), ref.sample([ref.length + eps])
    assert abs(a["x"][0] - b["x"][0]) < 1e-4 and abs(a["y"][0] - b["y"][0]) < 1e-4
    assert abs(a["psi"][0] - b["psi"][0]) < 1e-4  # continuous heading, no 2 pi jump
    assert abs(ref.total_turn) == pytest.approx(2 * math.pi, abs=1e-6)  # one loop


def test_project_points_on_the_line(ref, raceline):
    for k in range(0, len(raceline), 7):
        p = ref.project(raceline.x[k], raceline.y[k])
        assert p.s == pytest.approx(raceline.s[k], abs=1e-9)
        assert abs(p.n) < 1e-9


def test_project_lateral_offset_sign_left_positive(ref, raceline):
    k = 40
    left = np.array([-math.sin(raceline.psi[k]), math.cos(raceline.psi[k])])
    for offset in (0.15, -0.15):
        x, y = raceline.xy[k] + offset * left
        p = ref.project(x, y)
        assert p.n == pytest.approx(offset, abs=2e-3)  # chord vs tangent at the knot
        assert p.s == pytest.approx(raceline.s[k], abs=2e-2)


def test_project_window_restricts_the_search(ref, raceline):
    k = 60
    p = ref.project(raceline.x[k], raceline.y[k], s_guess=raceline.s[k] + 0.3, window=1.0)
    assert p.s == pytest.approx(raceline.s[k], abs=1e-9)
    with pytest.raises(ValueError):
        ref.project(raceline.x[k], raceline.y[k], s_guess=raceline.s[k], window=-1.0)


def test_project_across_start_finish_with_window(ref, raceline):
    # point at the start, expected position just before the end of the lap
    p = ref.project(raceline.x[0], raceline.y[0], s_guess=ref.length - 0.1, window=0.5)
    assert min(p.s, ref.length - p.s) < 1e-6


def test_stage_references_follow_arc_length(ref):
    s0, n, dt = ref.length - 1.0, 25, 0.1  # crosses start/finish
    st = ref.stage_references(s0, n, dt)
    assert st["s"].shape == (n + 1,) and st["s"][0] == s0
    assert np.all(np.diff(st["s"]) > 0)  # not wrapped
    np.testing.assert_allclose(np.diff(st["s"]), st["vx"][:-1] * dt, rtol=1e-12)
    assert np.all(np.abs(np.diff(st["psi"])) < 0.5)  # continuous heading
    np.testing.assert_allclose(st["t"], dt * np.arange(n + 1))


def test_policy_lookahead_shape_and_horizon(ref):
    v, k = ref.policy_lookahead(0.0)
    assert v.shape == (5,) and k.shape == (5,)
    # the last sample is ~2.55 s ahead: the distance covered matches the speed profile
    st = ref.stage_references(0.0, 255, 0.01)
    assert v[-1] == pytest.approx(float(ref.sample(st["s"][-1])["vx"]), abs=1e-6)


def test_from_npz_round_trip(raceline, tmp_path):
    path = tmp_path / "ref.npz"
    raceline.save_npz(path)
    ref2 = ReferenceTrack.from_npz(path)
    assert ref2.length == raceline.length and len(ref2) == len(raceline)
    np.testing.assert_array_equal(ref2.raceline.vx, raceline.vx)


# --------------------------------------------------------------- validation


def test_scenario_reference_passes_validation(raceline, limits):
    report = validate_reference(raceline, limits)
    print(report.format())
    assert report.passed, report.format()


def test_validation_catches_too_much_speed(raceline, limits):
    fast = dataclasses.replace(raceline, vx=raceline.vx * 1.2)  # a_lat and a_x too high
    report = validate_reference(fast, limits)
    assert not report.passed
    failed = {c.name for c in report.checks if not c.passed}
    assert any("a_lat" in name for name in failed)


def test_validation_catches_a_speed_jump(raceline, limits):
    # the start/finish bug of tph: no braking before the first corner
    vx = raceline.vx.copy()
    vx[-10:] = raceline.vx.max()
    report = validate_reference(dataclasses.replace(raceline, vx=vx, ax=np.append(np.diff(vx**2) / (2 * np.diff(raceline.s)), -50.0)), limits)
    assert not report.passed


def test_plot_writes_png(raceline, limits, tmp_path):
    path = tmp_path / "ref.png"
    plot_reference(raceline, path, lim=limits)
    assert path.stat().st_size > 10_000
