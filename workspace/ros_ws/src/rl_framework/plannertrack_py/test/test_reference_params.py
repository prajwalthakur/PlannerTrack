# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Stage parameters for the OCP (steps.md 4.2) and the MPC-only track tightening (4.9)."""

from pathlib import Path

import numpy as np
import pytest

from plannertrack_py import ReferenceTrack, compute_raceline
from diff_weight_mpc.mpc.ocp import STAGE_REF_NAMES, OcpConfig
from diff_weight_mpc.mpc.reference_params import align_yaw, stage_arc_lengths, stage_parameters
from plannertrack_py.utils import angle_diff, wrap_angle
from plannertrack_py.plant import NX, PSI, Plant

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
CFG = OcpConfig()


@pytest.fixture(scope="module")
def ref():
    return ReferenceTrack(compute_raceline(PARAMS / "agents.yaml", "agent_1", PARAMS / "min_curv_planner.yaml"))


@pytest.fixture(scope="module")
def plant(ref):
    return Plant.from_scenario(PARAMS, reference=ref)


def states_on_reference(ref, s0):
    x = np.zeros((len(s0), NX))
    r = ref.sample(np.asarray(s0))
    x[:, 0], x[:, 1], x[:, PSI], x[:, 3] = r["x"], r["y"], wrap_angle(r["psi"]), r["vx"]
    return x


def test_shapes_and_single_car_matches_stage_references(ref, plant):
    s0 = np.array([0.0, 12.0, ref.length - 0.5])
    x0, p = stage_parameters(ref, states_on_reference(ref, s0), s0, CFG.N, CFG.dt, plant.limits.vehicle_width)
    assert x0.shape == (3, NX) and set(p) == set(STAGE_REF_NAMES)
    assert all(v.shape == (3, CFG.N + 1, 1) for v in p.values())
    for i, s in enumerate(s0):
        st = ref.stage_references(s, CFG.N, CFG.dt)
        np.testing.assert_allclose(stage_arc_lengths(ref, [s], CFG.N, CFG.dt)[0], st["s"], atol=1e-12)
        np.testing.assert_allclose(p["x_ref"][i, :, 0], st["x"], atol=1e-12)
        np.testing.assert_allclose(p["v_ref"][i, :, 0], st["vx"], atol=1e-12)
        np.testing.assert_allclose(angle_diff(p["psi_ref"][i, :, 0], st["psi"]), 0.0, atol=1e-12)


def test_limits_without_margin_are_the_plants(ref, plant):
    s0 = np.array([0.0, 7.0, 19.0])
    _, p = stage_parameters(ref, states_on_reference(ref, s0), s0, CFG.N, CFG.dt, plant.limits.vehicle_width)
    S = stage_arc_lengths(ref, s0, CFG.N, CFG.dt)
    for i in range(len(s0)):
        lim = np.array([plant.track_limits(s) for s in S[i]])
        np.testing.assert_allclose(p["n_left"][i, :, 0], lim[:, 0], atol=1e-12)
        np.testing.assert_allclose(p["n_right"][i, :, 0], lim[:, 1], atol=1e-12)


def test_track_margin_tightens_but_keeps_the_reference_inside(ref, plant):
    """Each side moves inward by the margin, but never past the reference line (n = 0)."""
    s0 = np.linspace(0.0, ref.length, 16, endpoint=False)
    x = states_on_reference(ref, s0)
    w = plant.limits.vehicle_width
    _, p0 = stage_parameters(ref, x, s0, CFG.N, CFG.dt, w)
    _, p3 = stage_parameters(ref, x, s0, CFG.N, CFG.dt, w, track_margin=0.03)
    np.testing.assert_allclose(p3["n_left"], np.maximum(p0["n_left"] - 0.03, 0.0))
    np.testing.assert_allclose(p3["n_right"], np.minimum(p0["n_right"] + 0.03, 0.0))
    assert np.all(p3["n_left"] >= 0.0) and np.all(p3["n_right"] <= 0.0)
    # at the tight apex (s ~ 19 m) the car-side limit is 0.102 m: tightened to 0.072 m
    _, pa = stage_parameters(ref, states_on_reference(ref, [19.0]), [19.0], CFG.N, CFG.dt, w, track_margin=0.03)
    assert pa["n_left"][0, 0, 0] == pytest.approx(plant.track_limits(19.0)[0] - 0.03)
    # a margin wider than the space to the line clamps at the line
    _, pw = stage_parameters(ref, x, s0, CFG.N, CFG.dt, w, track_margin=5.0)
    assert np.all(pw["n_left"] == 0.0) and np.all(pw["n_right"] == 0.0)


def test_align_yaw_follows_x0_over_the_whole_row(ref, plant):
    """The plant's wrapped yaw (any branch) and a row crossing the start line: the row starts within pi of
    x0's yaw and is continuous, the directions unchanged."""
    s0 = np.array([ref.length - 0.5])
    x = states_on_reference(ref, s0)
    x[0, PSI] += 2 * np.pi  # another branch for the car
    x0, p = stage_parameters(ref, x, s0, CFG.N, CFG.dt, plant.limits.vehicle_width)
    row = p["psi_ref"][0, :, 0]
    assert abs(row[0] - x0[0, PSI]) < 1e-9  # the car is exactly on the reference here
    assert np.all(np.abs(np.diff(row)) < 0.5)
    st = ref.stage_references(s0[0], CFG.N, CFG.dt)
    np.testing.assert_allclose(angle_diff(row, st["psi"]), 0.0, atol=1e-12)
    np.testing.assert_array_equal(x0, x)  # x0 itself is not changed


def test_align_yaw_repairs_wrapped_rows():
    psi = np.array([[3.10, 3.13, -3.12, -3.09, -3.05]])  # a wrapped row crossing +-pi
    out = align_yaw(np.array([3.08]), psi)
    np.testing.assert_allclose(np.diff(out[0]), [0.03, 2 * np.pi - 6.25, 0.03, 0.04], atol=1e-12)
    np.testing.assert_allclose(angle_diff(out[0], psi[0]), 0.0, atol=1e-12)


def test_controller_uses_the_config_margin():
    assert OcpConfig().track_margin == pytest.approx(0.03)
