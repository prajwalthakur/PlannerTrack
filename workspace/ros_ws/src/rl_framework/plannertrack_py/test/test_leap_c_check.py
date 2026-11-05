# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Regression tests for the Step 5 sensitivity check (experiments/diff_weight_mpc/utils/leap_c_check.py, steps.md §7).

Keeps what Step 5 established: leap-c's derivatives of the MPC solution with respect to the weights agree with
finite differences, also with the track and friction-ellipse constraints active; the training path (backward())
equals the chain rule on them; batched solves give the same derivatives; and leap-c's "dx_dp_global" is the SUM
over stages (if a leap-c update changes that, leap_c_check._stage_sensitivities can be simplified).
Compiles one SQP solver (~40 s).
"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("leap_c.torch")
pytest.importorskip("torch")

SCRIPT = Path(__file__).resolve().parents[2] / "experiments" / "diff_weight_mpc" / "utils" / "leap_c_check.py"
EPSILON = 1e-4  # inside the noise-free range of every case except the hairpin's kink (results/leap_c_check.md)


@pytest.fixture(scope="module")
def lcc():
    spec = importlib.util.spec_from_file_location("leap_c_check", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def cfg(lcc):
    return lcc.CheckConfig()


@pytest.fixture(scope="module")
def plant(lcc):
    ref = lcc.ReferenceTrack(lcc.compute_raceline(lcc.PARAMS / "agents.yaml", "agent_1",
                                                  lcc.PARAMS / "min_curv_planner.yaml"))
    return lcc.Plant.from_scenario(lcc.PARAMS, reference=ref, dt=0.02)


@pytest.fixture(scope="module")
def sqp(lcc, plant, cfg, tmp_path_factory):
    return lcc.build_diff_mpc(plant, cfg, "SQP", export_directory=tmp_path_factory.mktemp("acados_check"),
                              name="test_check_sqp")


@pytest.fixture(scope="module")
def cases(lcc, plant, cfg):
    return {case.name: case for case in lcc.make_test_cases(plant, cfg)}


def test_constrained_cases_have_active_constraints(lcc, plant, sqp, cases, cfg):
    """The check covers active constraints: the ellipse case uses the friction ellipse, off_line the track."""
    theta = np.array(cfg.theta)
    ellipse = lcc.active_set(plant, cases["ellipse"], lcc.solve(sqp, cases["ellipse"], theta))
    off_line = lcc.active_set(plant, cases["off_line"], lcc.solve(sqp, cases["off_line"], theta))
    assert "ellipse" in ellipse["active"]
    assert "track_left" in off_line["active"]


@pytest.mark.parametrize("name", ["straight", "ellipse", "off_line"])
def test_leapc_matches_finite_differences(lcc, sqp, cases, cfg, name):
    theta = np.array(cfg.theta)
    loss = lcc.make_synthetic_loss(cfg.seed)
    leapc = lcc.leapc_sensitivities(sqp, cases[name], theta, loss)
    fd = lcc.finite_difference_sensitivities(sqp, cases[name], theta, EPSILON, loss)
    assert not fd.failed
    result = lcc.compare(leapc, fd)
    for output in ("x1", "u1", "u0", "value", "loss"):
        assert result[output]["rel_error"] < 1e-4, (output, result[output])
        assert result[output]["cosine"] > 1 - 1e-6, (output, result[output])


def test_backward_equals_chain_rule(lcc, sqp, cases, cfg):
    """The training path: autograd through leap-c's backward() = cᵀ·∂x₁/∂θ + dᵀ·∂u₁/∂θ."""
    c, d = lcc.make_synthetic_loss(cfg.seed)
    sens = lcc.leapc_sensitivities(sqp, cases["off_line"], np.array(cfg.theta), (c, d))
    np.testing.assert_allclose(sens.dloss, c @ sens.dx1 + d @ sens.du1, rtol=1e-10, atol=1e-12)


def test_leapc_dx_dp_global_is_sum_over_stages(lcc, sqp, cases, cfg):
    """leap-c's sensitivity(ctx, "dx_dp_global") seeds every stage at once: it returns Σ_k ∂x_k/∂θ, not the
    per-stage Jacobian (which leap_c_check computes stage by stage)."""
    theta = np.array(cfg.theta)
    ctx = lcc._forward(sqp, cases["straight"], theta)[0]
    fun = sqp.diff_mpc_fun
    per_stage = lcc._jacobians(fun, ctx, theta.size)[0].dx
    leapc = fun.sensitivity(ctx, "dx_dp_global")[0]
    assert leapc.shape == (lcc.NX, theta.size)
    np.testing.assert_allclose(per_stage.sum(axis=0), leapc, rtol=1e-9, atol=1e-9)
    assert np.abs(per_stage[0]).max() == 0.0  # x₀ is the measured state: fixed


def test_batch_equals_single(lcc, sqp, cases, cfg):
    result = lcc.batch_vs_single(sqp, [cases["straight"], cases["ellipse"]], np.array(cfg.theta))
    for name, r in result.items():
        assert r["status"] == 0
        for output in ("x1", "u1", "u0", "value"):
            assert r["compare"][output]["rel_error"] < 1e-9, (name, output, r["compare"][output])
