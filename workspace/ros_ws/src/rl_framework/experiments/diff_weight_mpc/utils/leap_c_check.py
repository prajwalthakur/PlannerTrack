#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Step 5: are leap-c's derivatives of the MPC solution with respect to the weights correct?

Plan and status: project_docs/learning_based_controls/steps.md §7. Every sub-step of Step 5 lives in this script,
one function each (5.0 … 5.7); they are filled in one by one.

What is checked: the derivatives of the plan at node 1 (x₁, u₁; the paper takes gradients at node 1), of the
applied input u₀ and of the optimal cost V with respect to the 6 weights θ = (q_n, q_ψ, q_v, q_alat, r_j, r_ω),

    leap-c   per-stage acados adjoints for x₁, u₁ (leap-c's "dx_dp_global"/"du_dp_global" are sums over stages),
             diff_mpc_fun.sensitivity(ctx, "du0_dp_global" | "dvalue_dp_global"), and backward() through a synthetic loss L = cᵀx₁ + dᵀu₁ (fixed random c, d: independent of the
             paper's α…δ, which are not scaled yet)
    vs.      central finite differences: re-solve with θ_i·(1 ± ε), converged to tight tolerance (SQP)

at test states chosen to include active constraints (track bound, friction ellipse), plus the controller's real
setting (SQP_RTI, one warm-started iteration per 0.02 s) and batched solves.

Also measured (5.4, for the Step 6 loss design): how strongly θ moves the lateral error n_k compared with the speed
v_k at nodes k = 1, 2, 5, …, and how much of the paper's loss gradient comes from each term.

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/leap_c_check.py                 # all sub-steps
    python3 src/rl_framework/experiments/diff_weight_mpc/utils/leap_c_check.py --only fd       # 5.1–5.4 only
"""

import argparse
import json
from dataclasses import dataclass, field, replace
import sys
from pathlib import Path

import casadi as ca
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch
from leap_c.diff_mpc.utils.prepare_solver import prepare_batch_solver_for_backward
from leap_c.torch import AcadosDiffMpcTorch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from plannertrack_py import ReferenceTrack, compute_raceline  # noqa: F401
from diff_weight_mpc.learning.loss import LossConfig  # noqa: F401
from diff_weight_mpc.mpc.initializer import ReferenceInitializer  # noqa: F401
from diff_weight_mpc.mpc.model import NU, NX
from diff_weight_mpc.mpc.ocp import (  # noqa: F401
    STAGE_REF_NAMES, THETA0, THETA_NAMES, OcpConfig, build_ocp, path_constraints,
)
from diff_weight_mpc.mpc.reference_params import stage_parameters, vehicle_parameters
from plannertrack_py.utils import angle_diff
from plannertrack_py.plant import Perturbation, Plant

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent
PARAMS = SRC / "scenarios" / "ground_vehicle_racing_rl" / "params"

NODE = 1  # the shooting node the paper differentiates (Table I)
OUTPUTS = ("x1", "u1", "u0", "value")  # what is differentiated with respect to θ


# ----------------------------------------------------------------------------------------------- data


@dataclass(frozen=True)
class CheckConfig:
    """Settings of the check (defaults are the starting point; 5.4 decides the final ones)."""

    theta: tuple = THETA0  # the weights at which the derivatives are taken
    epsilons: tuple = (1e-2, 1e-3, 1e-4, 1e-5)  # relative finite-difference steps (5.4 sweep)
    fd_epsilon: float = 1e-3  # main comparison step: bottom of the error-vs-ε V (first sweep, 2026-10-24)
    sqp_tol: float = 1e-10  # convergence of the reference solves (KKT residuals)
    sqp_max_iter: int = 200
    rti_steps: int = 50  # closed-loop steps for the RTI-vs-converged check (5.5), 1 s at 0.02 s
    rti_s0: float = 16.0  # start of that run: the approach to the baseline's failure corner (s ≈ 19 m)
    corner_s: float = 19.0  # where the baseline's perturbed cars leave the track (results/baseline.md)
    # provisional pass criterion (5.4), on x₁ and u₁ at the best ε of the sweep
    pass_cosine: float = 0.999
    pass_rel_error: float = 1e-3
    seed: int = 0  # for the synthetic loss weights c, d
    feature_nodes: tuple = (1, 2, 5, 10, 20)  # nodes for loss_term_gradients (the paper's loss uses node 1)
    scale_n: float = 0.049  # typical lateral error, m (baseline median RMS |n|, results/baseline.md)
    scale_v: float = 0.39  # typical speed error, m/s (baseline median RMS |v − v_ref|)
    loss: LossConfig = LossConfig()  # the paper's α…δ, for the paper-loss split in loss_term_gradients
    ocp: OcpConfig = OcpConfig()  # N, dt, slack weights, track margin: as used in training
    # track slack weights (L1, L2): both candidates are checked (steps.md 4.9: (1e2, 1e3) is the baseline,
    # (1e4, 1e5) proposed); each needs its own compiled solver, ocp.slack_track is replaced per run
    slack_tracks: tuple = ((1e2, 1e3), (1e4, 1e5))


@dataclass
class TestCase:
    """One OCP instance: the measured state and the stage parameters it is solved with (5.3)."""

    name: str
    description: str
    x0: np.ndarray  # (NX,)
    params: dict  # stage parameters {name: (1, N+1, 1)} (+ "vehicle" (1, N+1, 5) for an oracle case)
    s0: float = np.nan  # arc length of the state on the reference
    active: dict = field(default_factory=dict)  # filled by active_set(): which constraints are active


@dataclass
class Solution:
    """A solved OCP instance (5.0)."""

    x: np.ndarray  # (N+1, NX) planned states
    u: np.ndarray  # (N, NU) planned inputs
    value: float  # optimal cost V
    status: int  # acados status (0 = success)
    iterations: int  # SQP iterations
    residuals: np.ndarray  # (4,) KKT residuals [stationarity, equality, inequality, complementarity]
    ctx: object = None  # leap-c context (for the sensitivities and as a warm start)


@dataclass
class Sensitivities:
    """Derivatives with respect to θ (6 weights), one entry per output (5.1, 5.2)."""

    dx1: np.ndarray  # (NX, 6)    ∂x₁/∂θ
    du1: np.ndarray  # (NU, 6)    ∂u₁/∂θ
    du0: np.ndarray  # (NU, 6)    ∂u₀/∂θ
    dvalue: np.ndarray  # (6,)    ∂V/∂θ
    dloss: np.ndarray = None  # (6,) ∂L/∂θ of the synthetic loss (backward() for leap-c, chain rule for FD)
    dx: np.ndarray = None  # (N+1, NX, 6) ∂x_k/∂θ over the whole plan (leap-c only; for loss_term_gradients)
    source: str = ""  # "leap-c" or "fd(eps)"
    failed: list = field(default_factory=list)  # FD only: (weight, ±1, status) of perturbed solves that failed


# ------------------------------------------------------------------------------------- 5.0 setup


def build_diff_mpc(plant, cfg, nlp_solver="SQP", export_directory=None, name="leap_c_check", n_batch=1):
    """The Step 4 OCP compiled with leap-c, with the ReferenceInitializer.

    Args:
        plant: a Plant (nominal parameters, limits, reference).
        cfg: CheckConfig; the OCP settings come from cfg.ocp, with nlp_solver replaced and, for "SQP",
            tolerance cfg.sqp_tol and cfg.sqp_max_iter iterations.
        nlp_solver: "SQP" (converged reference solves) or "SQP_RTI" (the controller's setting, 5.5).
        n_batch: initial batch size of leap-c's solvers (5.6 solves several cases at once).

    Returns:
        an AcadosDiffMpcTorch (float64), with its ReferenceInitializer installed.
    """
    ocp_cfg = replace(cfg.ocp, nlp_solver=nlp_solver)
    if nlp_solver == "SQP":
        ocp_cfg = replace(ocp_cfg, tol=cfg.sqp_tol, max_iter=cfg.sqp_max_iter)
    ocp, manager = build_ocp(plant.nominal_params, plant.limits, ocp_cfg, name=name)
    diff_mpc = AcadosDiffMpcTorch(ocp, manager, export_directory=export_directory, n_batch_init=n_batch,
                                  verbose=False, dtype=torch.float64)
    # leap-c's default initial guess (all zeros, vx = 0) gives NaN; start every cold solve on the reference
    diff_mpc.diff_mpc_fun.initializer = ReferenceInitializer(diff_mpc.diff_mpc_fun.ocp, manager,
                                                             plant.nominal_params.wheelbase)
    return diff_mpc


def solve(diff_mpc, case, theta, ctx=None):
    """Solve one TestCase with weights theta (6,), optionally warm-started from ``ctx``.

    Returns a Solution (plan, value, status, iterations, KKT residuals, ctx). For "SQP" check status 0 and
    residuals below cfg.sqp_tol; the finite differences are only as good as these solves.
    """
    ctx, _, x, u, value = _forward(diff_mpc, case, theta, ctx)
    solver = diff_mpc.diff_mpc_fun.forward_batch_solver.ocp_solvers[0]
    return Solution(
        x=x[0].detach().numpy(),
        u=u[0].detach().numpy(),
        value=float(value[0, 0]),
        status=int(np.asarray(ctx.status).ravel()[0]),
        iterations=int(solver.get_stats("sqp_iter")),
        residuals=np.asarray(solver.get_residuals(recompute=True)).ravel(),
        ctx=ctx,
    )


def _forward(diff_mpc, case, theta, ctx=None):
    """One leap-c forward on a single case. ``theta`` (6,) as numpy, or a torch tensor to keep the graph
    (5.2). Returns leap-c's ``(ctx, u0, x, u, value)``, batch size 1."""
    theta = theta if isinstance(theta, torch.Tensor) else torch.as_tensor(np.asarray(theta, dtype=float))
    params = {**case.params, "theta": theta.to(torch.float64).reshape(1, -1)}
    x0 = torch.as_tensor(np.asarray(case.x0, dtype=float).reshape(1, NX))
    return diff_mpc(x0, params=params, ctx=ctx)


# --------------------------------------------------------------------------- 5.1 finite differences


def finite_difference_sensitivities(diff_mpc, case, theta, epsilon, synthetic_loss=None):
    """Central finite differences of x₁, u₁, u₀ and V with respect to each weight.

    For each i: θ⁺ = θ with θ_i·(1 + ε), θ⁻ with θ_i·(1 - ε) (relative steps: the weights span 0.01 to 50);
    ∂out/∂θ_i ≈ (out(θ⁺) - out(θ⁻)) / (2·ε·θ_i). Each perturbed solve is warm-started from the unperturbed
    solution and converged to tight tolerance; check every status. With ``synthetic_loss`` also ∂L/∂θ by
    the chain rule from the finite differences of x₁ and u₁.

    A perturbed solve that does not converge (status != 0 or non-finite plan) leaves column i NaN and is
    listed in ``Sensitivities.failed``; the other columns are still usable.

    Returns:
        Sensitivities (source "fd(eps)").
    """
    theta = np.asarray(theta, dtype=float)
    base = solve(diff_mpc, case, theta)
    if not _converged(base):
        raise RuntimeError(f"{case.name}: unperturbed solve failed (status {base.status}, "
                           f"residuals {base.residuals})")
    # leap-c builds a new ctx in every forward, so base.ctx stays the warm start for all perturbed solves
    dx1, du1, du0, dvalue = (np.full(shape + (theta.size,), np.nan) for shape in ((NX,), (NU,), (NU,), ()))
    failed = []
    for i in range(theta.size):
        h = epsilon * theta[i]
        sols = {}
        for sign in (+1, -1):
            perturbed = theta.copy()
            perturbed[i] += sign * h
            sols[sign] = solve(diff_mpc, case, perturbed, ctx=base.ctx)
            if not _converged(sols[sign]):
                failed.append((THETA_NAMES[i], sign, sols[sign].status))
        if not all(_converged(s) for s in sols.values()):
            continue
        plus, minus = sols[+1], sols[-1]
        dx1[:, i] = (plus.x[NODE] - minus.x[NODE]) / (2 * h)
        du1[:, i] = (plus.u[NODE] - minus.u[NODE]) / (2 * h)
        du0[:, i] = (plus.u[0] - minus.u[0]) / (2 * h)
        dvalue[i] = (plus.value - minus.value) / (2 * h)
    dloss = None
    if synthetic_loss is not None:
        c, d = synthetic_loss
        dloss = c @ dx1 + d @ du1  # L = cᵀx₁ + dᵀu₁ is linear: ∂L/∂θ = cᵀ·∂x₁/∂θ + dᵀ·∂u₁/∂θ
    return Sensitivities(dx1, du1, du0, dvalue, dloss=dloss, source=f"fd({epsilon:g})", failed=failed)


def _converged(solution):
    """Status 0 (acados: converged to the solver's tolerance) and a finite plan."""
    return (solution.status == 0 and np.all(np.isfinite(solution.x)) and np.all(np.isfinite(solution.u))
            and np.isfinite(solution.value))


# --------------------------------------------------------------------------- 5.2 leap-c sensitivities


def make_synthetic_loss(seed):
    """Fixed random weights for L = cᵀx₁ + dᵀu₁.

    Returns:
        ``(c (NX,), d (NU,))``; scale them so every state/input component contributes comparably.
    """
    rng = np.random.default_rng(seed)
    # c_j = N(0, 1) / typical size of component j, so no component dominates ∂L/∂θ (rough sizes, in units of
    # STATE_NAMES / (jerk, steer rate))
    x_scale = np.array([0.05, 0.05, 0.05, 0.4, 0.1, 0.2, 0.05, 1.0])  # x, y, yaw, vx, vy, yaw_rate, steer, acc
    u_scale = np.array([8.0, 0.25])  # jerk, steer rate (the loss thresholds)
    return rng.standard_normal(NX) / x_scale, rng.standard_normal(NU) / u_scale


def leapc_sensitivities(diff_mpc, case, theta, synthetic_loss=None):
    """leap-c's derivatives at the same solution.

    - ∂x_k/∂θ, ∂u_k/∂θ per stage: acados' adjoint with an identity seed on that stage alone
      (``_stage_sensitivities``). NOT leap-c's ``sensitivity(ctx, "dx_dp_global")`` / ``"du_dp_global"``: those
      seed every stage at once and return the SUM over stages, Σ_k ∂x_k/∂θ (NX, 6) (found 2026-10-24);
      node 1 is picked from the per-stage arrays, the whole plan is kept in ``dx``;
    - ∂u₀/∂θ: ``"du0_dp_global"`` (stage 0 only, so correct as is); ∂V/∂θ: ``"dvalue_dp_global"``;
    - ∂L/∂θ: theta as a leaf tensor with requires_grad, L from the returned x, u tensors, ``L.backward()``;
      must equal cᵀ·∂x₁/∂θ + dᵀ·∂u₁/∂θ (a consistency check inside leap-c itself).

    Returns:
        Sensitivities (source "leap-c").
    """
    theta_t = torch.tensor(np.asarray(theta, dtype=float), requires_grad=True)
    ctx, _, x, u, _ = _forward(diff_mpc, case, theta_t)
    status = int(np.asarray(ctx.status).ravel()[0])
    if status != 0:
        raise RuntimeError(f"{case.name}: solve failed (status {status})")
    sens = _jacobians(diff_mpc.diff_mpc_fun, ctx, theta_t.numel())[0]
    if synthetic_loss is not None:
        # the training path: autograd through leap-c's backward (adjoint), not the Jacobians above
        c, d = (torch.as_tensor(a, dtype=x.dtype) for a in synthetic_loss)
        loss = c @ x[0, NODE] + d @ u[0, NODE]
        loss.backward()
        sens.dloss = theta_t.grad.numpy().copy()
    return sens


def _jacobians(fun, ctx, n_theta):
    """leap-c's derivatives at a solved ``ctx`` (any batch size, no re-solve): one Sensitivities per batch row
    (no loss). Also used at an RTI iterate (5.5) and for batched solves (5.6)."""
    batch = ctx.solver_input.batch_size
    N = fun.ocp.solver_options.N_horizon
    dx = _stage_sensitivities(fun, ctx, "x", N + 1, n_theta)
    du = _stage_sensitivities(fun, ctx, "u", N, n_theta)
    # leap-c returns (batch, output components, p_global); p_global is theta alone (the only differentiable
    # parameter). Reshape so a wrong layout fails loudly instead of being silently misread.
    du0 = fun.sensitivity(ctx, "du0_dp_global").reshape(batch, NU, n_theta)
    dvalue = fun.sensitivity(ctx, "dvalue_dp_global").reshape(batch, n_theta)
    return [Sensitivities(dx[b, NODE], du[b, NODE], du0[b], dvalue[b], dx=dx[b], source="leap-c")
            for b in range(batch)]


def _stage_sensitivities(fun, ctx, kind, n_stages, n_theta):
    """(batch, n_stages, n, n_theta): ∂x_k/∂θ (kind "x") or ∂u_k/∂θ (kind "u") for every stage k.

    One acados adjoint per stage, seeded with the identity on that stage only (leap-c's own
    "dx_dp_global" seeds all stages together and so returns their sum).
    """
    prepare_batch_solver_for_backward(fun.backward_batch_solver, ctx.iterate, ctx.solver_input)
    batch = ctx.solver_input.batch_size
    n = NX if kind == "x" else NU
    seed = np.repeat(np.eye(n)[None], batch, axis=0)  # (batch, n, n seeds)
    out = np.empty((batch, n_stages, n, n_theta))
    for k in range(n_stages):
        seeds = ([(k, seed)], []) if kind == "x" else ([], [(k, seed)])
        out[:, k] = fun.backward_batch_solver.eval_adjoint_solution_sensitivity(*seeds, "p_global", False)
    return out


# ------------------------------------------------------------------------------- 5.3 test states


def make_test_cases(plant, cfg):
    """The OCP instances the derivatives are checked at.

    Cases (trimmed states on the reference from ``Plant.start_state``, located from the reference itself):
      - "straight": the start of the 3 m stretch with the least curvature; no constraint expected near its limit;
      - "braking": the strongest braking (most negative a_x,ref), entering a corner;
      - "hairpin": 0.5 m before the point of largest curvature (s ≈ 1.2 m, radius ≈ 1 m), shifted towards the
        nearer edge by half of the MPC's room there;
      - "baseline_corner": the same at the raceline's closest approach to an edge nearest cfg.corner_s (s ≈ 19 m,
        where the baseline's perturbed cars leave the track);
      - "ellipse": a μ × 0.8 car at the largest reference a_lat, oracle model (true μ in the OCP): the reference
        is too fast for that grip, so the friction ellipse is at its limit;
      - "off_line": a moderate corner, 40 % of the room to the left edge and 0.15 rad heading offset.

    Every case carries the "vehicle" stage parameter (nominal, or the true one for the oracle), so cases can be
    stacked into one batch (5.6).

    Returns:
        list of TestCase.
    """
    ref, N, dt = plant.reference, cfg.ocp.N, cfg.ocp.dt
    step = 0.05
    s_grid = np.arange(0.0, ref.length, step)
    r = ref.sample(s_grid)
    half = 0.5 * plant.limits.vehicle_width
    clearance = np.minimum(r["w_left"], r["w_right"]) - half  # raceline to the nearer edge, car side
    ahead = int(round(3.0 / step))
    kappa_ahead = np.array([np.abs(np.take(r["kappa"], np.arange(i, i + ahead), mode="wrap")).max()
                            for i in range(s_grid.size)])
    abs_kappa = np.abs(r["kappa"])
    # the raceline is exactly 10 cm (the planner's margin) from an edge at many points, so "closest approach"
    # is a tie: the corner cases are located explicitly (largest curvature; clearance minimum nearest corner_s)
    minima = [i for i in range(s_grid.size) if clearance[i] <= clearance[i - 1]
              and clearance[i] <= clearance[(i + 1) % s_grid.size] and clearance[i] < clearance.min() + 0.01]
    i_corner = min(minima, key=lambda i: abs(s_grid[i] - cfg.corner_s))

    def approach(i_apex):
        """(s0, dn): 0.5 m before the apex, shifted half the MPC's room towards the nearer edge."""
        left_is_near = r["w_left"][i_apex] < r["w_right"][i_apex]
        s0 = (s_grid[i_apex] - 0.5) % ref.length
        lim_left, lim_right = plant.track_limits(s0)
        room = (lim_left if left_is_near else -lim_right) - cfg.ocp.track_margin  # MPC's room to the near edge
        return s0, (1.0 if left_is_near else -1.0) * 0.5 * room

    i_hairpin = int(np.argmax(abs_kappa))
    s_hairpin, dn_hairpin = approach(i_hairpin)
    s_corner, dn_corner = approach(i_corner)
    s_mid = s_grid[int(np.argmin(np.abs(abs_kappa - 0.5 * abs_kappa.max())))]
    dn_mid = 0.4 * (plant.track_limits(s_mid)[0] - cfg.ocp.track_margin)
    specs = [
        ("straight", "least curvature over the next 3 m", s_grid[int(np.argmin(kappa_ahead))], {}, None),
        ("braking", "strongest braking of the reference", s_grid[int(np.argmin(r["ax"]))], {}, None),
        ("hairpin", f"0.5 m before the largest curvature (s = {s_grid[i_hairpin]:.2f} m, radius "
                    f"{1 / abs_kappa[i_hairpin]:.2f} m), shifted {100 * dn_hairpin:+.1f} cm towards the nearer edge",
         s_hairpin, {"dn": dn_hairpin}, None),
        ("baseline_corner", f"0.5 m before the clearance minimum at s = {s_grid[i_corner]:.2f} m (raceline "
                            f"{100 * clearance[i_corner]:.1f} cm from the edge), shifted {100 * dn_corner:+.1f} cm "
                            "towards it", s_corner, {"dn": dn_corner}, None),
        ("ellipse", "μ × 0.8 car, oracle model, largest reference a_lat", s_grid[int(np.argmax(np.abs(r["a_lat"])))],
         {}, Perturbation(mu_scale=0.8)),
        ("off_line", f"moderate corner, dn {100 * dn_mid:+.1f} cm, dpsi +0.15 rad", s_mid,
         {"dn": dn_mid, "dpsi": 0.15}, None),
    ]
    cases = []
    for name, description, s0, offsets, perturbation in specs:
        perturbation = perturbation or Perturbation()
        plant.set_perturbation(perturbation)  # the start trim uses the true parameters
        state, converged = plant.start_state(float(s0), **offsets)
        plant.set_perturbation(Perturbation())
        x0, params = stage_parameters(ref, state, float(s0), N, dt, plant.limits.vehicle_width,
                                      track_margin=cfg.ocp.track_margin)
        params.update(vehicle_parameters([perturbation.apply(plant.nominal_params)], N))
        if not converged:
            description += " (start trim did not converge)"
        cases.append(TestCase(name, description, x0[0], params, float(s0)))
    return cases


def active_set(plant, case, solution, tol=1e-6):
    """Which constraints are active in the solution, per stage, from the primal plan.

    Soft constraints (h ≤ 0, stages 1..N): active where h ≥ −tol (at the bound or violated, i.e. slack used or
    about to be). Hard bounds: within a relative tol of the bound (inputs on stages 0..N−1, δ and a on 1..N).
    Also the margin of each soft constraint (−max_k h_k: how far its closest stage is from activating) and the
    largest violation. Derivatives are only well defined where the active set does not change under the
    ±ε perturbation: a soft constraint with a margin near 0 but not active marks a possible kink.

    Returns:
        {"stages": {name: [stages]}, "margin": {soft name: float}, "violation": {soft name: float},
         "active": [names with at least one active stage]}.
    """
    lim, x, u = plant.limits, solution.x, solution.u
    N = u.shape[0]
    h_fun = _constraint_function(plant)
    refs = np.concatenate([case.params[name][0] for name in STAGE_REF_NAMES], axis=1)  # (N+1, 7)
    vehicle = case.params["vehicle"][0]
    h = np.array([np.asarray(h_fun(x[k], refs[k], vehicle[k])).ravel() for k in range(1, N + 1)])  # (N, 3)
    soft = ("track_left", "track_right", "ellipse")
    stages = {name: (np.flatnonzero(h[:, j] >= -tol) + 1).tolist() for j, name in enumerate(soft)}

    def at_bound(values, bound):
        return np.abs(values) >= bound * (1.0 - tol)

    stages["jerk"] = np.flatnonzero(at_bound(u[:, 0], lim.max_jerk)).tolist()
    stages["steer_rate"] = np.flatnonzero(at_bound(u[:, 1], lim.max_steer_rate)).tolist()
    stages["steer"] = (np.flatnonzero(at_bound(x[1:, 6], lim.max_steer)) + 1).tolist()
    a = x[1:, 7]
    stages["acc"] = (np.flatnonzero((a >= lim.max_accel * (1 - tol)) | (a <= -lim.max_decel * (1 - tol))) + 1).tolist()
    return {
        "stages": stages,
        "margin": {name: float(-h[:, j].max()) for j, name in enumerate(soft)},
        "violation": {name: float(max(h[:, j].max(), 0.0)) for j, name in enumerate(soft)},
        "active": [name for name, ks in stages.items() if ks],
    }


def _constraint_function(plant):
    """The OCP's soft constraints h(x, ref, vehicle) (track left, track right, ellipse) as a CasADi Function."""
    x, r, v = ca.SX.sym("x", NX), ca.SX.sym("ref", len(STAGE_REF_NAMES)), ca.SX.sym("vehicle", 5)
    ref = {name: r[i] for i, name in enumerate(STAGE_REF_NAMES)}
    h = path_constraints(x, ref, v, plant.limits, plant.nominal_params.mu)
    return ca.Function("h", [x, r, v], [h])


# ------------------------------------------------------------------------------- 5.4 comparison


def compare(leapc, fd):
    """Per output (x1, u1, u0, value, and loss when both have it): relative error ‖S_leapc − S_fd‖ / ‖S_fd‖, the
    largest component-wise error, the cosine between the flattened gradient vectors, and ‖S_fd‖. Weights whose
    column is NaN in either (a failed finite-difference solve) are left out; ``n_weights`` says how many remain.
    Works for any two Sensitivities (5.5 compares RTI against converged, 5.6 batched against single).

    Returns:
        dict output -> {"rel_error", "max_abs_error", "cosine", "norm", "n_weights"}.
    """
    pairs = {"x1": (leapc.dx1, fd.dx1), "u1": (leapc.du1, fd.du1), "u0": (leapc.du0, fd.du0),
             "value": (leapc.dvalue, fd.dvalue)}
    if leapc.dloss is not None and fd.dloss is not None:
        pairs["loss"] = (leapc.dloss, fd.dloss)
    out = {}
    for name, (a, b) in pairs.items():
        a = np.asarray(a, dtype=float).reshape(-1, len(THETA_NAMES))
        b = np.asarray(b, dtype=float).reshape(-1, len(THETA_NAMES))
        cols = np.all(np.isfinite(a), axis=0) & np.all(np.isfinite(b), axis=0)
        a, b = a[:, cols].ravel(), b[:, cols].ravel()
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        out[name] = {
            "rel_error": float(np.linalg.norm(a - b) / nb) if nb > 0 else np.nan,
            "max_abs_error": float(np.abs(a - b).max()) if a.size else np.nan,
            "cosine": float(a @ b / (na * nb)) if na * nb > 0 else np.nan,
            "norm": float(nb),
            "n_weights": int(cols.sum()),
        }
    return out


def epsilon_sweep(diff_mpc, case, theta, epsilons, synthetic_loss=None, leapc=None):
    """compare(leap-c, finite differences at each ε) for one case.

    Expected shape of the curve: the error falls with ε (truncation error ~ε²) until solver noise takes over
    (~tol/ε); the minimum tells the usable ε and the agreement achievable (finite_difference_notes.md).

    Returns:
        {eps: {"compare": compare(...), "failed": the failed perturbed solves}}.
    """
    leapc = leapc if leapc is not None else leapc_sensitivities(diff_mpc, case, theta, synthetic_loss)
    sweep = {}
    for eps in epsilons:
        fd = finite_difference_sensitivities(diff_mpc, case, theta, eps, synthetic_loss)
        sweep[float(eps)] = {"compare": compare(leapc, fd), "failed": fd.failed}
    return sweep


def best_epsilon(sweep, output="x1"):
    """(ε, compare at that ε) with the smallest relative error of ``output`` in an epsilon_sweep."""
    eps = min(sweep, key=lambda e: np.nan_to_num(sweep[e]["compare"][output]["rel_error"], nan=np.inf))
    return eps, sweep[eps]["compare"]


def loss_term_gradients(sens, solution, case, cfg):
    """How much θ can move the lateral error vs the speed, per node (for the Step 6 loss design, steps.md §8).

    From the (verified) leap-c sensitivities ``sens.dx`` at the nodes k in cfg.feature_nodes:
      - ∂n_k/∂θ = [−sin ψ_ref,k, cos ψ_ref,k] · ∂[X, Y]_k/∂θ (n as in learning/loss.py node_loss),
        ∂v_k/∂θ = ∂vx_k/∂θ;
      - sensitivity, scale-free: g_n = θ ⊙ ∂n_k/∂θ / cfg.scale_n, g_v = θ ⊙ ∂v_k/∂θ / cfg.scale_v (a relative
        change of each weight, in units of the typical error) and the ratio ‖g_n‖ / ‖g_v‖. This is the structural
        question: does θ reach n at node 1 at all (n is one integration further from the inputs than v)?
      - with the paper's loss (cfg.loss α, β at this solution's n_k, dv_k): the two gradient parts
        2α·n_k·∂n_k/∂θ and 2β·dv_k·∂v_k/∂θ, and the ratio of their norms. This is the weighting question.

    Returns:
        dict node -> {"dn", "dv" (6,), "g_n", "g_v" (6,), "sens_ratio", "paper_grad_n", "paper_grad_v" (6,),
        "paper_ratio", "n", "dv_err"}.
    """
    theta = np.asarray(cfg.theta, dtype=float)
    ref = {name: case.params[name][0, :, 0] for name in ("x_ref", "y_ref", "psi_ref", "v_ref")}
    out = {}
    for k in cfg.feature_nodes:
        sin, cos = np.sin(ref["psi_ref"][k]), np.cos(ref["psi_ref"][k])
        dn = -sin * sens.dx[k, 0] + cos * sens.dx[k, 1]
        dv = sens.dx[k, 3]
        n = -sin * (solution.x[k, 0] - ref["x_ref"][k]) + cos * (solution.x[k, 1] - ref["y_ref"][k])
        dv_err = solution.x[k, 3] - ref["v_ref"][k]
        g_n, g_v = theta * dn / cfg.scale_n, theta * dv / cfg.scale_v
        grad_n, grad_v = 2 * cfg.loss.alpha * n * dn, 2 * cfg.loss.beta * dv_err * dv
        out[int(k)] = {
            "dn": dn, "dv": dv, "g_n": g_n, "g_v": g_v,
            "sens_ratio": float(np.linalg.norm(g_n) / np.linalg.norm(g_v)),
            "paper_grad_n": grad_n, "paper_grad_v": grad_v,
            "paper_ratio": float(np.linalg.norm(grad_n) / max(np.linalg.norm(grad_v), 1e-300)),
            "n": float(n), "dv_err": float(dv_err),
        }
    return out


# NOT USED (kept for reference, 2026-10-24): ∂V/∂θ by the envelope theorem, a third value for the "value" output
# next to leap-c and the finite differences. At an optimum, and with constraints that do not depend on θ, only the
# explicit θ-dependence of the cost counts: ∂V/∂θ_i = Σ_k scale_k · ½·e_{i,k}*², with acados' cost scaling
# scale_k = dt for the stages and 1 for the terminal stage. Tested on two states: equal to leap-c's
# "dvalue_dp_global" to round-off (0.0); scale 1 or 1/N gives errors of 12 and 0.6. Not used because acados computes
# its value gradient the same way (∂Lagrangian/∂θ at the solution), so it confirms the cost scaling rather than
# being an independent method; the independent check of V is the finite difference (5.1).
# To use: also import error_function and STAGE_REF_NAMES from diff_weight_mpc.mpc.ocp, and in main() add
# "envelope_dvalue": envelope_value_gradient(solution, case, cfg) to the per-case results.
#
# def envelope_value_gradient(solution, case, cfg):
#     """(6,) ∂V/∂θ = Σ_k scale_k·½·(e_{i,k}² for the 4 state terms, u_{j,k}² for the 2 input terms)."""
#     N, dt = cfg.ocp.N, cfg.ocp.dt
#     refs = np.concatenate([case.params[name][0] for name in STAGE_REF_NAMES], axis=1)  # (N+1, 7)
#     errors = error_function()  # (x, ref) -> [n, e_psi, e_v, e_alat], THETA_NAMES order of the q's
#     e = np.array([np.asarray(errors(solution.x[k], refs[k])).ravel() for k in range(N + 1)])
#     squares = np.zeros((N + 1, len(THETA_NAMES)))
#     squares[:, :4] = e**2
#     squares[:N, 4:] = solution.u**2  # jerk, steer rate (the terminal stage has no input)
#     scale = np.r_[np.full(N, dt), 1.0]  # acados: stage costs × dt, terminal × 1
#     return 0.5 * scale @ squares


# ------------------------------------------------------------------------- 5.5 RTI vs converged


def rti_vs_converged(plant, diff_mpc_rti, diff_mpc_sqp, cfg):
    """The controller's setting against the converged reference, along a short closed-loop run.

    The nominal car starts on the reference at cfg.rti_s0 (no start noise) and is driven cfg.rti_steps control
    steps by the RTI solver, warm-started from its previous ctx (one SQP iteration per step, as MpcController
    does; yaw kept continuous the same way). At every step the same state is also solved by the converged SQP
    solver (warm-started from its own previous solution). Recorded per step: the KKT residuals after RTI, the
    SQP iterations, and compare(leap-c sensitivities at the RTI iterate, leap-c sensitivities at the converged
    solution); leap-c's derivatives at a non-KKT point are what training would use with RTI.

    Returns:
        list of per-step dicts {"step", "s", "rti_status", "sqp_status", "rti_residuals", "sqp_iterations",
        "du0" (‖u₀_RTI − u₀_SQP‖), "compare"}.
    """
    N, dt = cfg.ocp.N, cfg.ocp.dt
    theta = np.asarray(cfg.theta, dtype=float)
    noise = plant.start_noise
    plant.start_noise = replace(noise, n_std=0.0, psi_std=0.0, v_std=0.0)
    try:
        state, info = plant.reset(seed=cfg.seed, s0=cfg.rti_s0, perturbation=Perturbation())
    finally:
        plant.start_noise = noise
    s = info["s0"]
    vehicle = vehicle_parameters([plant.nominal_params], N)
    ctx_rti = ctx_sqp = None
    yaw_prev = None
    rows = []
    for k in range(cfg.rti_steps):
        state = np.array(state, dtype=float)
        if yaw_prev is not None:  # continuous yaw, as MpcController (the warm start is on the previous branch)
            state[2] = yaw_prev + angle_diff(state[2], yaw_prev)
        yaw_prev = state[2]
        x0, params = stage_parameters(plant.reference, state, s, N, dt, plant.limits.vehicle_width,
                                      track_margin=cfg.ocp.track_margin)
        params.update(vehicle)
        case = TestCase(f"rti_{k}", "", x0[0], params, s)
        rti = solve(diff_mpc_rti, case, theta, ctx_rti)
        sqp = solve(diff_mpc_sqp, case, theta, ctx_sqp)
        row = {"step": k, "s": float(s), "rti_status": rti.status, "sqp_status": sqp.status,
               "rti_residuals": rti.residuals, "sqp_iterations": sqp.iterations}
        if _converged(sqp) and _converged(rti):
            at_rti = _jacobians(diff_mpc_rti.diff_mpc_fun, rti.ctx, theta.size)[0]
            at_sqp = _jacobians(diff_mpc_sqp.diff_mpc_fun, sqp.ctx, theta.size)[0]
            row["compare"] = compare(at_rti, at_sqp)
            row["du0"] = float(np.linalg.norm(rti.u[0] - sqp.u[0]))
        rows.append(row)
        ctx_rti, ctx_sqp = rti.ctx, sqp.ctx
        state, info = plant.step(rti.u[0])  # the controller applies the RTI input
        s = info["s"]
        if info["terminated"]:
            break
    return rows


# ------------------------------------------------------------------------------ 5.6 batch vs single


def batch_vs_single(diff_mpc, cases, theta):
    """All cases solved in one batched call vs each alone: the sensitivities must agree to solver precision.

    Both are cold solves from the ReferenceInitializer, converged; the batched Jacobians come from one backward
    preparation of the whole batch (``_jacobians`` over all rows).

    Returns:
        dict case name -> {"status": batched solve status, "compare": compare(batched, single)}.
    """
    theta = np.asarray(theta, dtype=float)
    x0 = torch.as_tensor(np.stack([case.x0 for case in cases]))
    params = {name: np.concatenate([case.params[name] for case in cases]) for name in cases[0].params}
    params["theta"] = torch.as_tensor(np.tile(theta, (len(cases), 1)))
    ctx, *_ = diff_mpc(x0, params=params)
    status = np.asarray(ctx.status).ravel()
    batched = _jacobians(diff_mpc.diff_mpc_fun, ctx, theta.size)
    return {case.name: {"status": int(st), "compare": compare(row, leapc_sensitivities(diff_mpc, case, theta))}
            for case, row, st in zip(cases, batched, status)}


# ------------------------------------------------------------------------------------- 5.7 report

# chart palette and style shared with closed_loop_baseline.py (categorical order fixed)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
GRID_COLOR = "#e1e0d9"


def _style(ax):
    ax.grid(True, color=GRID_COLOR, lw=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    return obj


def _fmt(v, spec=".1e"):
    return "–" if v is None or not np.isfinite(v) else format(v, spec)


def write_report(results, out_dir, cfg):
    """``results`` is {"cases": {name: description and s0}, slack tag: {"fd", "rti", "batch"}}, one slack entry per
    CheckConfig.slack_tracks. Writes to out_dir (rl_framework/data/leap_c_check/):

      - results.json: everything;
      - report.md: per slack weighting, the case table (active constraints, relative error of x₁, u₁, u₀, V and the
        synthetic loss at fd_epsilon, the best ε, relative error and cosine of x₁ and u₁ there, pass/fail by
        cfg.pass_cosine and cfg.pass_rel_error on x₁ and u₁), the lateral-vs-speed table (loss_term_gradients),
        the RTI-vs-converged summary and the batch check;
      - sweep.png: relative error of x₁ and u₁ against ε, one line per case, one row per slack weighting;
      - rti.png: per step of the RTI run, the stationarity residual and the relative error of x₁ and u₁.

    The write-up for the docs (project_docs/learning_based_controls/results/leap_c_check.md) is made from these.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "results.json").write_text(json.dumps(_jsonable(results), indent=1))
    tags = [t for t in results if t != "cases"]
    lines = ["# leap-c sensitivity check (Step 5)", "",
             f"θ = {tuple(cfg.theta)}, N = {cfg.ocp.N}, dt = {cfg.ocp.dt}, track margin {cfg.ocp.track_margin} m; "
             f"reference solves SQP to {cfg.sqp_tol:g}; main ε = {cfg.fd_epsilon:g}. Pass: cosine ≥ {cfg.pass_cosine} "
             f"and relative error ≤ {cfg.pass_rel_error:g} for x₁ and u₁ at the best ε.", "", "## Cases", ""]
    lines += [f"- **{name}** (s0 = {c['s0']:.2f} m): {c['description']}" for name, c in results["cases"].items()]
    outputs = ("x1", "u1", "u0", "value", "loss")
    for tag in tags:
        res = results[tag]
        lines += ["", f"## Track slack weights {tag.removeprefix('slack_')}", ""]
        if "fd" in res:
            lines += ["| case | active (stages) | track margin L / R [cm] | ellipse margin | "
                      + " | ".join(f"rel {o} (main ε)" for o in outputs)
                      + " | best ε | rel x1 / u1 at best | cos x1 / u1 at best | pass |",
                      "|" + "---|" * (9 + len(outputs))]
            for name, r in res["fd"].items():
                act = r["active"]
                active = ", ".join(f"{a} ({len(act['stages'][a])})" for a in act["active"]) or "none"
                main = r["sweep"][cfg.fd_epsilon]["compare"]
                eps, best = best_epsilon(r["sweep"])
                ok = all(best[o]["cosine"] >= cfg.pass_cosine and best[o]["rel_error"] <= cfg.pass_rel_error
                         for o in ("x1", "u1"))
                lines.append(
                    f"| {name} | {active} | {100 * act['margin']['track_left']:.1f} / "
                    f"{100 * act['margin']['track_right']:.1f} | {act['margin']['ellipse']:.3f} | "
                    + " | ".join(_fmt(main[o]["rel_error"]) if o in main else "–" for o in outputs)
                    + f" | {eps:g} | {_fmt(best['x1']['rel_error'])} / {_fmt(best['u1']['rel_error'])} | "
                    f"{_fmt(best['x1']['cosine'], '.6f')} / {_fmt(best['u1']['cosine'], '.6f')} | "
                    f"{'yes' if ok else '**no**'} |")
            failed = {f"{n} ε={e:g}": v["failed"] for n, r in res["fd"].items() for e, v in r["sweep"].items()
                      if v["failed"]}
            if failed:
                lines += ["", f"Failed finite-difference solves: {failed}"]
            lines += ["", "Lateral vs speed (loss_term_gradients): ‖θ⊙∂n_k/∂θ‖/σ_n over ‖θ⊙∂v_k/∂θ‖/σ_v "
                      "(sens), and the paper loss's ‖2αn·∂n/∂θ‖ over ‖2β·dv·∂v/∂θ‖ (paper).", "",
                      "| case | " + " | ".join(f"node {k} sens / paper" for k in cfg.feature_nodes) + " |",
                      "|---|" + "---|" * len(cfg.feature_nodes)]
            for name, r in res["fd"].items():
                lt = r["loss_terms"]
                lines.append(f"| {name} | " + " | ".join(
                    f"{_fmt(lt[k]['sens_ratio'], '.2g')} / {_fmt(lt[k]['paper_ratio'], '.1e')}"
                    for k in cfg.feature_nodes) + " |")
        if "rti" in res:
            rows = [r for r in res["rti"] if "compare" in r]
            warm = [r for r in rows if r["step"] > 0]
            stat = np.array([r["rti_residuals"][0] for r in res["rti"]])
            lines += ["", f"RTI vs converged along {len(res['rti'])} closed-loop steps from s = {cfg.rti_s0} m "
                      f"({len(rows)} compared; step 0 is the cold start):", "",
                      "| quantity | median (warm steps) | worst warm step (max; min for cosines) | step 0 |", "|---|---|---|---|"]
            series = {"RTI stationarity residual": lambda r: r["rti_residuals"][0],
                      "‖u₀ RTI − u₀ SQP‖": lambda r: r["du0"],
                      **{f"rel error {o}": (lambda r, o=o: r["compare"][o]["rel_error"]) for o in ("x1", "u1", "u0")},
                      **{f"cosine {o}": (lambda r, o=o: r["compare"][o]["cosine"]) for o in ("x1", "u1")},
                      "SQP iterations (warm)": lambda r: r["sqp_iterations"]}
            for label, get in series.items():
                values = np.array([get(r) for r in warm]) if warm else np.array([np.nan])
                first = get(rows[0]) if rows and rows[0]["step"] == 0 else np.nan
                spec = ".6f" if label.startswith("cosine") else ".3g"
                extreme = np.min(values) if label.startswith("cosine") else np.max(values)  # the worst step
                lines.append(f"| {label} | {_fmt(np.median(values), spec)} | {_fmt(extreme, spec)} | "
                             f"{_fmt(first, spec)} |")
            statuses, counts = np.unique([r["rti_status"] for r in res["rti"]], return_counts=True)
            lines += ["", f"RTI status counts: {dict(zip(statuses.tolist(), counts.tolist()))}; "
                      f"largest stationarity residual {stat.max():.3g}."]
        if "batch" in res:
            worst = max(v["compare"][o]["rel_error"] for v in res["batch"].values() for o in ("x1", "u1", "u0", "value"))
            lines += ["", f"Batch ({len(res['batch'])} cases in one call) vs single: largest relative error {worst:.1e}; "
                      f"statuses {[v['status'] for v in res['batch'].values()]}."]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")
    _plot_sweep(results, tags, out_dir / "sweep.png")
    _plot_rti(results, tags, out_dir / "rti.png")


def _plot_sweep(results, tags, path):
    tags = [t for t in tags if "fd" in results[t]]
    if not tags:
        return
    fig, axes = plt.subplots(len(tags), 2, figsize=(10, 3.6 * len(tags)), squeeze=False, sharex=True, sharey=True)
    for row, tag in enumerate(tags):
        for col, output in enumerate(("x1", "u1")):
            ax = axes[row, col]
            for i, (name, r) in enumerate(results[tag]["fd"].items()):
                eps = sorted(r["sweep"])
                ax.loglog(eps, [r["sweep"][e]["compare"][output]["rel_error"] for e in eps], "-o", lw=2, ms=6,
                          color=SERIES[i], label=name)
            ax.set_title(f"∂{output[0]}₁/∂θ, track slack {tag.removeprefix('slack_')}", fontsize=10)
            _style(ax)
        axes[row, 0].set_ylabel("relative error, leap-c vs FD")
    axes[0, 1].legend(frameon=False, fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("relative finite-difference step ε")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_rti(results, tags, path):
    tags = [t for t in tags if "rti" in results[t]]
    if not tags:
        return
    panels = [("RTI stationarity\nresidual", lambda r: r["rti_residuals"][0]),
              ("rel. error ∂x₁/∂θ\nRTI vs converged", lambda r: r["compare"]["x1"]["rel_error"]),
              ("rel. error ∂u₁/∂θ\nRTI vs converged", lambda r: r["compare"]["u1"]["rel_error"])]
    fig, axes = plt.subplots(len(panels), 1, figsize=(8, 2.6 * len(panels)), sharex=True)
    for ax, (label, get) in zip(axes, panels):
        for i, tag in enumerate(tags):
            rows = [r for r in results[tag]["rti"] if "compare" in r]
            ax.semilogy([r["step"] for r in rows], [get(r) for r in rows], "-", lw=2, color=SERIES[i],
                        label=tag.removeprefix("slack_"))
        ax.set_ylabel(label, fontsize=8)
        _style(ax)
    axes[0].legend(frameon=False, fontsize=8, title="track slack", title_fontsize=8)
    axes[-1].set_xlabel("control step (0.02 s)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", choices=("fd", "rti", "batch"), default=None,
                    help="fd: 5.1-5.4, rti: 5.5, batch: 5.6 (default: all)")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data" / "leap_c_check")
    args = ap.parse_args()

    cfg = CheckConfig()
    theta = np.array(cfg.theta, dtype=float)
    ref = ReferenceTrack(compute_raceline(PARAMS / "agents.yaml", "agent_1", PARAMS / "min_curv_planner.yaml"))
    plant = Plant.from_scenario(PARAMS, reference=ref)
    results = {"cases": {}}

    loss = make_synthetic_loss(cfg.seed)

    for slack in cfg.slack_tracks:  # every check once per candidate track slack weighting
        tag = f"slack_{slack[0]:.0e}_{slack[1]:.0e}"
        run_cfg = replace(cfg, ocp=replace(cfg.ocp, slack_track=slack))
        res = results[tag] = {}
        sqp = build_diff_mpc(plant, run_cfg, nlp_solver="SQP", export_directory=args.out / f"acados_sqp_{tag}",
                             name=f"check_sqp_{tag.replace('-', 'm').replace('+', '')}")
        cases = make_test_cases(plant, run_cfg)
        results["cases"] = {c.name: {"description": c.description, "s0": c.s0} for c in cases}

        if args.only in (None, "fd"):
            res["fd"] = {}
            for case in cases:
                solution = solve(sqp, case, theta)
                if not _converged(solution):
                    print(f"{tag} {case.name}: solve failed (status {solution.status}), skipped")
                    continue
                case.active = active_set(plant, case, solution)
                leapc = leapc_sensitivities(sqp, case, theta, loss)
                res["fd"][case.name] = {
                    "solve": {"iterations": solution.iterations, "residuals": solution.residuals,
                              "value": solution.value},
                    "sweep": epsilon_sweep(sqp, case, theta, cfg.epsilons, loss, leapc=leapc),  # incl. fd_epsilon
                    "active": case.active,
                    "loss_terms": loss_term_gradients(leapc, solution, case, run_cfg),
                }
                best = best_epsilon(res["fd"][case.name]["sweep"])[1]
                print(f"{tag} {case.name}: active {case.active['active']}, rel err at best ε: "
                      f"x1 {best['x1']['rel_error']:.1e}, u1 {best['u1']['rel_error']:.1e}", flush=True)
        if args.only in (None, "rti"):
            rti = build_diff_mpc(plant, run_cfg, nlp_solver="SQP_RTI", export_directory=args.out / f"acados_rti_{tag}",
                                 name=f"check_rti_{tag.replace('-', 'm').replace('+', '')}")
            res["rti"] = rti_vs_converged(plant, rti, sqp, run_cfg)
        if args.only in (None, "batch"):
            res["batch"] = batch_vs_single(sqp, cases, theta)

    write_report(results, args.out, cfg)
    print(f"report: {args.out / 'report.md'}")


if __name__ == "__main__":
    main()
