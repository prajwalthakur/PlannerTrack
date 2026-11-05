# Author: Prajwal Thakur <prajwalthakur98@gmail.com>

# Leap-c based model predictive control
# for accelerated learning .

from dataclasses import dataclass

import casadi as ca
import numpy as np
from acados_template import ACADOS_INFTY, AcadosOcp
from leap_c.parameters import AcadosParameterManager

from diff_weight_mpc.mpc.model import NU, NX, dynamics_expr, parameter_values
from plannertrack_py.utils import angle_diff

G = 9.81

# The paper's initial weights (Table I), tuned for a full-size racecar at up to 79 m/s. On our 1:10 car
# the jerk weight dominates (steps.md 4.1, mpc_ocp_lessons.md §4), so theta0 below is rescaled.
PAPER_THETA0 = (2.5, 2.9, 2.0, 5.0, 4.3, 6.8)  # q_n, q_psi, q_v, q_alat, r_j, r_omega
# theta0 for our car (steps.md 4.5, decided 2026-10-20): q_n x20 and r_j / 430 relative to the paper.
THETA0 = (50.0, 2.9, 2.0, 5.0, 0.01, 6.8)


@dataclass(frozen=True)
class OcpConfig:
    N: int = 34
    dt: float = 0.075
    theta0: tuple = THETA0  # q_n, q_psi, q_v, q_alat, r_j, r_omega
    rk4_steps: int = 4  # RK4 steps per interval (DISCRETE dynamics, see discrete_dynamics)
    nlp_solver: str = "SQP_RTI"
    max_iter: int = 100  # SQP iterations (nlp_solver="SQP")
    globalization: str = "FIXED_STEP"  # or "MERIT_BACKTRACKING" (line search)
    regularize_method: str = "MIRROR"  # forward solver only (exact Hessian may be indefinite)
    tol: float = 1e-6
    slack_track: tuple = (1e2, 1e3)  # (L1, L2) penalty on track-bound violation, fixed
    slack_friction: tuple = (1e2, 1e3)  # (L1, L2) penalty for the friction ellipse
    # MPC-only tightening of the track limits [m]: the MPC keeps the car's side this far inside the real
    # bounds (robustness to model mismatch near the edge; steps.md 4.9). The plant's off_track flag and the
    # metrics keep the real limits. Applied in reference_params.stage_parameters (the limits are stage
    # parameters, so changing it needs no recompile).
    track_margin: float = 0.03


# 1. parameters
ERROR_NAMES = ("n", "e_psi", "e_v", "e_alat")
THETA_NAMES = ("q_n", "q_psi", "q_v", "q_alat", "r_j", "r_omega")
STAGE_REF_NAMES = ("x_ref", "y_ref", "psi_ref", "v_ref", "alat_ref", "n_left", "n_right")
IDX_STEER, IDX_ACC = 6, 7
IDX_JERK, IDX_STEER_RATE = 0, 1


def register_parameters(manager, vp, cfg) -> dict:
    """
    theta (6, differentiable, global) (learnable): what the policy outputs, shape (B, 6).
    vehicle (5, non-diff): p = parameter_values(vp) in PARAMETER_NAMES order (nominal; oracle = true).
    one entry per STAGE_REF_NAMES (non-diff): set per stage at solve time, shape (B, N+1, 1).
    Returns {name: casadi symbol}.
    """
    sym = {
        "theta": manager.register_parameter("theta", np.array(cfg.theta0), differentiable=True),
        "vehicle": manager.register_parameter("vehicle", parameter_values(vp)),  # non-diff
    }
    for name in STAGE_REF_NAMES:  # non-diff, one value per stage at solve time
        sym[name] = manager.register_parameter(name, np.zeros(1))
    return sym


# 2. tracking errors
def tracking_errors(x, ref) -> dict:
    """
    ocp tracking erros:
    n      = -sin(psi_ref)·(X - x_ref) + cos(psi_ref)·(Y - y_ref)   (lateral only)
    e_psi  = wrap(psi - psi_ref)                                    (body heading error, utils.angle_diff)
    e_v    = vx - v_ref
    e_alat = vx·r - alat_ref
    """
    X, Y, psi, vx, r = x[0], x[1], x[2], x[3], x[5]
    dx = X - ref["x_ref"]
    dy = Y - ref["y_ref"]
    return {
        "n": -ca.sin(ref["psi_ref"]) * dx + ca.cos(ref["psi_ref"]) * dy,
        "e_psi": angle_diff(psi, ref["psi_ref"]),
        "e_v": vx - ref["v_ref"],
        "e_alat": vx * r - ref["alat_ref"],
    }


#  3. cost
def weights(theta) -> dict:
    """theta (6,) -> {name: entry}, in THETA_NAMES order. theta is one CasADi vector (the
    differentiable parameter), so it is indexed by position, not by name."""
    return {name: theta[i] for i, name in enumerate(THETA_NAMES)}


def terminal_cost(err, theta):
    """½(q_n n² + q_ψ e_ψ² + q_v e_v² + q_alat e_alat²): the four state terms (stage N has no input)."""
    w = weights(theta)
    return 0.5 * (
        w["q_n"] * err["n"] ** 2
        + w["q_psi"] * err["e_psi"] ** 2
        + w["q_v"] * err["e_v"] ** 2
        + w["q_alat"] * err["e_alat"] ** 2
    )


def stage_cost(err, u, theta):
    """½(q_n n² + q_ψ e_ψ² + q_v e_v² + q_alat e_alat² + r_j j² + r_ω ω²): the state terms plus the
    input effort (jerk, steering rate)."""
    w = weights(theta)
    return terminal_cost(err, theta) + 0.5 * (
        w["r_j"] * u[IDX_JERK] ** 2 + w["r_omega"] * u[IDX_STEER_RATE] ** 2
    )


# 4. constraints
def set_bounds(ocp, limits):
    """
    Hard constraints on control and state
    u = [j, ω]:  |j| <= max_jerk,  |ω| <= max_steer_rate
    x:           |δ| <= max_steer,  -max_decel <= a <= max_accel
    """
    c = ocp.constraints
    # constraints on u
    c.idxbu = np.array([IDX_JERK, IDX_STEER_RATE])
    c.lbu = np.array([-limits.max_jerk, -limits.max_steer_rate])
    c.ubu = np.array([limits.max_jerk, limits.max_steer_rate])

    # constraints on x
    c.idxbx = np.array([IDX_STEER, IDX_ACC])
    c.lbx = np.array([-limits.max_steer, -limits.max_decel])
    c.ubx = np.array([limits.max_steer, limits.max_accel])

    # terminal state
    c.idxbx_e, c.lbx_e, c.ubx_e = c.idxbx.copy(), c.lbx.copy(), c.ubx.copy()

    # placeholder; leap-c sets x0 at every solve
    c.x0 = np.zeros(ocp.model.x.rows())


def path_constraints(x, ref, vehicle, limits, mu_nominal):
    """Soft constraints , h ≤ 0:
            n - n_left,  n_right - n,
            (a/max_decel)² + (vx·r / a_y,max(μ))² - 1  (friction ellipse, exponent 2)
    a_y,max(μ) = limits.ay_max · μ / μ_nominal = friction_margin·μ·g, with the margin taken from the
    planner's limits (one source). μ is the vehicle parameter, so the oracle's (model which is exactly
    same as sim model) ellipse uses the true μ.
    Returns h (3,); used for con_h_expr and con_h_expr_e."""

    mu = vehicle[1]  # Parameters_NAMES = (mass,mu,Iz,cf,cr)
    n = tracking_errors(x, ref)["n"]
    vx, r, a = x[3], x[5], x[IDX_ACC]
    ay_max = limits.ay_max * mu / mu_nominal
    return ca.vertcat(
        n - ref["n_left"],
        ref["n_right"] - n,
        (a / limits.max_decel) ** 2 + (vx * r / ay_max) ** 2 - 1.0,
    )


def set_path_constraints(ocp, h, cfg):
    """h <= 0 on stages 1..N-1 and N,
    every row soft: violation s >= 0 costs z·s + ½·Z·s².
    Track rows use cfg.slack_track, the ellipse cfg.slack_friction (fixed numbers: acados cannot
    make slack weights parameters)."""
    nh = h.rows()
    z = np.array([cfg.slack_track[0]] * 2 + [cfg.slack_friction[0]])  # l1
    Z = np.array([cfg.slack_track[1]] * 2 + [cfg.slack_friction[1]])  # l2

    ocp.model.con_h_expr = h
    ocp.constraints.lh = np.full(nh, -ACADOS_INFTY)
    ocp.constraints.uh = np.zeros(nh)
    ocp.constraints.idxsh = np.arange(nh)
    # acados gives every soft row a lower AND an upper slack. Only the upper side (h <= 0) can be
    # violated, but the lower slack still needs a positive weight: a slack with zero cost has no unique
    # value, and that degeneracy stalls the interior-point QP's complementarity (SQP never converged).
    ocp.cost.zl = z
    ocp.cost.zu = z
    ocp.cost.Zl = Z
    ocp.cost.Zu = Z

    # same constraints at the terminal stage
    ocp.model.con_h_expr_e = h
    ocp.constraints.lh_e = np.full(nh, -ACADOS_INFTY)
    ocp.constraints.uh_e = np.zeros(nh)
    ocp.constraints.idxsh_e = np.arange(nh)
    ocp.cost.zl_e, ocp.cost.zu_e = z, z
    ocp.cost.Zl_e, ocp.cost.Zu_e = Z, Z


# ---------------------------------------------------------------- 5. solver options
def set_solver_options(ocp, cfg):
    """N, tf = N·dt, DISCRETE dynamics, SQP_RTI (paper) or SQP, PARTIAL_CONDENSING_HPIPM,
    EXACT Hessian. The forward solver regularizes the (possibly indefinite) exact Hessian; leap-c builds
    the separate sensitivity solver with the exact Hessian and no regularization itself."""
    o = ocp.solver_options
    o.N_horizon = cfg.N
    o.tf = cfg.N * cfg.dt  # 34 × 0.075 = 2.55 s
    o.integrator_type = "DISCRETE"  # model.disc_dyn_expr (acados' parameter
    # sensitivities need discrete dynamics)
    o.nlp_solver_type = cfg.nlp_solver
    o.nlp_solver_max_iter = cfg.max_iter  # SQP only; RTI does one iteration per call
    o.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    o.hessian_approx = "EXACT"
    o.regularize_method = cfg.regularize_method
    o.globalization = cfg.globalization
    o.nlp_solver_tol_stat = o.nlp_solver_tol_eq = cfg.tol
    o.nlp_solver_tol_ineq = o.nlp_solver_tol_comp = cfg.tol
    # the QP solver must resolve complementarity below the NLP tolerance, or SQP stalls just above it
    o.qp_solver_tol_stat = o.qp_solver_tol_eq = o.qp_solver_tol_ineq = 0.1 * cfg.tol
    o.qp_solver_tol_comp = 0.1 * cfg.tol


def discrete_dynamics(f, x, u, dt, steps):
    """x_{k+1} as a CasADi expression: ``steps`` RK4 steps of dt/steps on xdot = f(x, u).

    acados computes solution sensitivities w.r.t. parameters only for DISCRETE dynamics, so the
    integrator is written here instead of acados' ERK; numerically it is the same RK4 (steps.md 2.6:
    4 steps per 0.075 s interval). ``f(x, u)`` returns the CasADi expression of xdot (it may contain
    parameter symbols, e.g. the vehicle parameters).
    """
    h = dt / steps
    for _ in range(steps):
        k1 = f(x, u)
        k2 = f(x + 0.5 * h * k1, u)
        k3 = f(x + 0.5 * h * k2, u)
        k4 = f(x + h * k3, u)
        x = x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return x


# ---------------------------------------------------------------- 6. assembly
def build_ocp(vp, limits, cfg=OcpConfig(), name="diff_wmpc"):
    """The Diff-WMPC OCP and its parameter manager (not yet compiled).

    Parameters (register_parameters): theta (differentiable), vehicle and the stage references
    (non-differentiable). The model is mpc.model's dynamics with the vehicle parameters as symbols,
    discretized by discrete_dynamics (RK4, rk4_steps per dt); the
    cost is EXTERNAL (theta inside it); constraints from block 4. ocp.model.p / p_global are NOT set here:
    AcadosDiffMpcTorch(ocp, manager) calls manager.assign_to_ocp(ocp).

    Returns ``(ocp, manager)``.
    """
    ocp = AcadosOcp()
    manager = AcadosParameterManager(N_horizon=cfg.N)
    sym = register_parameters(manager, vp, cfg)

    x, u = ca.SX.sym("x", NX), ca.SX.sym("u", NU)

    def f(xk, uk):
        return dynamics_expr(xk, uk, vp, p=sym["vehicle"])

    m = ocp.model
    m.name, m.x, m.u = name, x, u
    m.disc_dyn_expr = discrete_dynamics(f, x, u, cfg.dt, cfg.rk4_steps)

    ref = {key: sym[key] for key in STAGE_REF_NAMES}
    err = tracking_errors(x, ref)
    ocp.cost.cost_type = ocp.cost.cost_type_e = "EXTERNAL"
    m.cost_expr_ext_cost = stage_cost(err, u, sym["theta"])
    m.cost_expr_ext_cost_e = terminal_cost(err, sym["theta"])

    set_bounds(ocp, limits)  # needs m.x (size of the x0 placeholder)
    set_path_constraints(ocp, path_constraints(x, ref, sym["vehicle"], limits, vp.mu), cfg)
    set_solver_options(ocp, cfg)
    return ocp, manager


# ---------------------------------------------------------------- 7. numeric helpers (tests, loss)
def error_function() -> ca.Function:
    """tracking_errors as a CasADi Function ``(x (NX,), ref (7,)) -> [n, e_psi, e_v, e_alat]``, with ref
    in STAGE_REF_NAMES order; to check the OCP's features numerically against Plant.step's info."""
    x = ca.SX.sym("x", NX)
    r = ca.SX.sym("ref", len(STAGE_REF_NAMES))
    err = tracking_errors(x, {key: r[i] for i, key in enumerate(STAGE_REF_NAMES)})
    return ca.Function("tracking_errors", [x, r], [ca.vertcat(*(err[k] for k in ERROR_NAMES))])
