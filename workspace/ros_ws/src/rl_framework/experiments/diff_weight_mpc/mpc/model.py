# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""MPC prediction model: the augmented single-track dynamics in CasADi (steps.md Step 2.3).

STATUS: implemented; equal to the C++ plant to ~1e-14 for vx >= MIN_VALID_SPEED
(test/test_mpc_model.py).

Same equations as the C++ plant ``SingleTrackAugDynModel::xdot()``
(``motion_model_ground_vehicles/src/singleTrackAugDynModel/SingleTrackAugDynModel.cpp``) in its
dynamic regime, written symbolically so acados can differentiate it and generate code. Differences
from the C++ model, all removing non-smooth points (exact Hessian / leap-c sensitivities need a
smooth model):

========================================  ==================  =====================================
C++ part                                  here                why that is safe
========================================  ==================  =====================================
low-speed kinematic blend (0.5-2.5 m/s)   dropped             valid for vx >= MIN_VALID_SPEED; the
                                                              reference's slowest point is 2.954 m/s
                                                              (hairpin apex)
``vx_safe`` guard against vx -> 0          dropped             only active below 1e-3 m/s
``max(F_z, F_min)`` axle-load floor        dropped             the front axle unloads only above
                                                              a = g*lr/hcg ~ 22.5 m/s^2, far beyond
                                                              max_decel = 8.64 m/s^2
========================================  ==================  =====================================

State and input (same layout as the C++ plugin)::

    x = [X, Y, psi, vx, vy, r, delta, a]      u = [jerk, delta_dot]

Dynamics (L = lf + lr, g = 9.81)::

    vx_dot  = a + vy*r
    Fzf     = m*(g*lr - vx_dot*hcg)/L,      Fzr = m*(g*lf + vx_dot*hcg)/L
    alpha_f = delta - atan2(vy + lf*r, vx), alpha_r = -atan2(vy - lr*r, vx)
    Fyf     = mu*Fzf * tanh(cf*alpha_f),    Fyr = mu*Fzr * tanh(cr*alpha_r)
              (C++ writes Fy_max*tanh(K*alpha/Fy_max) with K = mu*C*Fz, Fy_max = mu*Fz: the same)
    vy_dot  = (Fyf*cos(delta) + Fyr)/m - vx*r
    r_dot   = (lf*Fyf*cos(delta) - lr*Fyr)/Iz
    X_dot   = vx*cos(psi) - vy*sin(psi),    Y_dot = vx*sin(psi) + vy*cos(psi),   psi_dot = r
    delta_dot = u[1],   a_dot = u[0] (jerk)

The model parameters PARAMETER_NAMES = (mass, mu, Iz, cf, cr) can be passed as symbols, so one
compiled solver takes their values at run time: the nominal values for the MPC, the true (perturbed)
ones for an oracle (Step 8). lf, lr and hcg are always constants from :class:`VehicleParams`.
"""

import casadi as ca
import numpy as np

from plannertrack_py.vehicle import VehicleParams

NX = 8
NU = 2
STATE_NAMES = ("x", "y", "yaw", "vx", "vy", "yaw_rate", "steer", "acc")
INPUT_NAMES = ("jerk", "steer_rate")

# The C++ plant blends linearly from the kinematic model (vx <= 0.5 m/s) to the dynamic one
# (vx >= 2.5 m/s). This model is the dynamic one, so it equals the plant from 2.5 m/s on. (The band
# was 1-3 m/s; moved down so the model is exact at the hairpin apex, R = 1.03 m, v_ref = 2.954 m/s,
# instead of differing by the blend's 2.3 % kinematic share (~1 % in the steady steer). The dynamic
# model cannot hold that radius at >= 3 m/s at all (max 2.986 m/s): steps.md §5.)
MIN_VALID_SPEED = 2.5  # m/s

G = 9.81  # m/s^2, as in the C++ plugin

# Model parameters that can be symbols (the MPC's parameter vector p, in this order) and that the
# plant can perturb (plannertrack_py.plant.PERTURBABLE is the same tuple).
PARAMETER_NAMES = ("mass", "mu", "Iz", "cf", "cr")


def parameter_values(params: VehicleParams):
    """``p`` for the model: the values of PARAMETER_NAMES in ``params``, shape (5,)."""
    return np.array([float(getattr(params, name)) for name in PARAMETER_NAMES])


def dynamics_expr(x, u, params: VehicleParams, p=None, **overrides):
    """Symbolic state derivative dx/dt = f(x, u).

    Args:
        x: state, CasADi SX/MX of size NX.
        u: input, CasADi SX/MX of size NU.
        params: vehicle parameters; every value not given by ``p`` / ``overrides`` is a constant
            from here (lf, lr, hcg always).
        p: optional parameter vector (size 5) in the order of PARAMETER_NAMES, e.g. a CasADi symbol.
        **overrides: single parameters by name (mass, mu, Iz, cf, cr), e.g. ``mass=sym``.

    Returns:
        CasADi expression of size NX, same layout as ``x``.
    """
    values = {name: getattr(params, name) for name in PARAMETER_NAMES}
    if p is not None:
        values.update({name: p[i] for i, name in enumerate(PARAMETER_NAMES)})
    unknown = set(overrides) - set(PARAMETER_NAMES)
    if unknown:
        raise ValueError(f"unknown model parameters {sorted(unknown)} (have {PARAMETER_NAMES})")
    values.update({k: v for k, v in overrides.items() if v is not None})

    psi, vx, vy, r, delta, a = x[2], x[3], x[4], x[5], x[6], x[7]
    jerk, delta_dot = u[0], u[1]
    m_, mu_, Iz, cf, cr = (values[name] for name in PARAMETER_NAMES)
    lf, lr, hcg = params.lf, params.lr, params.hcg
    L = lf + lr
    vx_dot = a + vy*r
    # 3. axle loads with longitudinal load transfer (C++ clamps at F_min; only reached
    #    above ~22.5 m/s^2 of braking -> dropped)
    fzf = m_ * (G * lr - vx_dot * hcg) / L
    fzr = m_ * (G * lf + vx_dot * hcg) / L
    alpha_f = delta - ca.atan2(vy + lf * r, vx)
    alpha_r = -ca.atan2(vy - lr * r, vx)
    # 5. friction-limited tire forces. C++: Fy_max * tanh(K*alpha / Fy_max) with
    #    K = mu*C*Fz and Fy_max = mu*Fz, i.e. mu*Fz*tanh(C*alpha): same value, no division   
    fyf = mu_ * fzf * ca.tanh(cf * alpha_f)
    fyr = mu_ * fzr * ca.tanh(cr * alpha_r)
    vy_dot = (fyf * ca.cos(delta) + fyr) / m_ - vx * r
    r_dot = (lf * fyf * ca.cos(delta) - lr * fyr) / Iz
    return ca.vertcat(
        vx * ca.cos(psi) - vy * ca.sin(psi),   # X_dot
        vx * ca.sin(psi) + vy * ca.cos(psi),   # Y_dot
        r,                                     # psi_dot
        vx_dot,                                # vx_dot
        vy_dot,                                # vy_dot
        r_dot,                                 # r_dot
        delta_dot,                             # delta_dot = steer rate
        jerk,                                  # a_dot = jerk
    )


def dynamics_function(params: VehicleParams):
    """CasADi Function ``f(x, u, p) -> xdot`` with p = :func:`parameter_values` (order of
    PARAMETER_NAMES); lf, lr, hcg from ``params``. For evaluation and tests (Step 2.4)."""
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    p = ca.SX.sym("p", len(PARAMETER_NAMES))
    return ca.Function("f_single_track_aug", [x, u, p], [dynamics_expr(x, u, params, p=p)],
                       ["x", "u", "p"], ["xdot"])
