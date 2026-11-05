# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""The prediction model as an acados model (steps.md Steps 2.6 and 4).

acados discretizes the continuous model ``f_expl_expr`` itself (ERK = explicit Runge-Kutta, RK4 with
4 stages) and generates C code for it and its derivatives. The model parameters are
``p = [mass, mu, Iz, cf, cr]`` (PARAMETER_NAMES), so one compiled solver serves the nominal model, the
oracle (the true, perturbed values) and parameter sweeps without regenerating code.

In the Diff-WMPC OCP (Step 4) the parameter vector may instead come from leap-c's
AcadosParameterManager; :func:`dynamics_expr` is the shared piece in both cases.
"""

import casadi as ca
from acados_template import AcadosModel

from diff_weight_mpc.mpc.model import (  # noqa: F401  (parameter_values re-exported)
    INPUT_NAMES,
    NU,
    NX,
    PARAMETER_NAMES,
    STATE_NAMES,
    dynamics_expr,
    parameter_values,
)
from plannertrack_py.vehicle import VehicleParams


def export_acados_model(params: VehicleParams, name="single_track_aug"):
    """AcadosModel of the augmented single-track dynamics with ``p = [mass, mu, Iz, cf, cr]``.

    Explicit form ``f_expl_expr`` (for ERK) and the matching implicit form
    ``f_impl_expr = xdot - f`` (for implicit integrators) are both set.
    """
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    p = ca.SX.sym("p", len(PARAMETER_NAMES))
    xdot = ca.SX.sym("xdot", NX)
    f_expl = dynamics_expr(x, u, params, p=p)

    model = AcadosModel()
    model.name = name
    model.x = x
    model.u = u
    model.p = p
    model.xdot = xdot
    model.f_expl_expr = f_expl        # explicit rk model
    model.f_impl_expr = xdot - f_expl # implicit model 
    model.x_labels = list(STATE_NAMES)
    model.u_labels = list(INPUT_NAMES)
    return model
