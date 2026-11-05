# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Initial guesses for the Diff-WMPC solver (leap-c ``AcadosDiffMpcInitializer``).

leap-c's default initial iterate is all zeros, i.e. vx = 0 at every stage, where the MPC model's
slip angles atan2(., vx) have no derivative (the plant's vx -> 0 guard is not in the MPC model):
the first solve fails with NaN. :class:`ReferenceInitializer` starts every stage on its reference
point instead. It is used for a cold start (first solve, solver failure); afterwards the previous
solution (leap-c ``ctx``) warm-starts the solver.
"""

from copy import deepcopy

import numpy as np
from leap_c.diff_mpc.initializer import AcadosDiffMpcInitializer

from diff_weight_mpc.mpc.model import NU, NX


class ReferenceInitializer(AcadosDiffMpcInitializer):
    """Stage k = 0..N starts on the stage-k reference: X, Y, psi = x_ref, y_ref, psi_ref, vx = v_ref,
    vy = 0, r = a_lat,ref / v_ref, delta = atan(wheelbase · a_lat,ref / v_ref²) (kinematic), a = 0;
    stage 0 is the measured x0. Inputs zero, multipliers and slacks at acados' defaults.

    Args:
        ocp: the OCP passed to ``AcadosDiffMpcTorch`` (after ``manager.assign_to_ocp``).
        manager: its ``AcadosParameterManager`` (to find the stage references in p).
        wheelbase: for the kinematic steering angle.
    """

    def __init__(self, ocp, manager, wheelbase):
        self.default = ocp.create_default_initial_iterate().flatten()
        self.N = ocp.solver_options.N_horizon
        self.wheelbase = wheelbase
        indices = manager._non_differentiable_parameter_store.indices  # name -> (start, end) in p
        self.idx = {name: indices[name][0] for name in
                    ("x_ref", "y_ref", "psi_ref", "v_ref", "alat_ref")}

    def single_iterate(self, solver_input):
        it = deepcopy(self.default)
        p = solver_input.p_stagewise  # (N+1, np)
        x = np.zeros((self.N + 1, NX))
        if p is not None:
            v = np.maximum(p[:, self.idx["v_ref"]], 0.5)
            alat = p[:, self.idx["alat_ref"]]
            x[:, 0] = p[:, self.idx["x_ref"]]
            x[:, 1] = p[:, self.idx["y_ref"]]
            x[:, 2] = p[:, self.idx["psi_ref"]]
            x[:, 3] = v
            x[:, 5] = alat / v
            x[:, 6] = np.arctan2(self.wheelbase * alat, v**2)
        x[0] = solver_input.x0
        it.x = x.reshape(-1)
        it.u = np.zeros(self.N * NU)
        return it
