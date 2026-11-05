# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Small helpers shared by the plant, the reference and the OCP, so every part of the stack does the
same calculation.

Angles: one convention everywhere, ``atan2(sin a, cos a)`` in [-pi, pi] (exactly +-pi can come out
either way, since sin(pi) is not exactly 0 in floating point). The same formula works on
numpy values and CasADi expressions; in the OCP it is smooth (d/da = 1) except at exactly +-pi, which
a car inside the heading limits (Plant: +-60 deg) never reaches. casadi is imported only for CasADi
inputs, so the plant does not depend on it.
"""

import numpy as np


def _is_casadi(a):
    return type(a).__module__.startswith("casadi")


def wrap_angle(a):
    """Angle(s) mapped into [-pi, pi]. numpy scalar/array in -> numpy out; CasADi in -> CasADi out."""
    if _is_casadi(a):
        import casadi as ca
        return ca.atan2(ca.sin(a), ca.cos(a))
    a = np.asarray(a, dtype=float)
    return np.arctan2(np.sin(a), np.cos(a))


def angle_diff(a, b):
    """Signed angle from ``b`` to ``a``: wrap(a - b) in [-pi, pi]. E.g. heading error
    ``angle_diff(psi, psi_ref)``."""
    return wrap_angle(a - b)
