# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""One angle convention for the stack (plannertrack_py.utils): numpy and CasADi give the same numbers."""

import casadi as ca
import numpy as np
import pytest

from plannertrack_py.utils import angle_diff, wrap_angle


def test_range_and_values():
    a = np.array([0.0, 0.1, -0.1, np.pi, -np.pi, 3 * np.pi, 2 * np.pi + 0.2, -2 * np.pi - 0.2, 7.0, -7.0])
    w = wrap_angle(a)
    assert np.all(np.abs(w) <= np.pi)
    np.testing.assert_allclose(np.cos(w), np.cos(a), atol=1e-12)
    np.testing.assert_allclose(np.sin(w), np.sin(a), atol=1e-12)
    assert abs(wrap_angle(np.pi)) == pytest.approx(np.pi)  # the boundary: +pi or -pi
    assert wrap_angle(2 * np.pi + 0.2) == pytest.approx(0.2)


def test_angle_diff_crosses_pi():
    assert angle_diff(np.pi - 0.1, -np.pi + 0.1) == pytest.approx(-0.2)
    assert angle_diff(-np.pi + 0.1, np.pi - 0.1) == pytest.approx(0.2)
    assert angle_diff(0.3 + 4 * np.pi, 0.1) == pytest.approx(0.2)


def test_casadi_matches_numpy_and_is_smooth():
    a, b = ca.SX.sym("a"), ca.SX.sym("b")
    e = angle_diff(a, b)
    f = ca.Function("f", [a, b], [e, ca.jacobian(e, ca.vertcat(a, b))])
    rng = np.random.default_rng(0)
    for x, y in rng.uniform(-10.0, 10.0, size=(50, 2)):
        val, jac = f(x, y)
        assert float(val) == pytest.approx(float(angle_diff(x, y)), abs=1e-12)
        np.testing.assert_allclose(np.asarray(jac).ravel(), [1.0, -1.0], atol=1e-12)
