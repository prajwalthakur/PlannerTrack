# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Reference queries, validation and plotting for a closed raceline.

steps.md §3.4(6)-(7): the MPC and the weight policy query the raceline by
arc length, not by time, so falling behind schedule is not penalized as path
error.

Conventions: ``s`` in [0, length) on the closed loop; heading ``psi`` = 0
along +x, returned continuous (unwrapped) so it can be differenced across the
start/finish point; lateral offset ``n`` > 0 to the LEFT of the reference,
so the track bounds are ``-w_right <= n <= w_left``.

Example::

    from plannertrack_py.raceline import compute_raceline
    from plannertrack_py.reference import ReferenceTrack
    ref = ReferenceTrack(compute_raceline(agents, "agent_1", planner))
    proj = ref.project(x, y)                     # s0, lateral error n
    stages = ref.stage_references(proj.s, n_stages=25, dt=0.1)
    v_ahead, kappa_ahead = ref.policy_lookahead(proj.s)
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from plannertrack_py import _core
from plannertrack_py.config import load_agent_yaml
from plannertrack_py.utils import angle_diff
from plannertrack_py.raceline import Raceline, load_planner_config

_FIELDS = ("x", "y", "kappa", "vx", "ax", "w_right", "w_left")


# --------------------------------------------------------------------------- limits


@dataclass(frozen=True)
class RacelineLimits:
    """Limits used by the optimizer, as computed by the C++ parameter code."""

    curvature_limit: float  # tan(max_steer) / wheelbase [rad/m]
    ay_max: float  # friction_margin * mu * g [m/s^2]
    max_decel: float  # longitudinal tire grip: braking, and via the grip budget also acceleration [m/s^2]
    max_accel: float  # drive (motor) limit, accelerating only [m/s^2]
    v_max: float  # [m/s]
    max_steer: float  # steering angle bound [rad]
    max_steer_rate: float  # steering-rate bound [rad/s] (MPC input bound)
    max_jerk: float  # jerk bound [m/s^3] (MPC input bound)
    dyn_model_exp: float  # friction-ellipse exponent p
    optimization_width: float  # vehicle width + 2 * safety margin [m]
    vehicle_width: float  # [m]


def load_raceline_limits(agents_yaml_path, agent_name, planner_yaml_path):
    """Limits for ``agent_name`` and the ``min_curv_planner`` block of the planner config."""
    d = _core.raceline_limits(
        load_agent_yaml(agents_yaml_path, agent_name),
        yaml.safe_dump(load_planner_config(planner_yaml_path)),
    )
    return RacelineLimits(**d)


# ------------------------------------------------------------------------ queries


@dataclass(frozen=True)
class Projection:
    """Closest point of the reference to a position."""

    s: float  # arc length of the foot point, in [0, length) [m]
    n: float  # signed lateral offset, > 0 left of the reference [m]
    psi: float  # reference heading at s (continuous) [rad]
    index: int  # index of the reference segment containing the foot point


class ReferenceTrack:
    """A closed raceline queried by arc length (linear interpolation between samples)."""

    def __init__(self, raceline):
        self.raceline = raceline
        self.length = float(raceline.length)
        n = len(raceline)
        # knots including the closing point s = length (values wrap to point 0)
        self._s = np.append(raceline.s, self.length)
        self._values = {k: np.append(getattr(raceline, k), getattr(raceline, k)[0]) for k in _FIELDS}
        # continuous heading over one lap; one lap adds total_turn (about +-2 pi)
        self._psi = np.unwrap(np.append(raceline.psi, raceline.psi[0]))
        self.total_turn = self._psi[-1] - self._psi[0]
        # segments of the closed polyline for projection
        self._xy = raceline.xy
        self._seg = np.roll(self._xy, -1, axis=0) - self._xy
        self._seg_len = np.linalg.norm(self._seg, axis=1)
        # arc length per segment (spline arc length, slightly longer than the chord):
        # a foot point at fraction tau of segment i is at s_i + tau * ds_i, consistent
        # with sample()
        self._ds = np.diff(self._s)
        self._n = n

    @classmethod
    def from_npz(cls, path):
        """Load a reference written by :meth:`Raceline.save_npz`."""
        data = np.load(path)
        names = Raceline.__dataclass_fields__
        kwargs = {k: (data[k] if data[k].ndim else data[k].item()) for k in names}
        return cls(Raceline(**kwargs))

    def __len__(self):
        return self._n

    def wrap(self, s):
        """Arc length mapped into [0, length)."""
        return np.mod(s, self.length)

    def sample(self, s):
        """Reference values at arc length(s) ``s`` (any real values; the loop repeats).

        Returns a dict of arrays: x, y, psi (continuous), kappa, vx, ax, a_lat, w_right, w_left.
        """
        s = np.asarray(s, dtype=float)
        laps = np.floor(s / self.length)
        sw = s - laps * self.length
        out = {k: np.interp(sw, self._s, v) for k, v in self._values.items()}
        out["psi"] = np.interp(sw, self._s, self._psi) + laps * self.total_turn
        out["a_lat"] = out["vx"] ** 2 * out["kappa"]
        return out

    def project(self, x, y, s_guess=None, window=None):
        """Project a position onto the reference.

        Without ``s_guess`` the global nearest point is used. With ``s_guess`` and
        ``window`` [m], only segments within ``window`` of ``s_guess`` (around the loop)
        are considered (paper §3.6: search around the expected position s + v dt), which
        avoids jumping to a nearby part of the track.
        """
        p = np.array([x, y], dtype=float)
        d = p - self._xy
        tau = np.clip(np.einsum("ij,ij->i", d, self._seg) / self._seg_len**2, 0.0, 1.0)
        foot = self._xy + tau[:, None] * self._seg
        dist = np.linalg.norm(p - foot, axis=1)
        if s_guess is not None and window is not None:
            s_mid = self.raceline.s + 0.5 * self._ds
            gap = np.abs((s_mid - s_guess + 0.5 * self.length) % self.length - 0.5 * self.length)
            dist = np.where(gap <= window, dist, np.inf)
            if not np.isfinite(dist).any():
                raise ValueError("no reference segment within the search window")
        i = int(np.argmin(dist))
        s = float(self.wrap(self.raceline.s[i] + tau[i] * self._ds[i]))
        t = self._seg[i] / self._seg_len[i]
        rel = p - foot[i]
        n = float(t[0] * rel[1] - t[1] * rel[0])  # cross(t, rel): > 0 to the left
        return Projection(s=s, n=n, psi=float(self.sample(s)["psi"]), index=i)

    def stage_references(self, s0, n_stages, dt):
        """N + 1 stage references for the MPC, indexed by arc length.

        s_0 = s0, s_{k+1} = s_k + v_ref(s_k) * dt. Arc length is NOT wrapped (it keeps
        increasing across start/finish) and psi is continuous. Returns sample(s) plus
        "s" (continuous) and "t" (stage times).
        """
        s = np.empty(n_stages + 1)
        s[0] = s0
        for k in range(n_stages):
            s[k + 1] = s[k] + float(self.sample(s[k])["vx"]) * dt
        out = self.sample(s)
        out["s"] = s
        out["t"] = dt * np.arange(n_stages + 1)
        return out

    def policy_lookahead(self, s0, n_samples=5, horizon=2.55, dt=0.01):
        """Reference speed and curvature at ``n_samples`` future times over ``horizon`` [s].

        Times (j + 1) * horizon / n_samples, j = 0..n-1 (the last one at the horizon),
        reached by integrating ds/dt = v_ref(s) from s0 with step ``dt``. This is the
        Diff-WMPC policy input (5 speeds, 5 curvatures, 2.55 s).
        """
        times = (np.arange(n_samples) + 1) * horizon / n_samples
        steps = int(np.ceil(horizon / dt))
        stages = self.stage_references(s0, steps, horizon / steps)
        s_at = np.interp(times, stages["t"], stages["s"])
        v = self.sample(s_at)
        return v["vx"], v["kappa"]

    def speed(self, s):
        """v_ref at arc length(s) ``s`` (any shape, laps repeat); the cheap single-field lookup of
        :meth:`sample`."""
        s = np.asarray(s, dtype=float)
        return np.interp(np.mod(s, self.length), self._s, self._values["vx"])

    def policy_observation(self, s0, n_samples=5, horizon=2.55, dt=0.01):
        """:meth:`policy_lookahead` for B cars at once: ``(v (B, n), kappa (B, n))``.

        Same construction (ds/dt = v_ref integrated with step horizon/ceil(horizon/dt), values at times
        (j + 1)·horizon/n), one vectorized step for all cars."""
        s0 = np.atleast_1d(np.asarray(s0, dtype=float))
        steps = int(np.ceil(horizon / dt))
        h = horizon / steps
        S = np.empty((len(s0), steps + 1))
        S[:, 0] = s0
        for k in range(steps):
            S[:, k + 1] = S[:, k] + self.speed(S[:, k]) * h
        t = h * np.arange(steps + 1)
        times = (np.arange(n_samples) + 1) * horizon / n_samples
        s_at = np.stack([np.interp(times, t, row) for row in S])
        v = self.sample(s_at)
        return v["vx"], v["kappa"]


# --------------------------------------------------------------------- validation


@dataclass(frozen=True)
class Check:
    name: str
    value: float
    limit: float
    passed: bool
    severity: str  # "error" or "warning"
    detail: str


@dataclass(frozen=True)
class ValidationReport:
    checks: tuple

    @property
    def passed(self):
        """True if no check with severity "error" failed (warnings do not fail)."""
        return all(c.passed for c in self.checks if c.severity == "error")

    def format(self):
        lines = []
        for c in self.checks:
            tag = "PASS" if c.passed else ("FAIL" if c.severity == "error" else "WARN")
            lines.append(f"[{tag}] {c.name}: {c.value:.4g} (limit {c.limit:.4g}) {c.detail}")
        lines.append("RESULT: " + ("PASSED" if self.passed else "FAILED"))
        return "\n".join(lines)


def _step_lengths(rl):
    return np.diff(np.append(rl.s, rl.length))


def _accel_violation(rl, lim):
    """Worst |a_x| beyond the available longitudinal grip over all steps (closed loop).

    Uses the larger budget of the two step ends, as the forward/backward passes do.
    """
    n = len(rl)
    a_y = np.abs(rl.vx**2 * rl.kappa)
    r = np.clip(1.0 - (a_y / lim.ay_max) ** lim.dyn_model_exp, 0.0, None)
    budget = lim.max_decel * r ** (1.0 / lim.dyn_model_exp)
    budget = np.maximum(budget, np.roll(budget, -1))
    limit = np.where(rl.ax >= 0.0, np.minimum(budget, lim.max_accel), budget)
    excess = np.abs(rl.ax) - limit
    k = int(np.argmax(excess))
    return float(excess[k]), k if n else 0


def validate_reference(rl, lim, low_speed=2.5):
    """steps.md §3.4(6) checks on a raceline. Returns a :class:`ValidationReport`."""
    checks = []

    def add(name, value, limit, passed, detail="", severity="error"):
        checks.append(Check(name, float(value), float(limit), bool(passed), severity, detail))

    arrays = [getattr(rl, k) for k in ("s", "x", "y", "psi", "kappa", "vx", "ax", "w_right", "w_left")]
    add("finite values", sum(int(~np.isfinite(a).all()) for a in arrays), 0,
        all(np.isfinite(a).all() for a in arrays))
    add("IQP converged", rl.iqp_iterations, 0, rl.iqp_converged,
        f"({rl.iqp_iterations} iterations, error {rl.iqp_curv_error_max:.4f} rad/m)")

    kmax = np.abs(rl.kappa).max()
    add("max |kappa| <= curvature limit [rad/m]", kmax, lim.curvature_limit,
        kmax <= 1.05 * lim.curvature_limit, "(5 % tolerance: the QP limits the linearized curvature)")
    aymax = np.abs(rl.a_lat).max()
    add("max |a_lat| <= ay_max [m/s^2]", aymax, lim.ay_max, aymax <= lim.ay_max * (1 + 1e-6))
    add("max v <= v_max [m/s]", rl.vx.max(), lim.v_max, rl.vx.max() <= lim.v_max + 1e-9)
    add("max_decel <= ay_max (braking within the lateral grip) [m/s^2]", lim.max_decel, lim.ay_max,
        lim.max_decel <= lim.ay_max + 1e-9,
        "(an elliptic g-g diagram with a longer braking axis; allowed, the plant has no longitudinal tire limit)",
        severity="warning")

    excess, k = _accel_violation(rl, lim)
    add("a_x within the grip / drive limits: worst excess [m/s^2]", excess, 0.1, excess <= 0.1,
        f"(at s = {rl.s[k]:.2f} m; a_x in [{rl.ax.min():.2f}, {rl.ax.max():.2f}])")

    ds = _step_lengths(rl)
    dpsi = angle_diff(np.roll(rl.psi, -1), rl.psi)
    mismatch = np.abs(dpsi / ds - 0.5 * (rl.kappa + np.roll(rl.kappa, -1))).max()
    add("|dpsi/ds - kappa| (heading convention, curvature sign) [rad/m]", mismatch, 0.05,
        mismatch <= 0.05)

    wmin = min(rl.w_right.min(), rl.w_left.min())
    add("min distance to a track bound [m]", wmin, 0.5 * lim.optimization_width,
        wmin >= 0.5 * lim.optimization_width - 0.05, "(>= half the optimization width)")

    gap = np.hypot(rl.x[0] - rl.x[-1], rl.y[0] - rl.y[-1])
    add("closing gap last -> first point [m]", gap, 2.0 * ds[:-1].max(), gap <= 2.0 * ds[:-1].max())

    slow = rl.vx < low_speed
    add(f"points below the MPC model's valid speed ({low_speed} m/s)", int(slow.sum()), 0, not slow.any(),
        f"(min v {rl.vx.min():.3f} m/s; the MPC model drops the plant's kinematic blend)",
        severity="warning")
    return ValidationReport(tuple(checks))


# ------------------------------------------------------------------------- plot


def plot_reference(rl, path, lim=None, centerline=None, title=None):
    """Save a validation figure: raceline with bounds, speed, curvature / a_lat, a_x."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    psi = rl.psi
    left = np.column_stack((-np.sin(psi), np.cos(psi)))  # unit normal to the left
    bound_l = rl.xy + rl.w_left[:, None] * left
    bound_r = rl.xy - rl.w_right[:, None] * left

    fig = plt.figure(figsize=(14, 9))
    grid = fig.add_gridspec(3, 2, width_ratios=(1.1, 1.0))
    a = fig.add_subplot(grid[:, 0])
    close = lambda p: np.vstack((p, p[:1]))  # noqa: E731
    a.plot(*close(bound_l).T, "k-", lw=1, label="track bounds")
    a.plot(*close(bound_r).T, "k-", lw=1)
    if centerline is not None:
        c = np.asarray(centerline)
        a.plot(*close(c[:, :2]).T, "k--", lw=0.8, label="centerline")
    sc = a.scatter(rl.x, rl.y, c=rl.vx, s=10, cmap="viridis", label="raceline")
    fig.colorbar(sc, ax=a, label="v_ref [m/s]", shrink=0.8)
    a.plot(rl.x[0], rl.y[0], "r^", ms=9, label="start (s = 0)")
    a.set_aspect("equal")
    a.set_xlabel("x [m]")
    a.set_ylabel("y [m]")
    a.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=4, fontsize=8)

    a = fig.add_subplot(grid[0, 1])
    a.plot(rl.s, rl.vx, label="v_ref")
    if lim is not None:
        a.axhline(lim.v_max, color="gray", ls=":", label="v_max")
    a.set_ylabel("speed [m/s]")
    a.legend(fontsize=8, loc="best")

    a = fig.add_subplot(grid[1, 1])
    a.plot(rl.s, rl.kappa, color="C0", label="kappa")
    a.set_ylabel("kappa [rad/m]", color="C0")
    b2 = a.twinx()
    b2.plot(rl.s, rl.a_lat, color="C1", label="a_lat")
    b2.set_ylabel("a_lat [m/s^2]", color="C1")
    if lim is not None:
        for sign in (1, -1):
            a.axhline(sign * lim.curvature_limit, color="C0", ls=":", lw=0.8)
            b2.axhline(sign * lim.ay_max, color="C1", ls=":", lw=0.8)

    a = fig.add_subplot(grid[2, 1])
    a.step(rl.s, rl.ax, where="post", label="a_x")
    if lim is not None:
        a.axhline(lim.max_accel, color="gray", ls=":", label="drive limit")
        a.axhline(-lim.max_decel, color="gray", ls="--", label="braking limit")
    a.set_xlabel("s [m]")
    a.set_ylabel("a_x [m/s^2]")
    a.legend(fontsize=8, loc="lower left")

    fig.suptitle(title or f"reference: {rl.length:.2f} m, lap {rl.lap_time:.2f} s")
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)
