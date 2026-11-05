# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""The plant for the Diff-WMPC experiments (steps.md Step 3).

:class:`Plant` is "the car" for Steps 4-8: the C++ ``SingleTrackAugDynModel`` (through
:class:`plannertrack_py.Sim`), started on the reference at racing speed, with the true mass and
mu set by a :class:`Perturbation` of the scenario's nominal values. The MPC only ever sees the
nominal values; the mismatch exists only here.

Gym-shaped (``reset(seed) -> state, info``) without depending on gymnasium.

Example::

    plant = Plant.from_scenario(PARAMS_DIR, reference=ref, perturbation=Perturbation(mu_scale=0.9))
    x, info = plant.reset(seed=0)                     # random start on the reference
    x, info = plant.step([jerk, steer_rate])          # one MPC interval; info: s, n, dpsi, dv, progress
    plant.set_perturbation(Perturbation(mass_scale=1.2))   # e.g. between episodes

    # domain randomization: a new (mass, mu) drawn from the reset RNG at every reset
    plant = Plant.from_scenario(PARAMS_DIR, reference=ref,
                                perturbation_range=PerturbationRange(mass_scale=(0.9, 1.1)))
"""

from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import yaml

from plannertrack_py._core import Sim
from plannertrack_py.config import (
    CONTROL_NAMES,
    DEFAULT_DYNAMICS_PLUGIN,
    STATE_NAMES,
    load_agent_yaml,
)
from plannertrack_py.utils import wrap_angle
from plannertrack_py.vehicle import agent_with_vehicle_params, load_vehicle_params
from plannertrack_py.raceline import compute_raceline
from plannertrack_py.reference import ReferenceTrack, load_raceline_limits

PLUGIN = DEFAULT_DYNAMICS_PLUGIN  # state [x, y, yaw, vx, vy, yaw_rate, steer, acc], input [jerk, steer_rate]
NX = len(STATE_NAMES[PLUGIN])  # number of states (8)
NU = len(CONTROL_NAMES[PLUGIN])  # number of controls (2)
X, Y, PSI, VX, VY, R, DELTA, ACC = range(NX)
JERK, STEER_RATE = range(NU)


# ------------------------------------------------------------------ perturbations


# Vehicle parameters the "plant ( ground truth )" can perturb, each by its own scale factor. Same tuple, same order as
# diff_weight_mpc.mpc.model.PARAMETER_NAMES (the MPC's parameter vector p), so every perturbation can
# also be handed to the MPC (oracle). Not imported from there to keep casadi out of the plant.
PERTURBABLE = ("mass", "mu", "Iz", "cf", "cr")


@dataclass(frozen=True)
class Perturbation:
    """True plant parameters as multiples of the nominal ones (1.0 = nominal).

    Each of PERTURBABLE has its own independent factor; which ones to use (and any physical coupling,
    e.g. a payload that changes mass and I_z together, see :meth:`heavier`) is the experiment's choice
    (steps.md §1.4, dynamics_analysis). Positional order: mass, mu, Iz, cf, cr, max_accel.

    ``max_accel_scale`` scales the plant's drive limit (vehicle_limits.max_accel, enforced by the C++
    plant with clip_control_in_sim), e.g. a fixed motor force on a heavier car. It is an actuator
    limit, not a model parameter: it is not in the MPC's parameter vector (the oracle keeps the
    nominal bound).
    """

    mass_scale: float = 1.0
    mu_scale: float = 1.0
    Iz_scale: float = 1.0  # yaw inertia
    cf_scale: float = 1.0  # load-normalized front cornering stiffness
    cr_scale: float = 1.0  # load-normalized rear cornering stiffness
    max_accel_scale: float = 1.0  # drive limit (plant actuator only)

    def __post_init__(self):
        if not (all(s > 0.0 for s in self.scales().values()) and self.max_accel_scale > 0.0):
            raise ValueError(f"scales must be positive, got {self}")

    @classmethod
    def heavier(cls, s, mu_scale=1.0, drive_limit=True):
        """A car s times heavier with the same mass distribution and tires (steps.md §1.4, variant B):
        mass and I_z x s; load-normalized stiffness c_f, c_r x 1/s (fixed physical cornering
        stiffness: our tire model is linear in the load, so without this the mass cancels); with
        ``drive_limit`` also max_accel x 1/s (fixed motor force)."""
        return cls(
            mass_scale=s,
            mu_scale=mu_scale,
            Iz_scale=s,
            cf_scale=1.0 / s,
            cr_scale=1.0 / s,
            max_accel_scale=1.0 / s if drive_limit else 1.0,
        )

    def scales(self):
        """``{parameter name: factor}`` for PERTURBABLE."""
        return {name: getattr(self, f"{name}_scale") for name in PERTURBABLE}

    def apply(self, params):
        """``params`` (a VehicleParams) with every perturbable parameter scaled."""
        return params.with_overrides(
            **{name: s * getattr(params, name) for name, s in self.scales().items()}
        )

    @property
    def is_nominal(self):
        return all(s == 1.0 for s in self.scales().values()) and self.max_accel_scale == 1.0


@dataclass(frozen=True)
class PerturbationRange:
    """Uniform ranges of the scales, for drawing a new :class:`Perturbation` per episode.

    Fixed scales (lo == hi, default (1, 1)) take no random draw, so adding a range for one parameter
    does not change the draws of the others.
    """

    mass_scale: tuple = (1.0, 1.0)
    mu_scale: tuple = (1.0, 1.0)
    Iz_scale: tuple = (1.0, 1.0)
    cf_scale: tuple = (1.0, 1.0)
    cr_scale: tuple = (1.0, 1.0)
    max_accel_scale: tuple = (1.0, 1.0)

    def sample(self, rng):
        scales = {}
        for name in (*PERTURBABLE, "max_accel"):
            lo, hi = getattr(self, f"{name}_scale")
            scales[f"{name}_scale"] = float(lo) if lo == hi else float(rng.uniform(lo, hi))
        return Perturbation(**scales)


@dataclass(frozen=True)
class StartNoise:
    """Random start offsets from the reference: truncated normals (std 0 = start exactly on it).

    Every draw is kept physical: the lateral offset stays inside the track (the car's edge at
    least ``track_margin`` from the bound), the heading error within ``dpsi_max`` and inside
    [MIN_DEV_FROM_REF, MAX_DEV_FROM_REF] (so the car always points forward along the track),
    and the speed within ``dv_max`` of v_ref and inside [MIN_SPEED, v_max]. A draw whose trimmed
    state is infeasible (no steady cornering state for the true plant), or whose body heading
    (heading error minus sideslip) leaves [MIN_DEV_FROM_REF, MAX_DEV_FROM_REF], is redrawn.
    """

    n_std: float = 0.0  # lateral offset, > 0 left [m]
    psi_std: float = 0.0  # heading error of the velocity w.r.t. the reference [rad]
    v_std: float = 0.0  # speed error [m/s]
    dpsi_max: float = np.deg2rad(20.0)  # |heading error| bound [rad], further limited to the
    # [MIN_DEV_FROM_REF, MAX_DEV_FROM_REF] range
    dv_max: float = 1.0  # |speed error| bound [m/s]
    track_margin: float = 0.05  # min distance of the car's edge to the track bound [m]

    def __post_init__(self):
        if (
            min(self.n_std, self.psi_std, self.v_std, self.dpsi_max, self.dv_max, self.track_margin)
            < 0.0
        ):
            raise ValueError(f"negative std or bound in {self}")

    @property
    def is_zero(self):
        return self.n_std == 0.0 and self.psi_std == 0.0 and self.v_std == 0.0


# Lowest speed the MPC model is valid for (= diff_weight_mpc.mpc.model.MIN_VALID_SPEED; not
# imported to keep casadi out of the plant).
MIN_SPEED = 2.5  # m/s
# Heading deviation from the reference the car may have (wrap(psi - psi_ref)); beyond +-90 deg it
# would point sideways or backwards along the track.
MIN_DEV_FROM_REF = np.deg2rad(-60.0)  # rad
MAX_DEV_FROM_REF = np.deg2rad(60.0)  # rad
MAX_START_DRAWS = 50  # redraws of the start offsets before falling back to the reference itself
# Failure flags in step()'s info (steps.md 3.4); any of them sets info["terminated"].
FAILURE_FLAGS = ("off_track", "too_slow", "heading_out", "diverged")
# Projection in step(): only reference segments within speed * dt * PROJECTION_SPEED_FACTOR +
# PROJECTION_MARGIN of the previous arc length are searched, so the foot point cannot jump to a
# neighbouring part of the track (paper §3.6). The factor and margin cover the foot point moving
# faster than the car on the inside of a corner (ds = dist / (1 - n kappa)).
PROJECTION_SPEED_FACTOR = 2.0
PROJECTION_MARGIN = 1.0  # m


# ------------------------------------------------------------------------- plant


class Plant:
    """C++ single-track plant with perturbable mass and mu, reset onto the reference.

    Args:
        sim_yaml_path, agents_yaml_path, planner_yaml_path: the scenario's config files.
        agent_name: agent in ``agents.yaml`` (its single-track parameters are the nominal ones).
        reference: :class:`ReferenceTrack` the episodes start on.
        dt: plant step = one control interval [s] (default 0.02 s: the paper's MPC call rate); a
            multiple of the simulator step.
        perturbation: true mass/mu as scales of the nominal ones.
        perturbation_range: if given, every :meth:`reset` draws a new perturbation from it
            (with the reset's RNG) unless ``reset(perturbation=...)`` passes one.
        start_noise: random start offsets drawn at every reset.
    """

    def __init__(
        self,
        sim_yaml_path,
        agents_yaml_path,
        planner_yaml_path,
        agent_name,
        reference,
        dt=0.02,
        perturbation=Perturbation(),
        perturbation_range=None,
        start_noise=StartNoise(),
    ):
        self._sim_yaml = Path(sim_yaml_path).read_text()
        self._agent = yaml.safe_load(load_agent_yaml(agents_yaml_path, agent_name, PLUGIN))
        # actuator limits enforced by the C++ plant (clip_control_in_sim); reported in step()'s info
        self.clips_control = bool(self._agent["dynamics_params"].get("clip_control_in_sim", False))
        self.nominal_params = load_vehicle_params(agents_yaml_path, agent_name)
        self.limits = load_raceline_limits(agents_yaml_path, agent_name, planner_yaml_path)
        self.reference = reference
        self.perturbation_range = perturbation_range
        self.start_noise = start_noise
        self._rng = np.random.default_rng()
        self._sim = None
        self._s = None  # arc length of the last projection; None until reset()
        self._progress = 0.0  # arc length driven since reset [m]
        self._t = 0.0  # time since reset [s]
        self.set_perturbation(perturbation)

        # dt is control calling rate, and sim_dt is internal simulation stepping time
        # rk-4 intergration time-step is different and set it in yaml file, 10x faster than simulation stepping time
        self.substeps = int(round(dt / self._sim.dt))
        if self.substeps < 1 or abs(self.substeps * self._sim.dt - dt) > 1e-9:
            raise ValueError(f"dt = {dt} is not a multiple of the simulator step {self._sim.dt}")
        self.dt = self.substeps * self._sim.dt

    @classmethod
    def from_scenario(cls, params_dir, agent_name="agent_1", reference=None, **kwargs):
        """Plant from a scenario's ``params/`` folder (sim.yaml, agents.yaml, min_curv_planner.yaml).

        Without ``reference`` the raceline is computed from the scenario's planner config, with
        the nominal parameters.
        """
        p = Path(params_dir)
        sim, agents, planner = p / "sim.yaml", p / "agents.yaml", p / "min_curv_planner.yaml"
        if reference is None:
            reference = ReferenceTrack(compute_raceline(agents, agent_name, planner))
        return cls(sim, agents, planner, agent_name, reference, **kwargs)

    # ------------------------------------------------------------ parameters

    @property
    def perturbation(self):
        return self._perturbation

    @property
    def true_params(self):
        """Vehicle parameters the plant actually runs with (nominal ones with the perturbation)."""
        return self._perturbation.apply(self.nominal_params)

    @property
    def sim(self):
        """The underlying :class:`Sim` (rebuilt by :meth:`set_perturbation`)."""
        return self._sim

    def set_perturbation(self, perturbation):
        """Change the true parameters (and drive limit). Rebuilds the C++ plant (~5 ms); the current
        state is kept."""
        state = None if self._sim is None else self._sim.get_state()
        self._perturbation = perturbation
        agent = agent_with_vehicle_params(self._agent, perturbation.apply(self.nominal_params))
        if perturbation.max_accel_scale != 1.0:
            agent["vehicle_limits"]["max_accel"] = (
                perturbation.max_accel_scale * self.limits.max_accel
            )
        self._sim = Sim(self._sim_yaml, yaml.safe_dump(agent))
        if state is not None:
            self._sim.set_state(state)

    # ------------------------------------------------------------------ reset

    def reset(self, seed=None, s0=None, perturbation=None):
        """Start an episode on the reference at its local speed (steps.md §4, 2.8).

        Args:
            seed: reseeds the plant's RNG (start point, start noise, perturbation draw); None
                continues the current RNG stream.
            s0: start arc length [m]; uniform on the lap if None.
            perturbation: true mass/mu for this episode; else drawn from ``perturbation_range``
                if set, else the current one is kept.

        Returns:
            ``(state, info)``. ``info``: s0, the start offsets n0/dchi0/dv0 (drawn from
            ``start_noise`` inside :meth:`start_bounds`; dchi0 is the course heading offset),
            the start's body heading error dpsi0 and sideslip beta0 (dpsi0 = dchi0 - beta0, same
            meaning as dpsi/dchi/beta in :meth:`step`), ``start_draws`` (draws needed; 0 without
            noise, -1 if no draw had a feasible trim and the car starts on the reference), the
            perturbation, and ``trim_converged`` (False if no steady cornering state exists for
            the true parameters at the start point, e.g. mu too low for the reference's a_lat;
            the car then starts with the kinematic steer angle).
        """
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        if perturbation is None and self.perturbation_range is not None:
            perturbation = self.perturbation_range.sample(self._rng)
        if perturbation is not None:
            self.set_perturbation(perturbation)

        if s0 is None:
            s0 = float(self._rng.uniform(0.0, self.reference.length))
        s0 = float(self.reference.wrap(s0))

        # offsets: truncated normals inside start_bounds(); redraw if the trim is infeasible
        draws = 0
        offsets, (state, converged) = (0.0, 0.0, 0.0), self.start_state(s0)
        if not self.start_noise.is_zero:
            for draws in range(1, MAX_START_DRAWS + 1):
                candidate = self._draw_offsets(s0)
                state_c, converged_c = self.start_state(s0, *candidate)
                body_dev = self.heading_deviation(state_c, s0)
                if converged_c and MIN_DEV_FROM_REF <= body_dev <= MAX_DEV_FROM_REF:
                    offsets, state, converged = candidate, state_c, True
                    break
            else:
                draws = -1  # no feasible noisy start: start on the reference (zero offsets)

        self._sim.set_state(state)
        self._s, self._progress, self._t = s0, 0.0, 0.0
        dn, dchi, dv = offsets
        beta = float(np.arctan2(state[VY], state[VX]))
        info = {
            "s0": s0,
            "n0": dn,
            "dchi0": dchi,
            "dpsi0": self.heading_deviation(state, s0),
            "beta0": beta,
            "dv0": dv,
            "start_draws": draws,
            "perturbation": asdict(self._perturbation),
            "trim_converged": converged,
        }
        return self._sim.get_state(), info

    # ------------------------------------------------------------------- step

    def step(self, u):
        """Apply ``u = [jerk, steer_rate]`` for one interval ``dt`` (``substeps`` simulator steps).

        The input is held constant over the interval. Actuator limits are the C++ plant's job
        (``clip_control_in_sim`` in agents.yaml: u clipped, steer and acc held at their end
        stops); the input is passed through unchanged.

        Returns:
            ``(state, info)``. ``info``: ``t`` (time since reset), ``s`` (arc length of the
            projection onto the reference, in [0, length)), ``n`` (lateral error, > 0 left),
            ``dpsi`` (body heading minus reference heading, wrapped: the paper's heading error,
            the MPC cost term), ``dchi`` (course heading, i.e. velocity direction psi + beta,
            minus reference heading: 0 when the car moves along the line), ``beta`` (sideslip
            atan2(vy, vx); dpsi = dchi - beta, so a car tracking perfectly in a corner has
            dpsi = -beta != 0), ``dv`` (vx - v_ref(s)),
            ``v_ref``, ``ds`` (arc length gained this step, < 0 when going backwards) and
            ``progress`` (arc length since reset, keeps growing past the start/finish line).
            With a non-finite state the Frenet values are NaN and progress is not updated.

            Episode flags (steps.md 3.4; the caller decides what to do with them):

            - ``off_track``: the car's side (half its width from the centre) is beyond a track
              bound at s: n > w_left - w/2 or n < -(w_right - w/2). Ignores the body's yaw.
            - ``too_slow``: vx < MIN_SPEED (below the MPC model's valid range).
            - ``heading_out``: dpsi outside [MIN_DEV_FROM_REF, MAX_DEV_FROM_REF].
            - ``diverged``: non-finite state (all other flags then False).
            - ``terminated``: any of the above (FAILURE_FLAGS).
            - ``lap_done``: progress >= one lap (success; not part of ``terminated``).

            Actuators (with ``clip_control_in_sim``; all False without it): ``input_clipped`` (u
            outside ±max_jerk / ±max_steer_rate, so the plant applied less), ``steer_at_limit``
            and ``acc_at_limit`` (the state sits at its end stop after the step).
        """
        if self._s is None:
            raise RuntimeError("call reset() before step()")
        u = np.asarray(u, dtype=float)
        if u.shape != (NU,):
            raise ValueError(
                f"u must have {NU} controls {list(CONTROL_NAMES[PLUGIN])}, got shape {u.shape}"
            )

        state = np.asarray(self._sim.step(u, n=self.substeps))

        self._t += self.dt
        info = {"t": self._t, **self._actuator_flags(u, state)}
        if not np.all(np.isfinite(state)):
            nan = float("nan")
            info.update(
                s=nan,
                n=nan,
                dpsi=nan,
                dchi=nan,
                beta=nan,
                dv=nan,
                v_ref=nan,
                ds=nan,
                progress=self._progress,
                off_track=False,
                too_slow=False,
                heading_out=False,
                diverged=True,
                terminated=True,
                lap_done=False,
            )
            return state, info

        speed = float(np.hypot(state[VX], state[VY]))
        window = speed * self.dt * PROJECTION_SPEED_FACTOR + PROJECTION_MARGIN
        proj = self.reference.project(state[X], state[Y], s_guess=self._s, window=window)
        half = 0.5 * self.reference.length
        ds = (proj.s - self._s + half) % self.reference.length - half
        self._s = proj.s
        self._progress += ds

        v_ref = float(self.reference.sample(proj.s)["vx"])
        beta = float(np.arctan2(state[VY], state[VX]))
        dpsi = _wrap_angle(float(state[PSI]) - proj.psi)
        info.update(
            s=proj.s,
            n=proj.n,
            dpsi=dpsi,
            dchi=_wrap_angle(dpsi + beta),
            beta=beta,
            dv=float(state[VX]) - v_ref,
            v_ref=v_ref,
            ds=float(ds),
            progress=self._progress,
        )

        n_left, n_right = self.track_limits(proj.s)
        info.update(
            off_track=not (n_right <= proj.n <= n_left),
            too_slow=bool(state[VX] < MIN_SPEED),
            heading_out=not (MIN_DEV_FROM_REF <= dpsi <= MAX_DEV_FROM_REF),
            diverged=False,
            lap_done=bool(self._progress >= self.reference.length),
        )
        info["terminated"] = any(info[k] for k in FAILURE_FLAGS)
        return state, info

    def track_limits(self, s):
        """Lateral offsets ``(n_left, n_right)`` at which the car's side touches the track bounds at
        arc length ``s`` (n_right < 0 < n_left); used for ``off_track``."""
        ref = self.reference.sample(s)
        half = 0.5 * self.limits.vehicle_width
        return float(ref["w_left"]) - half, -(float(ref["w_right"]) - half)

    def _actuator_flags(self, u, state):
        if not self.clips_control:
            return {"input_clipped": False, "steer_at_limit": False, "acc_at_limit": False}
        lim = self.limits
        return {
            "input_clipped": bool(
                abs(u[JERK]) > lim.max_jerk or abs(u[STEER_RATE]) > lim.max_steer_rate
            ),
            "steer_at_limit": bool(abs(state[DELTA]) >= lim.max_steer),
            "acc_at_limit": bool(state[ACC] >= lim.max_accel or state[ACC] <= -lim.max_decel),
        }

    @property
    def progress(self):
        """Arc length driven along the reference since the last :meth:`reset` [m]."""
        return self._progress

    def start_bounds(self, s0):
        """Bounds on the start offsets at ``s0``: ``{"n": (lo, hi), "dpsi": (lo, hi), "dv": (lo, hi)}``.

        dpsi: +-dpsi_max within [MIN_DEV_FROM_REF, MAX_DEV_FROM_REF]; dv: +-dv_max with
        v_ref + dv in [MIN_SPEED, v_max].

        n: the car (half width w/2) stays ``track_margin`` inside the bounds. The reference's
        w_right/w_left are measured from the raceline itself, which the optimizer kept at least
        optimization_width/2 from them, so the range always contains n = 0.
        """
        noise, ref = self.start_noise, self.reference.sample(s0)
        half = 0.5 * self.limits.vehicle_width + noise.track_margin
        n_lo = -max(float(ref["w_right"]) - half, 0.0)
        n_hi = max(float(ref["w_left"]) - half, 0.0)
        v_ref = float(ref["vx"])
        dv_lo = max(-noise.dv_max, min(MIN_SPEED - v_ref, 0.0))
        dv_hi = min(noise.dv_max, max(self.limits.v_max - v_ref, 0.0))
        dpsi_lo = max(-noise.dpsi_max, MIN_DEV_FROM_REF)
        dpsi_hi = min(noise.dpsi_max, MAX_DEV_FROM_REF)
        return {"n": (n_lo, n_hi), "dpsi": (dpsi_lo, dpsi_hi), "dv": (dv_lo, dv_hi)}

    def heading_deviation(self, state, s):
        """Body heading minus reference heading at arc length ``s``, wrapped (plannertrack_py.utils.angle_diff) [rad]."""
        return _wrap_angle(float(state[PSI]) - float(self.reference.sample(s)["psi"]))

    def _draw_offsets(self, s0):
        noise, bounds = self.start_noise, self.start_bounds(s0)
        return tuple(
            _truncated_normal(self._rng, std, *bounds[key])
            for key, std in (("n", noise.n_std), ("dpsi", noise.psi_std), ("dv", noise.v_std))
        )

    def start_state(self, s0, dn=0.0, dpsi=0.0, dv=0.0):
        """Plant state on the reference at arc length ``s0``, shifted by the given offsets.

        Position: reference point moved ``dn`` along the left normal. Speed: vx = v_ref + dv.
        Lateral state: steady cornering for the TRUE parameters (:meth:`trim`) on the curve the
        car is on, the parallel curve at offset dn with curvature kappa / (1 - dn*kappa), so an
        episode does not start with a steering transient. The yaw is set so the velocity (not the
        body axis) points along the reference: psi = psi_ref - beta + dpsi. Acceleration state:
        chosen so that vx_dot = a_x,ref.

        The offsets are used as given (no bounds); :meth:`reset` draws them inside
        :meth:`start_bounds`. Returns ``(state, trim_converged)``.
        """
        ref = self.reference.sample(s0)
        psi_ref, kappa = float(ref["psi"]), float(ref["kappa"])
        # ds_n/ds = (1-dn*kappa)
        # |n| < R
        if 1.0 - dn * kappa <= 0.0:
            raise ValueError(f"offset {dn} m is beyond the centre of curvature (kappa = {kappa})")
        kappa = kappa / (1.0 - dn * kappa)
        vx = float(ref["vx"]) + dv
        delta, vy, a, converged = self.trim(vx, kappa, float(ref["ax"]))
        beta = np.arctan2(vy, vx)
        state = np.zeros(NX)
        state[X] = float(ref["x"]) - dn * np.sin(psi_ref)
        state[Y] = float(ref["y"]) + dn * np.cos(psi_ref)
        state[PSI] = _wrap_angle(psi_ref - beta + dpsi)
        state[VX], state[VY], state[R] = vx, vy, vx * kappa
        state[DELTA], state[ACC] = delta, a
        return state, converged

    def trim(self, vx, kappa, ax, tol=1e-9, max_iter=30):
        """Steady cornering of the plant (:func:`steady_cornering` on ``Sim.xdot`` with the TRUE
        parameters and the scenario's max_steer). Returns ``(delta, vy, a, converged)``."""
        return steady_cornering(
            self._sim.xdot,
            vx,
            kappa,
            ax,
            self.true_params.wheelbase,
            self.limits.max_steer,
            tol=tol,
            max_iter=max_iter,
        )


def steady_cornering(xdot, vx, kappa, ax, wheelbase, max_steer, tol=1e-9, max_iter=30):
    """Steady cornering: (delta, vy) with vy_dot = r_dot = 0 at yaw rate vx*kappa.

    ``xdot(state, u)`` is any derivative with the plant's layout: ``Sim.xdot`` (the plant) or the
    MPC's CasADi model, so the same check tells whether the reference is drivable by either.
    Newton's method with a finite-difference Jacobian, from the kinematic guess
    delta = atan(kappa * L), vy = 0. The acceleration state is a = ax - vy*r, so the longitudinal
    load transfer matches vx_dot = ax.

    Returns ``(delta, vy, a, converged)``; if there is no steady state (grip too low) or it
    needs |delta| > max_steer, the kinematic guess (vy = 0) and ``converged = False``.
    """
    r = vx * kappa
    delta0 = float(np.arctan(kappa * wheelbase))

    def residual(z):
        delta, vy = z
        f = xdot(np.array([0.0, 0.0, 0.0, vx, vy, r, delta, ax - vy * r]), np.zeros(NU))
        return np.array([f[VY], f[R]])

    z = np.array([delta0, 0.0])
    for _ in range(max_iter):
        res = residual(z)
        if np.max(np.abs(res)) < tol:
            delta, vy = z
            if abs(delta) > max_steer:
                break  # steady state exists but needs more steering than the car has
            return float(delta), float(vy), float(ax - vy * r), True
        h = 1e-7
        jac = np.column_stack([(residual(z + h * e) - res) / h for e in np.eye(2)])
        try:
            step = np.linalg.solve(jac, -res)
        except np.linalg.LinAlgError:
            break
        z = z + step
        if not np.all(np.isfinite(z)) or abs(z[0]) > 0.5 * np.pi:
            break
    return delta0, 0.0, float(ax), False


def _truncated_normal(rng, std, lo, hi, max_tries=1000):
    """N(0, std) restricted to [lo, hi] (lo <= 0 <= hi), by rejection; 0 if std == 0."""
    if std == 0.0 or lo == hi:
        return 0.0
    for _ in range(max_tries):
        v = float(rng.normal(0.0, std))
        if lo <= v <= hi:
            return v
    return float(rng.uniform(lo, hi))  # bounds far inside the std: about uniform anyway


def _wrap_angle(a):
    return float(wrap_angle(a))  # the stack's one angle convention (plannertrack_py.utils), as a Python float
