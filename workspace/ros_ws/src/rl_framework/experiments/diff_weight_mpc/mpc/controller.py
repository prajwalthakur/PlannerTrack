# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Closed-loop MPC controller for the Diff-WMPC experiments (steps.md 4.3).

:class:`MpcController` turns plant states into inputs for B cars at once: it builds the stage
references from each car's arc length (``reference_params.stage_parameters``), solves the OCP with
leap-c's ``AcadosDiffMpcTorch`` (SQP_RTI, as in the paper) and returns u0 for every car.

Warm start: every solve starts from the previous solution (leap-c ``ctx``). A row is cold-started
from :class:`ReferenceInitializer` (every stage on its reference point) on the first call, after
:meth:`reset` of that row (a new episode for that car) and after that row failed. leap-c itself
also retries a failed row once from the same initializer within a call.

Failure fallback: if a row's solve still fails (status != 0 or non-finite u0), that car gets the
input of its last successful plan at the current time (plan step k = elapsed // dt), i.e. it
keeps following the last good plan; after the plan's horizon, zero input (holds steer and
acceleration). ``info["failures"]`` counts consecutive failures per car; the caller decides when
to give up (e.g. end the episode).

Example::

    plant = BatchPlant.from_scenario(PARAMS_DIR, n_cars=4, reference=ref, dt=0.02)
    mpc = MpcController.create_controller(plant)
    x, info = plant.reset(seed=0)
    s = info["s0"]
    while ...:
        u, mpc_info = mpc.step(x, s)              # theta=None: OcpConfig.theta0
        x, info = plant.step(u)
        s = info["s"]
        done = np.flatnonzero(info["terminated"] | info["lap_done"])
        if done.size:
            x[done], reset_info = plant.reset_cars(done)
            s[done] = reset_info["s0"]
            mpc.reset(done)

For learning pass ``theta`` as a torch tensor with ``requires_grad``; the solution
tensors of the last call (graph intact) are in :attr:`last`.
"""

import dataclasses
import time
from copy import deepcopy

import numpy as np
import torch
from leap_c.diff_mpc.data import AcadosOcpSolverInput
from leap_c.torch import AcadosDiffMpcTorch

from diff_weight_mpc.mpc.initializer import ReferenceInitializer
from diff_weight_mpc.mpc.model import NU
from diff_weight_mpc.mpc.ocp import OcpConfig, build_ocp
from diff_weight_mpc.mpc.reference_params import stage_parameters, vehicle_parameters
from plannertrack_py.utils import angle_diff, wrap_angle

# fields for which we need to compute the sensitivity matrix
# primal variable (x:states,u:controls, z:algebraic-states , sl:lower slack, su:upper slack)
# dual variable (pi: equality constraints multiplier), lam: inequality-multipliers, Nonzero only when that constraint is active)
_ITERATE_FIELDS = ("x", "u", "z", "sl", "su", "pi", "lam")


@dataclasses.dataclass
class MpcSolution:
    """leap-c outputs of the last :meth:`MpcController.act` (torch, graph intact for learning)."""

    ctx: object  # AcadosDiffMpcCtx  (Acadaos - diffmpc - context )
    u0: torch.Tensor  # (B, NU)
    x: torch.Tensor  # (B, N+1, NX) planned states
    u: torch.Tensor  # (B, N, NU) planned inputs
    value: torch.Tensor  # (B,) optimal cost
    params: dict  # the parameters the solve used (stage references, theta, vehicle)


class MpcController:
    """Batched Diff-WMPC controller
    Args:
        nominal_params: VehicleParams the MPC model uses (the plant's nominal ones).
        limits: RacelineLimits (bounds, friction ellipse, vehicle width for the track limits).
        reference: ReferenceTrack the stage references come from.
        control_dt: time between calls = the plant step [s]; for the failure fallback.
        cfg: OcpConfig (N, dt = the MPC interval, theta0, solver).
        n_batch: number of cars B (the batch solver is sized for it; other B work too).
        export_directory: where acados writes the generated code (default: leap-c's temp dir).
        log_residuals: also report the KKT residuals of every solve (recomputed after RTI; costs
            one extra evaluation per car). Tells how far one RTI iteration is from a KKT point,
            which matters for the sensitivities (steps.md 6.3, question 2).
    """

    def __init__(
        self,
        nominal_params,
        limits,
        reference,
        control_dt,
        cfg=OcpConfig(),
        n_batch=1,
        export_directory=None,
        name="diff_wmpc",
        log_residuals=False,
        verbose=False,
    ):
        self.cfg = cfg
        self.reference = reference
        self.vehicle_width = limits.vehicle_width
        self.log_residuals = log_residuals
        self.control_dt = float(control_dt)
        ocp, manager = build_ocp(nominal_params, limits, cfg, name=name)
        self.mpc = AcadosDiffMpcTorch(
            ocp,
            manager,
            export_directory=export_directory,
            n_batch_init=n_batch,
            verbose=verbose,
            dtype=torch.float64,
        )
        self.manager = manager
        # If used leap-c's default initial guess (all zeros, vx = 0) gives NaN;
        self.initializer = ReferenceInitializer(
            self.mpc.diff_mpc_fun.ocp, manager, nominal_params.wheelbase
        )
        self.mpc.diff_mpc_fun.initializer = self.initializer
        self.last = None
        self.reset()

    @classmethod
    def create_controller(cls, plant_config, cfg=OcpConfig(), **kwargs):
        """Controller matching a Plant or BatchPlant (nominal parameters, limits, reference, dt)."""
        return cls(
            plant_config.nominal_params,
            plant_config.limits,
            plant_config.reference,
            plant_config.dt,
            cfg,
            n_batch=getattr(plant_config, "n_cars", 1),
            **kwargs,
        )

    # ------------------------------------------------------------------ state

    def reset(self, rows=None):
        """
        cold start the mpc for the given rows

        """
        if rows is None:
            self._ctx = None
            self._t = 0.0
            self._cold_start = None
            self._plan_u = None  # (B, N, NU) last successful plan per car
            self._plan_t = None  # (B,) controller time at which it was computed (nan: none)
            self._failures = None
            self._yaw_prev = None  # (B,) yaw passed to the solver last call (nan: none)
            return
        if self._cold_start is None:
            return  # nothing stored yet: the next call starts cold anyway
        rows = np.atleast_1d(rows)
        self._cold_start[rows] = True
        self._plan_t[rows] = np.nan
        self._failures[rows] = 0
        self._yaw_prev[rows] = np.nan  # a new episode starts from the plant's raw yaw

    def _allocate(self, batch_size):
        self._ctx = None
        self._cold_start = np.ones(batch_size, dtype=bool)
        self._plan_u = np.zeros((batch_size, self.cfg.N, NU))
        self._plan_t = np.full(batch_size, np.nan)
        self._failures = np.zeros(batch_size, dtype=int)
        self._yaw_prev = np.full(batch_size, np.nan)

    def _fallback_inputs(self):
        """(B, NU): each car's last good plan at the current time; zeros without a plan or
        beyond its horizon."""
        u = np.zeros((self._plan_u.shape[0], NU))
        have = np.isfinite(self._plan_t)
        k = np.floor((self._t - self._plan_t[have]) / self.cfg.dt + 1e-9).astype(int)
        rows = np.flatnonzero(have)
        valid = k < self.cfg.N
        u[rows[valid]] = self._plan_u[rows[valid], k[valid]]
        return u

    def _warm_start(self, x0, params):
        """The ctx to pass to leap-c: None (all rows from the initializer), the previous ctx, or
        the previous ctx with the cold rows replaced by the initializer's iterate."""
        if self._ctx is None or self._cold_start is None:
            return None  # nothing to warm-start from yet
        if self._cold_start.all():
            return None  # every row cold: leap-c's initializer does all rows
        if not self._cold_start.any():
            return self._ctx  # every row warm: reuse the previous solution as is

        non_diff = {
            k: v for k, v in params.items() if k in self.manager.non_differentiable_parameter_names
        }
        p_stage = self.manager.combine_non_differentiable_parameters(
            batch_size=x0.shape[0], **non_diff
        )
        iterate = deepcopy(self._ctx.iterate)
        for i in np.flatnonzero(self._cold_start):
            single = self.initializer.single_iterate(
                AcadosOcpSolverInput(x0=x0[i], p_stagewise=p_stage[i])
            )
            for field in _ITERATE_FIELDS:
                getattr(iterate, field)[i] = getattr(single, field)
        return dataclasses.replace(self._ctx, iterate=iterate)

    def _theta_tensor(self, theta, batch_size):
        t = (
            theta
            if isinstance(theta, torch.Tensor)
            else torch.as_tensor(np.asarray(theta, dtype=float))
        )
        t = t.to(torch.float64)
        if t.ndim == 1:
            t = t.expand(batch_size, -1)
        if t.shape != (batch_size, len(self.cfg.theta0)):
            raise ValueError(
                f"theta must be ({len(self.cfg.theta0)},) or ({batch_size}, "
                f"{len(self.cfg.theta0)}), got {tuple(t.shape)}"
            )
        return t

    def step(self, states, s0, theta=None, vehicle_params=None):
        """Inputs u0 for B cars.

        Args:
            states: (B, NX) or (NX,) plant states.
            s0: (B,) or scalar arc lengths (Plant info["s"]; info["s0"] right after a reset).
            theta: None (OcpConfig.theta0), (6,) or (B, 6) weights; a torch tensor keeps the graph.
            vehicle_params: None (nominal) or a list of B VehicleParams (the oracle's true ones).

        Returns:
            ``(u, info)``: u (B, NU) numpy, the inputs to apply. info (arrays of length B):
            ``status`` (acados, 0 = success), ``ok`` (solution used), ``fallback`` (previous
            plan's input applied), ``failures`` (consecutive), ``cold_start``, ``iterations``,
            ``x_plan`` (B, N+1, NX), ``u_plan`` (B, N, NU), ``value``; scalars ``solve_time``
            (acados, incl. leap-c's retry) and ``wall_time`` (the whole call); ``residuals``
            (B, 4) [stat, eq, ineq, comp] if ``log_residuals``.
        """
        states = np.array(np.atleast_2d(states), dtype=float, copy=True)
        batch_size = states.shape[0]
        if self._cold_start is None or self._cold_start.shape[0] != batch_size:
            self._allocate(batch_size)
        # Continuous yaw per car. The plant reports its yaw wrapped into [-pi, pi], so it jumps by 2pi when
        # the heading passes +-pi; the warm start (previous plan) would then be 2pi off and SQP_RTI's
        # linearization fails. New yaw = last yaw + the shortest turn to the plant's yaw: same direction,
        # no jump. Cars without a previous yaw (first call, just reset) keep the plant's value.
        have = np.isfinite(self._yaw_prev)
        states[have, 2] = self._yaw_prev[have] + angle_diff(states[have, 2], self._yaw_prev[have])
        self._yaw_prev = states[:, 2].copy()
        start = time.perf_counter()
        x0, params = stage_parameters(
            self.reference, states, s0, self.cfg.N, self.cfg.dt, self.vehicle_width,
            track_margin=self.cfg.track_margin,
        )
        if vehicle_params is not None:
            params.update(vehicle_parameters(vehicle_params, self.cfg.N))
        if theta is not None:
            params["theta"] = self._theta_tensor(theta, batch_size)
        cold_start = self._cold_start.copy()
        # solve ocp
        ctx, u0, x, u, value = self.mpc(
            torch.as_tensor(x0), params=params, ctx=self._warm_start(x0, params)
        )
        # store the last solution
        self.last = MpcSolution(ctx, u0, x, u, value, params)
        status = np.asarray(ctx.status).copy()
        # just transfer the u0 sol. back to cpu numpy from gpu
        u0_np = u0.detach().cpu().numpy()
        ok = (status == 0) & np.all(np.isfinite(u0_np), axis=1)
        u_apply = np.where(ok[:, None], u0_np, self._fallback_inputs())

        # bookkeeping
        self._ctx = ctx
        self._cold_start = ~ok  # a failed row starts from the reference next time; a solved one warm
        self._plan_u[ok] = u.detach().cpu().numpy()[ok]
        self._plan_t[ok] = self._t
        self._failures = np.where(ok, 0, self._failures + 1)
        self._t += self.control_dt
        solvers = self.mpc.diff_mpc_fun.forward_batch_solver.ocp_solvers[:batch_size]
        # KKT residuals [stationarity, equality (dynamics), inequality, complementarity]
        x_plan = x.detach().cpu().numpy().copy()
        x_plan[..., 2] = wrap_angle(x_plan[..., 2])  # reported in [-pi, pi], like the plant
        info = {
            "status": status,
            "ok": ok,
            "fallback": ~ok,
            "failures": self._failures.copy(),
            "cold_start": cold_start,
            "iterations": np.array([s.get_stats("sqp_iter") for s in solvers]),
            "x_plan": x_plan,
            "u_plan": u.detach().cpu().numpy(),
            "value": value.detach().cpu().numpy().reshape(batch_size),
            "solve_time": float(ctx.log["solving_time"]) if ctx.log else np.nan,
        }
        if self.log_residuals:
            info["residuals"] = np.stack([s.get_residuals(recompute=True).ravel() for s in solvers])
        info["wall_time"] = time.perf_counter() - start
        return u_apply, info
