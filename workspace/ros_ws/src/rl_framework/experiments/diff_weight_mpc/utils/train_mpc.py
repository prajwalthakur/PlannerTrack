#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Diff-MPC training (paper Algorithm 1; Jahncke et al., RA-L 2026): learn the MPC cost weights θ by gradient
descent through the MPC. Static branch (steps.md Step 6): ONE weight vector θ for the whole lap. Weights-varying
branch (Step 7, steps.md §10, --model policy): a policy θ_k = π(o_k) from the track ahead
(diff_weight_mpc.learning.policy_network.WeightPolicy), outputs in <out>/train_policy/<tag>/ (+ policy.pt).

Fair static vs policy comparison (7.5): the policy's output map is the static softplus (exact_softplus, β = 1) and
its last layer starts at weights 0, so z = bias = the static w and π(o) = θ0 for every o. The bias is trained with
--lr (the static run's lr); the rest of the network with --lr-net. With --lr-net 0 a policy run reproduces the
static run with the same settings exactly (test_train_mpc.py); --lr-net is the only new learning parameter.
(Renamed from train_static.py on 2026-10-25; static runs still write to <out>/train_static/.)

Algorithm 1 (static branch) and where each line lives here:

    1  θ ← θ0                                         StaticWeights(theta0)
    2  x_k ← GetInitialState(); ∇accum ← 0            rollout(...) resets the plant; optimizer.zero_grad()
    3  for k = 0 … N_total - 1:                       for sd in rollout(..., max_steps=N_total)
    4      θ_k ← θ                                    StaticWeights.forward (the rollout's theta_fn)
    5      (z*_k, S_k) ← SolveDiffMPC(x_k, θ_k)       inside rollout: MpcController.step (leap-c)
    6      L_k ← ComputeLoss(z*_k, Γ)                 step_loss(sd, ...) (learning/loss.py node_loss)
    7      ∇θ L_k ← WeightsGrad(L_k, z*_k, θ_k, S_k)  L_k.backward() (leap-c's sensitivities)
    8      (static: ∇θ L_k as is)
    9      ∇accum ← ∇accum + ∇θ L_k                   torch accumulates .grad across backward() calls
   10      if (k + 1) mod N_batch = 0:                is_update_step(...): N_batch = accumulate_steps (static: one episode)
   11          θ ← Optimizer(θ, ∇accum)               optimizer_update(...) (Adam, gradient clipping ±0.1)
   12          ∇accum ← 0                             optimizer.zero_grad()
   14      x_{k+1} ← SimulateStep(x_k, u*_k)          inside rollout: BatchPlant.step
   16  return θ                                       save_checkpoint(...) → theta.json

Terminology (as in closed_loop.py and steps.md §6.2): "batch" = the B cars driven in parallel (BatchPlant,
(B, ...) arrays); an "episode" = one car from a reset until it completes its laps, fails or hits the time limit;
the paper's N_batch (control steps whose gradients are summed per update) is ``accumulate_steps`` here, and the
steps between two updates are an "accumulation window".

Paper safeguards kept: positive weights (θ = exp(w) or softplus(w), --weight-param; the paper optimizes θ directly
in the static case and uses a softplus output in Step 7), gradient clipping ±0.1, fallback to the last feasible
weights when the MPC fails.

Our additions: rows whose solve failed (controller fallback, mpc_info["ok"] False) or that are not in an episode
(StepData.active False) contribute no loss (their solution tensors are not a valid KKT point).

Outputs (in <out>/train_static/<tag>/): theta.json (final and best θ), history.csv (one row per update: θ, mean
loss and its parts, failures, episode outcome), checkpoints/. Evaluate the learned θ with
`baseline_suite.py --theta-file <out>/train_static/<tag>/theta.json`.

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/experiments/diff_weight_mpc/utils/train_mpc.py --laps 20
"""

import argparse
import copy
import csv
import json
from dataclasses import asdict, dataclass, field, replace
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from plannertrack_py import ReferenceTrack, compute_raceline
from plannertrack_py.batch_plant import BatchPlant
from diff_weight_mpc.closed_loop import rollout
from diff_weight_mpc.learning.loss import LossConfig, node_loss, threshold_penalty
from diff_weight_mpc.learning.policy_network import HIDDEN_ACTIVATIONS, INPUTS, WeightPolicy, observation_scale
from diff_weight_mpc.mpc.controller import MpcController
from diff_weight_mpc.mpc.ocp import THETA0, THETA_NAMES, OcpConfig

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent


# ------------------------------------------------------------------ configuration

# Provisional loss scales (steps.md 6.0, 2026-10-24): each term normalized by a typical size, α = w_n/σ_n² etc.,
# all importances w = 1 ("balanced"), lateral term at node 1 (paper). σ_n, σ_v are the baseline's plant RMS errors
# (results/baseline.md), not yet the node-1 plan values; jerk and steering rate use the paper's penalty thresholds.
# To be measured and revisited when evaluating Step 6 (6.9).
LOSS_SCALES = {"n": 0.049, "v": 0.39, "jerk": 8.0, "steer_rate": 0.25}
LOSS_IMPORTANCE = {"n": 1.0, "v": 1.0, "jerk": 1.0, "steer_rate": 1.0}


def balanced_loss_config(importance=None):
    """LossConfig with α…δ = w_i / σ_i² (``importance`` or LOSS_IMPORTANCE, LOSS_SCALES): every term ≈ w_i at a
    typical error. ``importance``: {"n", "v", "jerk", "steer_rate"} → w_i."""
    w, sigma = importance or LOSS_IMPORTANCE, LOSS_SCALES
    return replace(
        LossConfig(),
        alpha=w["n"] / sigma["n"] ** 2,
        beta=w["v"] / sigma["v"] ** 2,
        gamma=w["jerk"] / sigma["jerk"] ** 2,
        delta=w["steer_rate"] / sigma["steer_rate"] ** 2,
    )


@dataclass
class TrainConfig:
    """Training settings. Defaults marked TODO are decisions still open (steps.md Step 6)."""

    laps: int = 20  # training length N_total, counted in episodes (laps or failures)
    theta0: tuple = THETA0  # initial weights (Algorithm 1 line 1); --theta0
    # accumulate_steps = the paper's N_batch (Algorithm 1 line 10): steps whose gradients are summed per update. "episode" = the paper's static
    # Diff-MPC (one update per lap, §II-C); an int = every accumulate_steps control steps (the paper's Diff-WMPC uses 10,
    # Table I). With an int, θ changes during the lap, so lr must be smaller (≈ 30 updates per lap at 10).
    accumulate_steps: object = "episode"
    n_cars: int = (
        1  # parallel cars (B); reference run B = 1 (steps.md §6.2, training-loop decisions)
    )
    lr: float = 3e-2  # TODO: Adam on w = log θ, tuned for accumulate_steps="episode"; the paper's 2.9e-5 is for the Step 7 policy
    grad_clip: float = 0.1  # paper: gradient clipping ±0.1 (element-wise on ∂L/∂w)
    starts: str = "random"  # TODO: "random" (new start each episode) or "fixed" (always s0)
    s0: float = 0.0  # start arc length for starts="fixed"
    seed: int = 0
    nlp_solver: str = "SQP_RTI"  # as the paper (Step 5: RTI gradients approximate, direction kept); "SQP" + max_iter = fallback
    max_iter: int = 3  # SQP iterations per solve when nlp_solver = "SQP"
    loss_cfg: LossConfig = field(
        default_factory=balanced_loss_config
    )  # provisional balanced α…δ (6.0); main() passes make_loss_config(args) (--loss balanced|paper)
    checkpoint_every: int = 5  # updates between checkpoints
    weight_param: str = "exp"  # θ = exp(w) or softplus(w) (6.1: both explored; softplus moves large weights ~lr absolute)
    model: str = "static"  # "static" (Step 6) or "policy" (Step 7: θ_k = π(o_k))
    lr_net: float = 1e-3  # policy only: lr of every parameter except the last layer's bias (which uses lr)
    hidden_activation: str = "softplus"  # policy only: "softplus" (paper) or "none"
    beta: float = 1.0  # policy only: softplus β of the output map (1 = the static softplus)
    policy_inputs: str = "all"  # policy only: "all", "speed", "curvature" (7.7 ablations)
    policy_seed: int = 0  # policy only: hidden-layer initialization


# ------------------------------------------------------------------ the learnable weights (Alg. 1 line 1, 4)

WEIGHT_PARAMS = (
    "exp",
    "softplus",
)  # θ = exp(w) (ours, relative steps) or softplus(w) (the paper's Step 7 output)


def _to_theta(w, param):
    """w -> θ > 0."""
    return torch.exp(w) if param == "exp" else torch.nn.functional.softplus(w)


def _to_w(theta, param):
    """Inverse of _to_theta, so that training starts exactly at θ0."""
    theta = torch.as_tensor(theta, dtype=torch.float64)
    if param == "exp":
        return torch.log(theta)
    # inverse softplus:
    return theta + torch.log(-torch.expm1(-theta))


class StaticWeights(torch.nn.Module):
    """
    One weight vector θ for every step and every car (Algorithm 1, static branch), θ = exp(w) or softplus(w).
    w (6,) is the learnable parameter, initialized so that θ(w0) = θ0. Both keep θ > 0 (convex cost). They differ in
    how far θ moves per Adam step on w (≈ lr per coordinate): exp moves every weight by ≈ lr relative; softplus moves
    large weights by ≈ lr absolute (q_n = 50: 0.06 %) and small ones relatively (r_j = 0.01: like exp). Used as the
    rollout's ``theta_fn``
    """

    def __init__(self, theta0=THETA0, param="exp"):
        """
        initialization module
        """
        super().__init__()
        if param not in WEIGHT_PARAMS:
            raise ValueError(f"param must be one of {WEIGHT_PARAMS}, got {param}")
        self.param = param
        self.w = torch.nn.Parameter(_to_w(theta0, self.param))

    def forward(self, obs, k):
        """θ for this control step: (n_cars, 6) float64, with graph to w (n_cars = 1 in the paper's setting).

        rollout calls theta_fn(obs, k) once per control step k; obs (n_cars, 10) is each car's track ahead (the Step 7
        policy input). Static weights ignore both: the same θ at every step. The paper's N_batch (steps per update) is
        not a dimension here: train() accumulates gradients over steps (cfg.accumulate_steps).
        """
        batch = 1 if obs is None else obs.shape[0]
        return _to_theta(self.w, self.param).unsqueeze(0).repeat(batch, 1)

    @property
    def theta(self):
        """Current θ as numpy (6,), for logging and checkpoints."""
        with torch.no_grad():
            return _to_theta(self.w, self.param).numpy().copy()


def make_optimizer(model, cfg):
    """Adam. Static: over w with cfg.lr. Policy: the last layer's bias (the static w's counterpart) with cfg.lr, every
    other parameter with cfg.lr_net."""
    if getattr(model, "param", None) != "policy":
        return torch.optim.Adam(model.parameters(), lr=cfg.lr)
    bias = model.policy.head.bias
    rest = [q for q in model.parameters() if q is not bias]
    return torch.optim.Adam([{"params": [bias], "lr": cfg.lr}, {"params": rest, "lr": cfg.lr_net}])


class PolicyWeights(torch.nn.Module):
    """A WeightPolicy as the training model, with StaticWeights' interface: forward = the policy (rollout's
    theta_fn), ``theta`` = its mean θ over a lap (``probe_obs``: observations along the lap) for the log, the
    progress line and theta.json, ``param`` = "policy"."""

    def __init__(self, policy, probe_obs):
        super().__init__()
        self.policy = policy
        self.param = "policy"
        self.register_buffer("probe_obs", torch.as_tensor(probe_obs, dtype=torch.float64))

    def forward(self, obs, k=None):
        return self.policy(obs, k)

    def theta_along_lap(self):
        """θ (n_probe, 6) along the lap, as numpy."""
        with torch.no_grad():
            return self.policy(self.probe_obs).numpy().copy()

    @property
    def theta(self):
        """Mean θ over the lap (6,), numpy."""
        return self.theta_along_lap().mean(axis=0)


def lap_observations(reference, n=200):
    """(n, 10) policy observations at n evenly spaced points of the lap (PolicyWeights.probe_obs)."""
    s = np.linspace(0.0, reference.length, n, endpoint=False)
    return np.concatenate(reference.policy_observation(s), axis=1)


def build_model(cfg, ref):
    """StaticWeights (cfg.model "static") or PolicyWeights around a WeightPolicy (exact_softplus with cfg.beta, so
    its start equals the static softplus model's)."""
    if cfg.model == "static":
        return StaticWeights(cfg.theta0, cfg.weight_param)
    policy = WeightPolicy(cfg.theta0, observation_scale(ref), hidden_activation=cfg.hidden_activation,
                          output_map="exact_softplus", beta=cfg.beta, seed=cfg.policy_seed, inputs=cfg.policy_inputs)
    return PolicyWeights(policy, lap_observations(ref))


# ------------------------------------------------------------------ setup


def make_loss_config(args):
    """LossConfig for training. ``args.loss``: "balanced" (default, provisional: α…δ = w_i / σ_i² from LOSS_SCALES and
    LOSS_IMPORTANCE, so every term is ≈ w_i at a typical error) or "paper" (Table I values, for comparison; their
    lateral term is negligible at our scale, steps.md §8)."""
    if args.loss_weights is not None:  # explicit α, β, γ, δ
        alpha, beta, gamma, delta = args.loss_weights
        return replace(LossConfig(), alpha=alpha, beta=beta, gamma=gamma, delta=delta)
    if args.loss_importance is not None:  # balanced scales, chosen importances
        return balanced_loss_config(dict(zip(("n", "v", "jerk", "steer_rate"), args.loss_importance)))
    return LossConfig() if args.loss == "paper" else balanced_loss_config()


def build_setup(args, cfg):
    """Reference, BatchPlant (cfg.n_cars, dt 0.02, nominal car unless a perturbation is asked for) and
    MpcController (OcpConfig with cfg.nlp_solver / cfg.max_iter). Returns ``(ref, plant, mpc)``."""
    params = SRC / "scenarios" / args.scenario / "params"
    ref = ReferenceTrack(
        compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml")
    )
    plant = BatchPlant.from_scenario(
        params, n_cars=cfg.n_cars, agent_name=args.agent, reference=ref
    )
    ocp_cfg = replace(OcpConfig(), nlp_solver=cfg.nlp_solver)
    if cfg.nlp_solver == "SQP":  # fallback (Step 5): a few SQP iterations per solve
        ocp_cfg = replace(ocp_cfg, max_iter=cfg.max_iter)
    mpc = MpcController.create_controller(
        plant,
        cfg=ocp_cfg,
        export_directory=args.out / "train_static" / f"acados_{cfg.nlp_solver.lower()}",
        name=f"train_static_{cfg.nlp_solver.lower()}",
    )
    return ref, plant, mpc


# ------------------------------------------------------------------ one step (Alg. 1 lines 6-9)


def step_loss(sd, loss_cfg):
    """Loss of one StepData over its valid rows: node_loss on sd.solution (x, u, params), masked by
    sd.active & sd.mpc_info["ok"].

    Returns ``(loss_sum, n_valid, parts)``: loss_sum a torch scalar with graph to w (0 without valid rows,
    then no backward), n_valid the number of rows used, parts {name: float} for logging (means over valid rows).
    """
    valid = sd.active & sd.mpc_info["ok"]
    n_valid = int(valid.sum())
    if n_valid == 0:  # no data point: no backward this step
        return torch.zeros((), dtype=torch.float64), 0, {}
    sol = sd.solution
    loss, data_obs = node_loss(
        sol.x, sol.u, sol.params, loss_cfg
    )  # (B,), data_obs: n, dv, jerk, steer_rate at mpc-step 1
    mask = torch.as_tensor(valid)
    loss_sum = loss[mask].sum()  # equation-9 (sum of loss, not mean)
    wght = loss_cfg
    terms = {  # each weighted term, to see which one drives the loss (6.0 revisit)
        "L_n": wght.alpha * data_obs["n"] ** 2,  # lateral-error
        "L_v": wght.beta * data_obs["dv"] ** 2,  # long-speed error
        "L_jerk": wght.gamma * threshold_penalty(data_obs["jerk"], wght.jerk_th, wght.kappa_j),
        "L_steer": wght.delta
        * threshold_penalty(data_obs["steer_rate"], wght.steer_rate_th, wght.kappa_omega),
    }
    parts = {name: float(v.detach()[mask].mean()) for name, v in {**data_obs, **terms}.items()}
    parts["loss"] = float(loss_sum.detach()) / n_valid  # avg-loss
    return loss_sum, n_valid, parts


def is_update_step(sd, k, cfg):
    """Algorithm 1 line 10: (k + 1) mod N_batch = 0, with N_batch = cfg.accumulate_steps.

    - cfg.accumulate_steps == "episode" (paper, static): True when the (single) car's episode ended at this step
      (sd.episode_end; a lap or a failure). With n_cars > 1: when every car finished at least one episode since
      the last update (to define).
    - cfg.accumulate_steps an int: True every accumulate_steps control steps, (k + 1) % accumulate_steps == 0 (paper's Diff-WMPC: 10).
    """
    if cfg.accumulate_steps == "episode":
        if sd.episode_end.size != 1:
            raise NotImplementedError(
                "accumulate_steps='episode' needs n_cars = 1 (multi-car: to define)"
            )
        return bool(sd.episode_end[0])
    return (k + 1) % cfg.accumulate_steps == 0


# ------------------------------------------------------------------ update (Alg. 1 lines 11-12) and fallback


def optimizer_update(model, optimizer, cfg, n_points):
    """Mean of the accumulated gradients over n_points, element-wise clip ±cfg.grad_clip (paper), Adam step, zero grad.
    Returns the gradient on the 6 weight-level parameters (static w, or the policy's last-layer bias; numpy, before
    clipping) for logging; None if there was none. A non-finite gradient anywhere skips the update."""
    params = [q for q in model.parameters() if q.grad is not None]
    if not params:
        optimizer.zero_grad()
        return None
    for q in params:
        q.grad.div_(max(n_points, 1))
    level = model.policy.head.bias if getattr(model, "param", None) == "policy" else model.w
    raw = level.grad.detach().numpy().copy() if level.grad is not None else np.full(len(THETA_NAMES), np.nan)
    if all(bool(torch.isfinite(q.grad).all()) for q in params):
        torch.nn.utils.clip_grad_value_(params, cfg.grad_clip)
        optimizer.step()
    optimizer.zero_grad()
    return raw


class FeasibleWeights:
    """Paper (§IV-A): "If the MPC solver fails to converge, a fallback mechanism reverts to the last feasible
    weights set." Keeps a copy of the w that last drove a whole accumulation window without a solver failure.

    Feasible = every solve in the window succeeded (mpc_info["ok"] on the active rows; train() tracks it as
    window_ok). Failure flags (off track, too slow, …) do not count: the paper's criterion is solver convergence.
    The initial w (θ0) is taken as feasible (baseline: 0 solver failures).
    """

    def __init__(self, model):
        self._state = copy.deepcopy(model.state_dict())  # every parameter (static: w; policy: the network)
        self.n_restores = 0

    def record(self, model):
        """Call at the end of a CLEAN window, before the update: the parameters that just drove it are feasible."""
        self._state = copy.deepcopy(model.state_dict())

    def restore(self, model, optimizer=None):
        """Write the last feasible parameters back (in place: the optimizer keeps its references). With
        ``optimizer``, also reset Adam's state: its momentum points in the direction that just led to the failure,
        and would push straight back there."""
        model.load_state_dict(self._state)
        if optimizer is not None:
            for q in model.parameters():
                optimizer.state.pop(q, None)  # Adam re-creates m, v and its step count at the next step
        self.n_restores += 1


# ------------------------------------------------------------------ logging and checkpoints


class TrainingLog:
    """One row per update (accumulation window): what drove it, what it produced, how the episodes went.

    ``add_step(sd, ...)`` every control step collects the window; ``end_update(...)`` closes it into a row:
    update index, steps, data points, θ that drove the window (theta_*) and θ after the update (theta_next_*),
    mean loss and its parts (raw node-1 errors n, dv, jerk, steer_rate and the weighted terms L_*), the gradient
    before clipping (grad_*, NaN if none) and how many components the clip hit, solver failures, episodes that
    ended in the window with their reasons and lap times, and whether the fallback was used. ``write(out_dir)`` →
    history.csv. ``rows`` (list of dicts) is what save_theta and the checkpoints use.
    """

    def __init__(self, dt=0.02):
        self.dt = dt
        self.rows = []
        self._reset()

    def _reset(self):
        self._steps = 0
        self._points = 0
        self._sums = {}
        self._failures = 0
        self._ends = []  # (reason, episode duration [s])
        self._theta_sum = None  # θ over the window's active rows: sum, min, max (constant for static weights)
        self._theta_min = None
        self._theta_max = None
        self._theta_rows = 0

    def add_step(self, sd, loss_sum, n_valid, parts):
        th = sd.theta.detach().cpu().numpy() if isinstance(sd.theta, torch.Tensor) else np.asarray(sd.theta)
        th = np.atleast_2d(th)
        th = th[np.asarray(sd.active, dtype=bool)] if th.shape[0] == len(sd.active) else th
        if th.size:
            if self._theta_sum is None:
                self._theta_sum = np.zeros(th.shape[1])
                self._theta_min = np.full(th.shape[1], np.inf)
                self._theta_max = np.full(th.shape[1], -np.inf)
            self._theta_sum += th.sum(axis=0)
            self._theta_min = np.minimum(self._theta_min, th.min(axis=0))
            self._theta_max = np.maximum(self._theta_max, th.max(axis=0))
            self._theta_rows += th.shape[0]
        self._steps += 1
        self._points += n_valid
        for name, value in parts.items():  # parts are means over the step's valid rows: weight by n_valid
            self._sums[name] = self._sums.get(name, 0.0) + value * n_valid
        self._failures += int((sd.active & ~sd.mpc_info["ok"]).sum())
        for i in np.flatnonzero(sd.episode_end):
            self._ends.append((sd.end_reason[i], (sd.episode_step[i] + 1) * self.dt))

    def end_update(self, update, model, grad, fallback_used, grad_clip=None):
        row = {"update": update, "steps": self._steps, "points": self._points}
        mean = self._theta_sum / self._theta_rows if self._theta_rows else np.full(len(THETA_NAMES), np.nan)
        row.update({f"theta_{n}": v for n, v in zip(THETA_NAMES, mean)})  # θ that drove the window (mean)
        row.update({f"theta_min_{n}": v for n, v in zip(THETA_NAMES, self._theta_min if self._theta_rows else mean)})
        row.update({f"theta_max_{n}": v for n, v in zip(THETA_NAMES, self._theta_max if self._theta_rows else mean)})
        row["theta_scale"] = float(np.exp(np.mean(np.log(mean)))) if np.all(mean > 0) else np.nan  # geometric mean
        row.update({f"theta_next_{n}": v for n, v in zip(THETA_NAMES, model.theta)})
        for name, total in self._sums.items():
            row[name] = total / self._points if self._points else np.nan
        g = np.full(len(THETA_NAMES), np.nan) if grad is None else np.asarray(grad)
        row.update({f"grad_{n}": v for n, v in zip(THETA_NAMES, g)})
        row["clipped"] = int(np.sum(np.abs(g) > grad_clip)) if grad_clip is not None and grad is not None else 0
        row["solver_failures"] = self._failures
        row["episodes"] = len(self._ends)
        row["end_reasons"] = ";".join(r for r, _ in self._ends)
        laps = [t for r, t in self._ends if r == "lap_done"]
        row["lap_time"] = float(np.mean(laps)) if laps else np.nan
        row["fallback"] = bool(fallback_used)
        self.rows.append(row)
        self._reset()
        return row

    def write(self, out_dir):
        if not self.rows:
            return
        keys = list(dict.fromkeys(k for row in self.rows for k in row))  # union, in first-seen order
        with open(Path(out_dir) / "history.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.rows)


def best_row(rows):
    """The update whose window is the best evidence for its θ: no fallback, no solver failure, every episode that
    ended in it completed its lap; among those the lowest mean loss. None if no window qualifies."""
    clean = [r for r in rows if not r["fallback"] and r["solver_failures"] == 0 and r["episodes"] > 0
             and all(reason == "lap_done" for reason in r["end_reasons"].split(";"))
             and np.isfinite(r.get("loss", np.nan))]
    return min(clean, key=lambda r: r["loss"]) if clean else None


def save_checkpoint(path, model, optimizer, update, log):
    """torch.save of the model (static: w and the weight map; policy: WeightPolicy config and state_dict), optimizer
    state, update index and the history so far (resume and inspection; eval_checkpoints.py)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {"optimizer": optimizer.state_dict(), "update": update, "history": log.rows}
    if model.param == "policy":
        data.update({"model": "policy", "policy_config": model.policy.config,
                     "state_dict": copy.deepcopy(model.policy.state_dict())})
    else:
        data.update({"model": "static", "w": model.w.detach().clone(), "weight_param": model.param})
    torch.save(data, path)


def save_theta(path, model, log, cfg):
    """theta.json: {"theta": final θ, "best_theta": θ that drove the best window (best_row: clean, every episode a
    completed lap, lowest mean loss) or null, "best_update", "theta0", "weight_param", "model", "config"};
    baseline_suite.py --theta-file reads "theta" (or another key with --theta-key). For a policy, "theta" is only
    the policy's mean θ over the lap (for reference); the policy itself goes to policy.pt next to theta.json
    (evaluate it with --policy-file)."""
    best = best_row(log.rows)
    config = asdict(cfg)
    data = {
        "theta": [float(v) for v in model.theta],
        "best_theta": None if best is None else [float(best[f"theta_{n}"]) for n in THETA_NAMES],
        "best_update": None if best is None else best["update"],
        "theta0": [float(v) for v in cfg.theta0],
        "theta_names": list(THETA_NAMES),
        "weight_param": model.param,
        "model": "policy" if model.param == "policy" else "static",
        "updates": len(log.rows),
        "config": config,
    }
    Path(path).write_text(json.dumps(data, indent=1, default=str))
    if model.param == "policy":
        model.policy.save(Path(path).with_name("policy.pt"))


# chart palette shared with closed_loop_baseline.py (categorical order fixed)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
GRID_COLOR, REF_COLOR = "#e1e0d9", "#52514e"


def plot_history(history_csv, out_png, smooth=10):
    """training.png: θ_i/θ0_i per update (one line per weight, log scale) and the loss terms (L_n, L_v, L_jerk,
    L_steer; rolling mean over ``smooth`` updates, raw total faint) per update."""
    with open(history_csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return
    upd = np.array([int(r["update"]) for r in rows])
    fig, (ax_t, ax_l) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for i, name in enumerate(THETA_NAMES):
        ratio = np.array([float(r[f"theta_next_{name}"]) for r in rows]) / THETA0[i]
        ax_t.plot(upd, ratio, lw=2, color=SERIES[i], label=name)
    ax_t.axhline(1.0, color=REF_COLOR, lw=1, ls=":")
    ax_t.set_yscale("log")
    plain = matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}")
    ax_t.yaxis.set_major_formatter(plain)
    ax_t.yaxis.set_minor_formatter(plain)
    ax_t.set_ylabel("θ / θ0 (after the update)")
    ax_t.legend(frameon=False, fontsize=8, ncol=3)

    def rolling(v):
        v = np.asarray(v, dtype=float)
        out = np.full_like(v, np.nan)
        for k in range(len(v)):
            window = v[max(0, k - smooth + 1):k + 1]
            out[k] = np.nanmean(window) if np.isfinite(window).any() else np.nan
        return out

    loss = [float(r.get("loss") or np.nan) for r in rows]
    ax_l.plot(upd, loss, lw=1, color=REF_COLOR, alpha=0.35, label="loss (per update)")
    ax_l.plot(upd, rolling(loss), lw=2, color=REF_COLOR, label=f"loss ({smooth}-update mean)")
    for i, term in enumerate(("L_n", "L_v", "L_jerk", "L_steer")):
        ax_l.plot(upd, rolling([float(r.get(term) or np.nan) for r in rows]), lw=2, color=SERIES[i], label=term)
    ax_l.set_ylabel("mean loss per data point")
    ax_l.set_xlabel("update (one per episode)")
    ax_l.legend(frameon=False, fontsize=8, ncol=3)
    for ax in (ax_t, ax_l):
        ax.grid(True, color=GRID_COLOR, lw=0.6)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ Algorithm 1 (static (one learned theta for whole loop))


def train(model, plant, mpc, cfg, out_dir):
    """Algorithm 1 (static or weights-varying, by the model). An update every cfg.accumulate_steps (the paper's N_batch: one episode, or a number of steps);
    stops after cfg.laps episodes."""
    optimizer = make_optimizer(model, cfg)
    feasible = FeasibleWeights(model)
    log = TrainingLog(dt=plant.dt)
    s0 = (
        cfg.s0 if cfg.starts == "fixed" else None
    )  # random: rollout restarts episodes at random points
    optimizer.zero_grad()  # line 2: ∇accum ← 0
    update, episodes, n_points, window_ok = 0, 0, 0, True
    for sd in rollout(
        plant, mpc, theta_fn=model, s0=s0, seed=cfg.seed, on_done="reset"
    ):  # lines 3, 5, 14
        loss_sum, n_valid, parts = step_loss(sd, cfg.loss_cfg)  # line 6
        if n_valid:
            loss_sum.backward()  # lines 7-9 
        n_points += n_valid
        window_ok &= bool(sd.mpc_info["ok"][sd.active].all())
        log.add_step(sd, loss_sum, n_valid, parts)
        episodes += int(sd.episode_end.sum())

        if is_update_step(sd, sd.k, cfg):  # line 10
            fallback_used = not window_ok
            if fallback_used:
                feasible.restore(model, optimizer)  # paper: back to the last feasible weights (fresh Adam state)
                optimizer.zero_grad()
                grad = None
            else:
                feasible.record(model)  # the w that just drove this clean window (before the update changes it)
                grad = optimizer_update(model, optimizer, cfg, max(n_points, 1))  # lines 11-12
            update += 1
            row = log.end_update(update, model, grad, fallback_used, cfg.grad_clip)
            print(f"update {update:4d}  loss {row.get('loss', np.nan):9.4f}  L_n {row.get('L_n', np.nan):8.4f}  "
                  f"L_v {row.get('L_v', np.nan):8.4f}  L_jerk {row.get('L_jerk', np.nan):8.4f}  "
                  f"L_steer {row.get('L_steer', np.nan):8.4f}  end {row['end_reasons'] or '-':>10}  "
                  f"lap {row['lap_time']:5.2f}  θ " + " ".join(f"{v:.4g}" for v in model.theta)
                  + ("  FALLBACK" if fallback_used else ""), flush=True)
            if update % cfg.checkpoint_every == 0:
                save_checkpoint(
                    out_dir / "checkpoints" / f"update_{update:04d}.pt",
                    model,
                    optimizer,
                    update,
                    log,
                )
            n_points, window_ok = 0, True
        if episodes >= cfg.laps:
            break  # with an int accumulate_steps, the gradient of a partial last window is dropped
    log.write(out_dir)
    if log.rows:
        plot_history(Path(out_dir) / "history.csv", Path(out_dir) / "training.png")
    return log


# ------------------------------------------------------------------ script


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    d = TrainConfig()
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument(
        "--laps", type=int, default=d.laps, help="training length in episodes (N_total)"
    )
    ap.add_argument(
        "--accumulate-steps",
        type=lambda v: v if v == "episode" else int(v),
        default=d.accumulate_steps,
        help='paper N_batch: "episode" (paper, static) or a number of control steps (paper Diff-WMPC: 10)',
    )
    ap.add_argument("--n-cars", type=int, default=d.n_cars)
    ap.add_argument("--lr", type=float, default=d.lr)
    ap.add_argument("--grad-clip", type=float, default=d.grad_clip)
    ap.add_argument("--starts", choices=("random", "fixed"), default=d.starts)
    ap.add_argument("--s0", type=float, default=d.s0)
    ap.add_argument("--seed", type=int, default=d.seed)
    ap.add_argument("--nlp-solver", choices=("SQP_RTI", "SQP"), default=d.nlp_solver)
    ap.add_argument("--max-iter", type=int, default=d.max_iter)
    ap.add_argument(
        "--weight-param",
        choices=WEIGHT_PARAMS,
        default=d.weight_param,
        help="θ = exp(w) (relative steps) or softplus(w) (the paper's Step 7 output)",
    )
    ap.add_argument(
        "--loss",
        choices=("balanced", "paper"),
        default="balanced",
        help="loss weights α…δ: balanced (provisional, steps.md 6.0) or the paper's Table I",
    )
    ap.add_argument("--model", choices=("static", "policy"), default=d.model,
                    help="static weights (Step 6) or a weight policy θ = π(track ahead) (Step 7)")
    ap.add_argument("--lr-net", type=float, default=d.lr_net,
                    help="policy: lr of the network except the last layer's bias (which uses --lr, as the static w)")
    ap.add_argument("--hidden-activation", choices=HIDDEN_ACTIVATIONS, default=d.hidden_activation)
    ap.add_argument("--beta", type=float, default=d.beta, help="policy: softplus β of the output map")
    ap.add_argument("--policy-inputs", choices=INPUTS, default=d.policy_inputs,
                    help="policy: all inputs, or speed / curvature only (7.7 ablations)")
    ap.add_argument("--policy-seed", type=int, default=d.policy_seed)
    ap.add_argument("--theta0", type=float, nargs=6, default=list(d.theta0),
                    metavar=("q_n", "q_psi", "q_v", "q_alat", "r_j", "r_omega"),
                    help="initial weights (default: ocp.THETA0)")
    ap.add_argument("--loss-importance", type=float, nargs=4, default=None, metavar=("w_n", "w_v", "w_jerk", "w_steer"),
                    help="balanced loss with these importances (α = w_n/σ_n², …; overrides --loss)")
    ap.add_argument("--loss-weights", type=float, nargs=4, default=None, metavar=("alpha", "beta", "gamma", "delta"),
                    help="explicit loss weights α β γ δ (overrides --loss and --loss-importance)")
    ap.add_argument("--checkpoint-every", type=int, default=d.checkpoint_every, help="updates between checkpoints")
    ap.add_argument("--tag", default="static")
    ap.add_argument("--out", type=Path, default=RL_FRAMEWORK / "data")
    return ap.parse_args()


def main():
    args = parse_args()
    cfg = TrainConfig(
        laps=args.laps,
        accumulate_steps=args.accumulate_steps,
        n_cars=args.n_cars,
        lr=args.lr,
        grad_clip=args.grad_clip,
        starts=args.starts,
        s0=args.s0,
        seed=args.seed,
        nlp_solver=args.nlp_solver,
        max_iter=args.max_iter,
        weight_param=args.weight_param,
        loss_cfg=make_loss_config(args),
        checkpoint_every=args.checkpoint_every,
        theta0=tuple(args.theta0),
        model=args.model,
        lr_net=args.lr_net,
        hidden_activation=args.hidden_activation,
        beta=args.beta,
        policy_inputs=args.policy_inputs,
        policy_seed=args.policy_seed,
    )
    out_dir = args.out / ("train_static" if cfg.model == "static" else "train_policy") / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)
    ref, plant, mpc = build_setup(args, cfg)
    model = build_model(cfg, ref)
    log = train(model, plant, mpc, cfg, out_dir)
    save_theta(out_dir / "theta.json", model, log, cfg)
    print(f"θ0 = {tuple(cfg.theta0)}\nθ  = {tuple(model.theta)}\nwritten: {out_dir}/")


if __name__ == "__main__":
    main()
