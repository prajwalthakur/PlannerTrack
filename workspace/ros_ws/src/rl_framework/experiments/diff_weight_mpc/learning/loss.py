# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""The Diff-WMPC task loss (paper eq. 4-5), evaluated on the MPC solution at shooting node 1.

    L = α·n² + β·(v − v_ref)² + γ·P_j(j) + δ·P_ω(ω)

with the threshold penalty P (paper eq. 5) made continuous at the threshold (steps.md §1.2: the paper's
form jumps from x_th² to exp(0) − 1 = 0 there):

    P(x) = x²                                   |x| <= x_th
           x_th² + exp(κ·(|x| − x_th)) − 1      |x| >  x_th

n and v − v_ref are taken at node 1 of the plan against the stage-1 reference (the paper's "sensitivity
shooting node index 1", Table I); j and ω are the inputs of node 1. Works on torch tensors (the
graph to theta is kept, for learning) and on numpy arrays (metrics).

Used by the closed-loop metrics (4.4) and by the training loop (Steps 6-7).
"""

from dataclasses import dataclass

import numpy as np
import torch

from diff_weight_mpc.mpc.model import NX


@dataclass(frozen=True)
class LossConfig:
    """Paper Table I / eq. 5. α…δ were found by Bayesian optimization for a full-size car; to be
    rescaled for ours in Step 6."""

    alpha: float = 2.5e-5  # lateral deviation
    beta: float = 2.25  # speed deviation
    gamma: float = 2.5e-7  # jerk penalty
    delta: float = 9e-3  # steering-rate penalty
    jerk_th: float = 8.0  # m/s^3
    kappa_j: float = 0.15
    steer_rate_th: float = 0.25  # rad/s
    kappa_omega: float = 0.8
    node: int = 1  # shooting node the loss is evaluated at


def threshold_penalty(x, threshold, kappa):
    """Continuous version of the paper's P (eq. 5): x² inside the threshold, exponential beyond."""
    a = torch.abs(x)
    outside = threshold**2 + torch.expm1(kappa * torch.clamp(a - threshold, min=0.0))
    return torch.where(a <= threshold, x**2, outside)


def node_loss(x_plan, u_plan, params, cfg=LossConfig()):
    """Loss per car at node ``cfg.node`` of the plans.

    Args:
        x_plan: (B, N+1, NX) planned states, u_plan: (B, N, NU) planned inputs (torch or numpy).
        params: the stage references of that solve, {name: (B, N+1, 1)} (MpcSolution.params).

    Returns:
        ``(loss (B,), parts)`` with ``parts`` = {"n", "dv", "jerk", "steer_rate"} (B,) at that node,
        torch tensors (graph kept if the inputs had one).
    """
    x = torch.as_tensor(x_plan)
    u = torch.as_tensor(u_plan, dtype=x.dtype)
    k = cfg.node
    ref = {name: torch.as_tensor(np.asarray(params[name])[:, k, 0], dtype=x.dtype)
           for name in ("x_ref", "y_ref", "psi_ref", "v_ref")}
    assert x.shape[-1] == NX
    xk = x[:, k]
    n = (-torch.sin(ref["psi_ref"]) * (xk[:, 0] - ref["x_ref"])
         + torch.cos(ref["psi_ref"]) * (xk[:, 1] - ref["y_ref"]))
    dv = xk[:, 3] - ref["v_ref"]
    jerk, steer_rate = u[:, k, 0], u[:, k, 1]
    loss = (cfg.alpha * n**2 + cfg.beta * dv**2
            + cfg.gamma * threshold_penalty(jerk, cfg.jerk_th, cfg.kappa_j)
            + cfg.delta * threshold_penalty(steer_rate, cfg.steer_rate_th, cfg.kappa_omega))
    return loss, {"n": n, "dv": dv, "jerk": jerk, "steer_rate": steer_rate}
