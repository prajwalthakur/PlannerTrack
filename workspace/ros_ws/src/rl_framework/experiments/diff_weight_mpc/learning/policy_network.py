# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Weight policy for Diff-WMPC (steps.md §10, 7.1): θ_k = π(o_k), the MPC cost weights from the track ahead.

o_k (B, 10) = 5 future reference speeds and 5 curvatures over the 2.55 s horizon (Reference.policy_observation),
as the paper (§III, Fig. 3), divided by fixed scales (observation_scale). Network 10 → 128 → 128 → 6 with hidden
activation softplus (paper) or none (linear hidden layers), then an output map that keeps θ > 0:

    "exact_softplus" (paper, default): θ = softplus_β(z)
    "relative_softplus":               θ = θ_start ⊙ softplus_β(z) / softplus_β(0)

softplus_β(z) = (1/β)·log(1 + e^(βz)). Both start exactly at θ_start for every input (last layer: weights 0, bias =
the output map's inverse at θ_start).
"""

import math

import numpy as np
import torch
from torch import nn

HIDDEN_ACTIVATIONS = ("softplus", "none")
OUTPUT_MAPS = ("exact_softplus", "relative_softplus")
INPUTS = ("all", "speed", "curvature")  # 7.7 ablations (paper Fig. 8): which half of the observation the policy sees


def softplus0(beta=1.0):
    """softplus_β(0) = log(2)/β."""
    return math.log(2.0) / beta


def inverse_softplus(theta, beta=1.0):
    """w with softplus_β(w) = θ, where softplus_β(w) = (1/β)·log(1 + e^(β·w)) (torch's ``softplus(w, beta)``).

    w = (1/β)·log(e^(βθ) - 1) = θ + (1/β)·log(1 - e^(−βθ)); the second form never computes e^(βθ) (no overflow
    for large θ) and uses expm1 (no cancellation for small θ). beta = 1 is the plain softplus.
    """
    theta = torch.as_tensor(theta, dtype=torch.float64)
    return theta + torch.log(-torch.expm1(-beta * theta)) / beta


def observation_scale(reference, n_samples=5):
    """(10,) fixed input scales over the whole lap: the reference's max speed for the 5 speeds, its max |κ| for the
    5 curvatures (signed κ stays signed: inputs in about [0.37, 1] and [-1, 0.37] on our track)."""
    v_max = float(np.max(reference.raceline.vx))
    kappa_max = float(np.max(np.abs(reference.raceline.kappa)))
    return np.r_[np.full(n_samples, v_max), np.full(n_samples, kappa_max)]


class WeightPolicy(nn.Module):
    """π: observation (B, 10) → θ (B, 6) > 0. Used as rollout's ``theta_fn(obs, k)`` (k ignored).

    Args:
        theta_start: (6,) the weights π outputs before training (for every observation).
        obs_scale: (10,) the observation is divided by it before the network (observation_scale(reference)).
        hidden: hidden layer sizes (paper: 128, 128).
        hidden_activation: "softplus" (paper) or "none" (linear hidden layers: the network is then an affine map of
            the observation followed by the output map).
        output_map: "exact_softplus" (paper) or "relative_softplus" (module docstring).
        beta: softplus sharpness of the output map; the bend between relative steps (θ ≪ 1/β) and absolute steps
            (θ ≫ 1/β) of the "exact_softplus" map is at θ ≈ 1/β.
        seed: initialization of the hidden layers (a local RNG: PyTorch's global random state is left untouched).
        inputs: "all" (paper), or "speed" / "curvature" only (the other half of the observation is set to 0; the
            ablations of 7.7, paper Fig. 8).

    Note: with the last layer's weights at 0, the first update moves only the last layer (the hidden layers'
    gradient passes through those weights); from the second update on every layer learns.
    """

    def __init__(self, theta_start, obs_scale, hidden=(128, 128), hidden_activation="softplus",
                 output_map="exact_softplus", beta=1.0, seed=0, inputs="all"):
        super().__init__()
        if hidden_activation not in HIDDEN_ACTIVATIONS:
            raise ValueError(f"hidden_activation must be one of {HIDDEN_ACTIVATIONS}, got {hidden_activation!r}")
        if output_map not in OUTPUT_MAPS:
            raise ValueError(f"output_map must be one of {OUTPUT_MAPS}, got {output_map!r}")
        if inputs not in INPUTS:
            raise ValueError(f"inputs must be one of {INPUTS}, got {inputs!r}")
        if beta <= 0:
            raise ValueError(f"beta must be > 0, got {beta}")
        self.config = {
            "theta_start": [float(v) for v in theta_start],
            "obs_scale": [float(v) for v in obs_scale],
            "hidden": list(hidden),
            "hidden_activation": hidden_activation,
            "output_map": output_map,
            "beta": float(beta),
            "seed": seed,
            "inputs": inputs,
        }
        self.output_map = output_map
        self.beta = float(beta)
        self.register_buffer("theta_start", torch.as_tensor(theta_start, dtype=torch.float64))
        self.register_buffer("obs_scale", torch.as_tensor(obs_scale, dtype=torch.float64))
        half = len(obs_scale) // 2  # observation = (5 speeds, 5 curvatures)
        mask = torch.ones(len(obs_scale), dtype=torch.float64)
        if inputs == "speed":
            mask[half:] = 0.0
        elif inputs == "curvature":
            mask[:half] = 0.0
        self.register_buffer("input_mask", mask)

        with torch.random.fork_rng():  # seeded initialization without changing the caller's random state
            torch.manual_seed(seed)
            layers, width = [], len(obs_scale)
            for size in hidden:
                layers += [nn.Linear(width, size, dtype=torch.float64),
                           nn.Softplus() if hidden_activation == "softplus" else nn.Identity()]
                width = size
            self.body = nn.Sequential(*layers)
            self.head = nn.Linear(width, len(theta_start), dtype=torch.float64)

        # exact start: z = bias for every observation, chosen so that the output map gives θ_start
        with torch.no_grad():
            self.head.weight.zero_()
            if output_map == "relative_softplus":
                self.head.bias.zero_()  # softplus_β(0) / softplus_β(0) = 1
            else:
                self.head.bias.copy_(inverse_softplus(self.theta_start, self.beta))

    def forward(self, obs, k=None):
        """θ (B, 6) float64 with graph to every parameter. ``obs`` (B, 10) numpy or torch; ``k`` ignored."""
        o = torch.as_tensor(obs, dtype=torch.float64) / self.obs_scale * self.input_mask
        z = self.head(self.body(o))
        theta = nn.functional.softplus(z, beta=self.beta)
        if self.output_map == "relative_softplus":
            return self.theta_start * theta / softplus0(self.beta)
        return theta

    # ---- save / load (train_mpc.py checkpoints; --policy-file in 7.3)

    def save(self, path):
        torch.save({"config": self.config, "state_dict": self.state_dict()}, path)

    @classmethod
    def load(cls, path):
        data = torch.load(path, weights_only=False)  # our own files
        policy = cls(**data["config"])
        policy.load_state_dict(data["state_dict"])
        return policy
