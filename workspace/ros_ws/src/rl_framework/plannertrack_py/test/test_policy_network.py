# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Tests for the Diff-WMPC weight policy (plannertrack_py/learning/policy_network.py, steps.md §10 7.1)."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from diff_weight_mpc.learning.policy_network import (  # noqa: E402
    HIDDEN_ACTIVATIONS, OUTPUT_MAPS, WeightPolicy, inverse_softplus,
)

THETA_START = np.array([32.4, 0.041, 16.6, 0.030, 1.1e-5, 1.62])  # the best static θ (wide range on purpose)
SCALE = np.r_[np.full(5, 8.07), np.full(5, 0.967)]


def observations(n=4, seed=0):
    rng = np.random.default_rng(seed)
    return np.c_[rng.uniform(3.0, 8.0, (n, 5)), rng.uniform(-0.9, 0.35, (n, 5))]


@pytest.mark.parametrize("beta", [1.0, 0.5, 10.0])
def test_inverse_softplus_round_trip(beta):
    w = inverse_softplus(THETA_START, beta)
    np.testing.assert_allclose(torch.nn.functional.softplus(w, beta=beta).numpy(), THETA_START, rtol=1e-12)


@pytest.mark.parametrize("hidden_activation", HIDDEN_ACTIVATIONS)
@pytest.mark.parametrize("output_map", OUTPUT_MAPS)
@pytest.mark.parametrize("beta", [1.0, 0.1])
def test_starts_exactly_at_theta_start(hidden_activation, output_map, beta):
    policy = WeightPolicy(THETA_START, SCALE, hidden_activation=hidden_activation, output_map=output_map, beta=beta)
    theta = policy(observations(), 0)
    assert theta.shape == (4, 6) and theta.dtype == torch.float64
    np.testing.assert_allclose(theta.detach().numpy(), np.tile(THETA_START, (4, 1)), rtol=1e-12)


@pytest.mark.parametrize("hidden_activation", HIDDEN_ACTIVATIONS)
def test_gradient_reaches_every_layer(hidden_activation):
    policy = WeightPolicy(THETA_START, SCALE, hidden_activation=hidden_activation)
    obs = observations()
    policy(obs).sum().backward()
    assert policy.head.bias.grad.abs().sum() > 0
    assert policy.body[0].weight.grad.abs().sum() == 0  # zero-initialized head: no gradient to the body yet
    opt = torch.optim.Adam(policy.parameters(), lr=1e-3)
    opt.step()
    opt.zero_grad()
    policy(obs).sum().backward()
    assert all(q.grad is not None and q.grad.abs().sum() > 0 for q in policy.parameters())


def test_input_ablation_masks(tmp_path):
    obs = observations()
    for inputs, changed in (("speed", slice(5, 10)), ("curvature", slice(0, 5))):
        policy = WeightPolicy(THETA_START, SCALE, inputs=inputs)
        with torch.no_grad():
            for q in policy.parameters():
                q.add_(0.1 * torch.randn_like(q))  # perturbed, so that the output depends on the inputs
        other = obs.copy()
        other[:, changed] += 1.0  # change only the inputs the policy must ignore
        torch.testing.assert_close(policy(obs), policy(other))


def test_save_load_round_trip(tmp_path):
    policy = WeightPolicy(THETA_START, SCALE, hidden_activation="none", beta=0.5, inputs="speed", seed=3)
    with torch.no_grad():
        for q in policy.parameters():
            q.add_(0.05 * torch.randn_like(q))
    policy.save(tmp_path / "policy.pt")
    loaded = WeightPolicy.load(tmp_path / "policy.pt")
    assert loaded.config == policy.config
    torch.testing.assert_close(loaded(observations()), policy(observations()), rtol=0, atol=0)


def test_seeded_init_leaves_global_rng_alone():
    state = torch.random.get_rng_state()
    a = WeightPolicy(THETA_START, SCALE, seed=7)
    assert torch.equal(state, torch.random.get_rng_state())
    b = WeightPolicy(THETA_START, SCALE, seed=7)
    assert all(torch.equal(x, y) for x, y in zip(a.state_dict().values(), b.state_dict().values()))


def test_none_activation_is_affine_before_the_output_map():
    policy = WeightPolicy(THETA_START, SCALE, hidden_activation="none")
    with torch.no_grad():
        for q in policy.parameters():
            q.add_(0.1 * torch.randn_like(q))
        o1, o2 = (torch.as_tensor(x) / policy.obs_scale for x in (observations(seed=1), observations(seed=2)))
        mid = 0.5 * (o1 + o2)
        z = lambda o: policy.head(policy.body(o))  # noqa: E731
        torch.testing.assert_close(z(mid), 0.5 * (z(o1) + z(o2)))  # affine: the midpoint maps to the midpoint
