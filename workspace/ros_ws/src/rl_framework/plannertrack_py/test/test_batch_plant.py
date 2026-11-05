# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""BatchPlant: B cars on one reference, equal to B individual Plants."""

from pathlib import Path

import numpy as np
import pytest

from plannertrack_py import ReferenceTrack, compute_raceline
from plannertrack_py.batch_plant import BatchPlant
from plannertrack_py.plant import NU, NX, Perturbation, PerturbationRange, Plant, StartNoise

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
B = 4


@pytest.fixture(scope="module")
def ref():
    return ReferenceTrack(compute_raceline(PARAMS / "agents.yaml", "agent_1", PARAMS / "min_curv_planner.yaml"))


def inputs(seed, n_steps):
    rng = np.random.default_rng(seed)
    return rng.uniform([-10.0, -1.0], [10.0, 1.0], size=(n_steps, B, NU))


def test_shapes_and_shared_reference(ref):
    batch = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, dt=0.02)
    assert len(batch) == B and batch.dt == pytest.approx(0.02)
    assert all(p.reference is ref for p in batch.plants)
    x, info = batch.reset(seed=0)
    assert x.shape == (B, NX) and info["s0"].shape == (B,)
    x, info = batch.step(np.zeros((B, NU)))
    assert x.shape == (B, NX)
    for key in ("s", "n", "dpsi", "dv", "progress", "terminated", "lap_done", "off_track"):
        assert info[key].shape == (B,), key
    assert info["terminated"].dtype == bool
    np.testing.assert_array_equal(batch.states(), x)


def test_batch_equals_individual_plants(ref):
    """Car i of a batch reset with seed 7 is a single Plant reset with the i-th child seed: same
    start, same perturbation draw, same trajectory."""
    kw = dict(dt=0.02, start_noise=StartNoise(n_std=0.05, psi_std=0.05, v_std=0.2),
              perturbation_range=PerturbationRange((0.9, 1.1), (0.9, 1.0)))
    batch = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, **kw)
    singles = [Plant.from_scenario(PARAMS, reference=ref, **kw) for _ in range(B)]
    xb, ib = batch.reset(seed=7)
    for i, (plant, child) in enumerate(zip(singles, np.random.SeedSequence(7).spawn(B))):
        xi, ii = plant.reset(seed=child)
        np.testing.assert_array_equal(xb[i], xi)
        assert ib["s0"][i] == ii["s0"] and ib["perturbation"][i] == ii["perturbation"]
    for u in inputs(1, 10):
        xb, ib = batch.step(u)
        for i, plant in enumerate(singles):
            xi, ii = plant.step(u[i])
            np.testing.assert_array_equal(xb[i], xi)
            assert ib["n"][i] == ii["n"] and ib["terminated"][i] == ii["terminated"]


def test_cars_are_independent_and_seeded(ref):
    a = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, dt=0.02)
    b = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, dt=0.02)
    _, ia = a.reset(seed=3)
    _, ib = b.reset(seed=3)
    np.testing.assert_array_equal(ia["s0"], ib["s0"])  # reproducible from one seed
    assert len(set(ia["s0"].tolist())) == B  # but every car starts elsewhere
    _, ic = a.reset(seed=4)
    assert not np.array_equal(ic["s0"], ia["s0"])


def test_per_car_perturbations(ref):
    perts = [Perturbation(1.0, 1.0), Perturbation(1.2, 1.0), Perturbation(1.0, 0.8), Perturbation(0.9, 1.1)]
    batch = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, perturbations=perts)
    assert batch.perturbations == perts
    x, _ = batch.reset(s0=12.0)
    # Off the steady state: at a trimmed state the mass cancels (tire forces scale with mu*F_z, i.e.
    # with m), so a heavier car only differs in transients, through I_z / m.
    off_trim = x[0].copy()
    off_trim[5] += 0.5  # yaw rate
    xdots = [p.sim.xdot(off_trim, np.array([1.0, 0.2])) for p in batch.plants]  # same state, different cars
    assert all(not np.allclose(xdots[0], f) for f in xdots[1:])
    batch.set_perturbations([Perturbation()] * B)
    assert all(p.perturbation.is_nominal for p in batch.plants)


def test_reset_cars_leaves_the_others_running(ref):
    batch = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, dt=0.02)
    batch.reset(seed=0, s0=[2.0, 6.0, 10.0, 14.0])
    for _ in range(5):
        x, info = batch.step(np.zeros((B, NU)))
    progress = info["progress"].copy()
    x_new, info_new = batch.reset_cars([1, 3], s0=[20.0, 22.0])
    assert x_new.shape == (2, NX) and list(info_new["s0"]) == [20.0, 22.0]
    states = batch.states()
    np.testing.assert_array_equal(states[[0, 2]], x[[0, 2]])  # untouched
    np.testing.assert_array_equal(states[[1, 3]], x_new)
    _, info = batch.step(np.zeros((B, NU)))
    assert info["progress"][0] > progress[0] and info["progress"][1] < progress[1]  # 1 restarted


def test_bad_calls(ref):
    with pytest.raises(ValueError):
        BatchPlant.from_scenario(PARAMS, n_cars=0, reference=ref)
    with pytest.raises(ValueError):
        BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref, perturbations=[Perturbation()])
    batch = BatchPlant.from_scenario(PARAMS, n_cars=B, reference=ref)
    batch.reset(seed=0)
    with pytest.raises(ValueError):
        batch.step(np.zeros((B - 1, NU)))
    with pytest.raises(ValueError):
        batch.reset(s0=[1.0, 2.0])
