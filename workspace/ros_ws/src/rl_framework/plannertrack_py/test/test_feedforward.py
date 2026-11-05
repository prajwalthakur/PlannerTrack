# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Open-loop feed-forward along the reference (steps.md Step 3.6)."""

from pathlib import Path

import numpy as np
import pytest

from plannertrack_py import ReferenceTrack, compute_raceline
from plannertrack_py.feedforward import feedforward_schedule, run_open_loop
from plannertrack_py.plant import ACC, DELTA, FAILURE_FLAGS, Plant

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"


@pytest.fixture(scope="module")
def ref():
    return ReferenceTrack(compute_raceline(PARAMS / "agents.yaml", "agent_1", PARAMS / "min_curv_planner.yaml"))


@pytest.fixture(scope="module")
def plant(ref):
    return Plant.from_scenario(PARAMS, reference=ref, dt=0.1)  # the 3.6 sanity run's interval


def schedule(plant, s0, n=30, limited=True):
    lim = plant.limits
    kw = dict(max_jerk=lim.max_jerk, max_steer_rate=lim.max_steer_rate) if limited else {}
    return feedforward_schedule(plant.reference, plant.sim.xdot, plant.nominal_params.wheelbase,
                                lim.max_steer, s0, n, plant.dt, **kw)


def test_schedule_without_limits_reproduces_the_steady_states(plant):
    sch = schedule(plant, 3.0, limited=False)
    assert sch.converged.all()
    np.testing.assert_allclose(sch.acc, sch.acc_target)
    np.testing.assert_allclose(sch.delta, sch.delta_target)
    # the inputs integrate back to the stage values
    np.testing.assert_allclose(sch.acc[0] + np.cumsum(sch.u[:, 0]) * sch.dt, sch.acc[1:], atol=1e-12)
    np.testing.assert_allclose(sch.delta[0] + np.cumsum(sch.u[:, 1]) * sch.dt, sch.delta[1:], atol=1e-12)


def test_reference_is_not_jerk_limited_so_the_schedule_is_rate_limited(plant):
    """The speed profile switches between drive and braking within one stage: the raw schedule
    asks for far more than max_jerk. The limited one stays inside and catches up."""
    lim = plant.limits
    raw, limited = schedule(plant, 0.0, n=60, limited=False), schedule(plant, 0.0, n=60)
    assert np.abs(raw.u[:, 0]).max() > 2.0 * lim.max_jerk
    assert np.all(np.abs(limited.u[:, 0]) <= lim.max_jerk + 1e-9)
    assert np.all(np.abs(limited.u[:, 1]) <= lim.max_steer_rate + 1e-9)
    lag = np.abs(limited.acc - limited.acc_target)
    assert lag.max() > 0.5 and np.sum(lag < 1e-9) > 0.8 * len(lag)  # lags only briefly


def test_plant_follows_the_schedule_exactly(plant):
    """No clipping: the plant's acc and steer states are the scheduled ones."""
    sch = schedule(plant, 12.0, n=10)
    plant.reset(s0=12.0)
    run = run_open_loop(plant, sch)
    k = len(run["t"])
    np.testing.assert_allclose(run["states"][:, ACC], sch.acc[1:k + 1], atol=1e-9)
    np.testing.assert_allclose(run["states"][:, DELTA], sch.delta[1:k + 1], atol=1e-9)


def test_open_loop_stays_near_the_line_briefly_then_fails(plant):
    """Without feedback the nominal car follows the line for a moment, then drifts off: the
    correction the MPC has to provide (feedforward_lap.py plots this)."""
    sch = schedule(plant, 12.0, n=int(np.ceil(1.2 * plant.reference.raceline.lap_time / plant.dt)))
    plant.reset(s0=12.0)
    run = run_open_loop(plant, sch)
    early = run["t"] <= 0.5 + 1e-9
    assert np.abs(run["n"][early]).max() < 0.05
    assert run["reason"] not in ("lap_done", "schedule_end")
    assert all(f in FAILURE_FLAGS for f in run["reason"].split("+"))
    assert run["progress"][-1] < plant.reference.length
