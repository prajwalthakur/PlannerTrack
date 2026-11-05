# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Plant: perturbed parameters, reset on the reference, step, episode flags (steps.md Steps 3.1-3.5)."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from plannertrack_py import ReferenceTrack, Sim, compute_raceline, load_agent_yaml
from plannertrack_py.vehicle import agent_with_vehicle_params
from plannertrack_py.plant import (
    ACC, DELTA, FAILURE_FLAGS, PERTURBABLE, MAX_DEV_FROM_REF, MIN_DEV_FROM_REF, MIN_SPEED, PSI, R, VX, VY, X, Y,
    Perturbation, PerturbationRange, Plant, StartNoise,
)

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
AGENTS, SIM_YAML, PLANNER = PARAMS / "agents.yaml", PARAMS / "sim.yaml", PARAMS / "min_curv_planner.yaml"


@pytest.fixture(scope="module")
def ref():
    return ReferenceTrack(compute_raceline(AGENTS, "agent_1", PLANNER))


def make(ref, **kwargs):
    """Plant with 0.1 s steps (these tests were written for that rate; the default is 0.02 s)."""
    return Plant.from_scenario(PARAMS, reference=ref, **{"dt": 0.1, **kwargs})


def random_states(n=20, seed=0):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        x = np.array([0.0, 0.0, rng.uniform(-3, 3), rng.uniform(3.5, 9.0), rng.uniform(-0.3, 0.3),
                      rng.uniform(-1.5, 1.5), rng.uniform(-0.3, 0.3), rng.uniform(-4, 4)])
        yield x, np.array([rng.uniform(-10, 10), rng.uniform(-1, 1)])


# ------------------------------------------------------------------ 3.1 perturbations


def test_defaults_are_nominal(ref):
    plant = Plant.from_scenario(PARAMS, reference=ref)
    assert plant.perturbation.is_nominal
    assert plant.true_params == plant.nominal_params
    assert plant.sim.dynamics_plugin == "SingleTrackAugDynModel"
    assert plant.dt == pytest.approx(0.02) and plant.substeps == 2  # the paper's control rate


@pytest.mark.parametrize("pert", [Perturbation(1.2, 1.0), Perturbation(1.0, 0.8), Perturbation(0.9, 1.1),
                                  Perturbation(Iz_scale=1.3), Perturbation(cf_scale=0.8, cr_scale=1.25),
                                  Perturbation(1.2, 1.0, 1.2, 1 / 1.2, 1 / 1.2)])
def test_perturbation_reaches_the_physics(ref, pert):
    """The perturbed plant equals a Sim built by hand with the same mass/mu, and differs from
    the nominal one."""
    plant = make(ref, perturbation=pert)
    nom = plant.nominal_params
    for name, s in pert.scales().items():
        assert getattr(plant.true_params, name) == pytest.approx(s * getattr(nom, name))
    assert (plant.true_params.lf, plant.true_params.lr, plant.true_params.hcg) == (nom.lf, nom.lr, nom.hcg)  # not perturbable

    agent = yaml.safe_load(load_agent_yaml(AGENTS, "agent_1", "SingleTrackAugDynModel"))
    by_hand = Sim(SIM_YAML.read_text(), yaml.safe_dump(agent_with_vehicle_params(agent, plant.true_params)))
    nominal = make(ref).sim
    for x, u in random_states():
        np.testing.assert_array_equal(plant.sim.xdot(x, u), by_hand.xdot(x, u))
    assert any(not np.allclose(plant.sim.xdot(x, u), nominal.xdot(x, u)) for x, u in random_states())


def test_set_perturbation_rebuilds_the_plant_and_keeps_the_state(ref):
    plant = make(ref)
    x0, _ = plant.reset(seed=0)
    x, u = next(random_states())
    before = plant.sim.xdot(x, u)
    plant.set_perturbation(Perturbation(mu_scale=0.8))
    np.testing.assert_array_equal(plant.sim.get_state(), x0)
    assert plant.true_params.mu == pytest.approx(0.8 * plant.nominal_params.mu)
    assert not np.allclose(plant.sim.xdot(x, u), before)
    plant.set_perturbation(Perturbation())
    np.testing.assert_array_equal(plant.sim.xdot(x, u), before)


def test_reset_can_set_the_perturbation(ref):
    plant = make(ref)
    _, info = plant.reset(seed=0, perturbation=Perturbation(mass_scale=1.15))
    assert info["perturbation"] == {"mass_scale": 1.15, "mu_scale": 1.0, "Iz_scale": 1.0, "cf_scale": 1.0,
                                    "cr_scale": 1.0, "max_accel_scale": 1.0}
    _, info = plant.reset(seed=1)  # kept
    assert plant.perturbation.mass_scale == 1.15 and info["perturbation"]["mass_scale"] == 1.15


def test_perturbation_range_draws_per_reset_and_is_seeded(ref):
    rng_range = PerturbationRange(mass_scale=(0.8, 1.2), mu_scale=(0.85, 1.0))
    plant = make(ref, perturbation_range=rng_range)
    draws = [plant.reset(seed=s)[1]["perturbation"] for s in range(10)]
    for d in draws:
        assert 0.8 <= d["mass_scale"] <= 1.2 and 0.85 <= d["mu_scale"] <= 1.0
    assert len({d["mass_scale"] for d in draws}) == 10  # a new draw each episode
    again = make(ref, perturbation_range=rng_range)
    assert [again.reset(seed=s)[1]["perturbation"] for s in range(10)] == draws
    # an explicit perturbation wins over the range
    _, info = plant.reset(seed=0, perturbation=Perturbation(mu_scale=0.9))
    assert info["perturbation"] == {**{f"{n}_scale": 1.0 for n in PERTURBABLE}, "max_accel_scale": 1.0,
                                    "mu_scale": 0.9}


def test_every_parameter_can_be_drawn_and_fixed_ones_take_no_draw():
    rng_range = PerturbationRange(Iz_scale=(0.8, 1.2), cf_scale=(0.9, 1.0), cr_scale=(1.0, 1.1))
    draws = [rng_range.sample(np.random.default_rng(s)) for s in range(20)]
    for d in draws:
        assert d.mass_scale == 1.0 and d.mu_scale == 1.0  # fixed: no draw
        assert 0.8 <= d.Iz_scale <= 1.2 and 0.9 <= d.cf_scale <= 1.0 and 1.0 <= d.cr_scale <= 1.1
    assert len({d.Iz_scale for d in draws}) == 20
    # adding a range for one parameter does not change the draws of the others
    a = PerturbationRange(mu_scale=(0.8, 1.0)).sample(np.random.default_rng(5))
    b = PerturbationRange(mu_scale=(0.8, 1.0), Iz_scale=(1.0, 1.0)).sample(np.random.default_rng(5))
    assert a == b


def test_invalid_inputs():
    with pytest.raises(ValueError):
        Perturbation(mass_scale=0.0)
    with pytest.raises(ValueError):
        Perturbation(cr_scale=-1.0)
    with pytest.raises(ValueError):
        Plant.from_scenario(PARAMS, reference=None, dt=0.105)


# ------------------------------------------------------------------ 3.2 reset


@pytest.mark.parametrize("s0", [0.0, 3.0, 7.5, 12.0])
def test_reset_lands_on_the_reference(ref, s0):
    plant = make(ref)
    x, info = plant.reset(s0=s0)
    assert info["trim_converged"]
    r = ref.sample(s0)
    proj = ref.project(x[0], x[1])
    assert proj.s == pytest.approx(s0, abs=1e-6) and abs(proj.n) < 1e-9
    assert x[VX] == pytest.approx(float(r["vx"]))
    assert x[R] == pytest.approx(float(r["vx"] * r["kappa"]))
    course = x[PSI] + np.arctan2(x[VY], x[VX])  # velocity direction
    assert np.angle(np.exp(1j * (course - float(r["psi"])))) == pytest.approx(0.0, abs=1e-12)
    # trimmed: no lateral transient, and vx_dot equals the reference's a_x
    f = plant.sim.xdot(x, np.zeros(2))
    assert abs(f[VY]) < 1e-7 and abs(f[R]) < 1e-7
    assert f[VX] == pytest.approx(float(r["ax"]), abs=1e-9)
    assert abs(x[DELTA]) <= plant.limits.max_steer


def test_trim_shows_understeer_and_load_transfer(ref):
    """Physics of the trim: at constant speed the car understeers (cf < cr with lf = lr), so it
    needs more steer than the kinematic angle; braking moves load to the front (more oversteer,
    less steer), accelerating to the rear (more steer). Hard braking on a near-straight can even
    need counter-steer, which is why the start state is trimmed rather than kinematic."""
    plant = make(ref)
    vx, kappa = 5.0, 0.2  # a_lat = 5 m/s^2, well inside the grip
    kin = np.arctan(kappa * plant.nominal_params.wheelbase)
    d_brake, _, _, ok_b = plant.trim(vx, kappa, -4.0)
    d_const, vy, _, ok_c = plant.trim(vx, kappa, 0.0)
    d_accel, _, _, ok_a = plant.trim(vx, kappa, 4.0)
    assert ok_b and ok_c and ok_a
    assert d_const > kin > 0.0
    assert d_brake < d_const < d_accel
    assert vy < 0.0  # left turn: the body points into the corner, velocity to its right


def test_start_holds_the_line_for_a_moment(ref):
    """Started in trim, 0.1 s with zero input stays within 1 cm of the line (a kinematic start
    with vy = 0 is noticeably worse)."""
    plant = make(ref)
    worst = 0.0
    for s0 in np.linspace(0.0, ref.length, 12, endpoint=False):
        x, _ = plant.reset(s0=s0)
        x = plant.sim.step(np.zeros(2), n=plant.substeps)
        worst = max(worst, abs(ref.project(x[0], x[1], s_guess=s0, window=3.0).n))
    assert worst < 0.01


def test_same_seed_same_start_different_seed_different_start(ref):
    noise = StartNoise(n_std=0.1, psi_std=0.05, v_std=0.3)
    a, b = make(ref, start_noise=noise), make(ref, start_noise=noise)
    xa, ia = a.reset(seed=7)
    xb, ib = b.reset(seed=7)
    np.testing.assert_array_equal(xa, xb)
    assert ia == ib
    xc, ic = a.reset(seed=8)
    assert ic["s0"] != ia["s0"] and not np.allclose(xc, xa)
    # seed=None continues the stream instead of repeating it
    a.reset(seed=7)
    assert a.reset()[1]["s0"] != ia["s0"]


def test_start_noise_is_applied(ref):
    plant = make(ref, start_noise=StartNoise(n_std=0.1, psi_std=0.05, v_std=0.3))
    for seed in range(5):
        x, info = plant.reset(seed=seed)
        r = ref.sample(info["s0"])
        proj = ref.project(x[0], x[1], s_guess=info["s0"], window=2.0)
        assert proj.n == pytest.approx(info["n0"], abs=1e-3)  # projection of a curved line: ~exact
        assert x[VX] == pytest.approx(float(r["vx"]) + info["dv0"])
        course = x[PSI] + np.arctan2(x[VY], x[VX])
        assert np.angle(np.exp(1j * (course - float(r["psi"])))) == pytest.approx(info["dchi0"], abs=1e-12)
    assert any(info["n0"] != 0.0 for info in [plant.reset(seed=s)[1] for s in range(3)])


def test_no_trim_when_mu_is_too_low(ref):
    """At the corner apex the reference needs a_lat ~ 0.82 * mu_nom * g; with 70 % of mu there is
    no steady state. reset() falls back to the kinematic start and reports it."""
    plant = make(ref, perturbation=Perturbation(mu_scale=0.7))
    s_apex = float(ref.raceline.s[np.argmax(np.abs(ref.raceline.a_lat))])
    x, info = plant.reset(s0=s_apex)
    assert not info["trim_converged"]
    assert x[VY] == 0.0
    assert x[DELTA] == pytest.approx(np.arctan(float(ref.sample(s_apex)["kappa"]) * plant.nominal_params.wheelbase))
    assert x[ACC] == pytest.approx(float(ref.sample(s_apex)["ax"]))


# ------------------------------------------------------------------ 3.2 start bounds


def test_noisy_starts_stay_physical(ref):
    """Even with absurd stds every start is inside the track, points forward along it, has a
    valid speed and a feasible trim (the draws are truncated and infeasible ones redrawn)."""
    noise = StartNoise(n_std=1.0, psi_std=2.0, v_std=5.0)
    plant = make(ref, start_noise=noise)
    half = 0.5 * plant.limits.vehicle_width
    for seed in range(60):
        x, info = plant.reset(seed=seed)
        r = ref.sample(info["s0"])
        proj = ref.project(x[0], x[1], s_guess=info["s0"], window=2.0)
        assert -(float(r["w_right"]) - half - noise.track_margin) - 1e-3 <= proj.n
        assert proj.n <= float(r["w_left"]) - half - noise.track_margin + 1e-3
        assert abs(info["dchi0"]) <= noise.dpsi_max
        body_err = np.angle(np.exp(1j * (x[PSI] - float(r["psi"]))))
        assert abs(body_err) < np.deg2rad(45.0)  # heading error + sideslip: never sideways or backwards
        assert MIN_SPEED <= x[VX] <= plant.limits.v_max and abs(info["dv0"]) <= noise.dv_max
        assert info["trim_converged"] and abs(x[DELTA]) <= plant.limits.max_steer
        assert info["start_draws"] >= 1


def test_start_bounds_contain_zero_and_respect_the_limits(ref):
    plant = make(ref, start_noise=StartNoise(n_std=0.1, dv_max=2.0))
    for s0 in np.linspace(0.0, ref.length, 50, endpoint=False):
        b = plant.start_bounds(s0)
        v_ref = float(ref.sample(s0)["vx"])
        for lo, hi in b.values():
            assert lo <= 0.0 <= hi
        assert v_ref + b["dv"][0] >= MIN_SPEED - 1e-12 and v_ref + b["dv"][1] <= plant.limits.v_max


def test_offset_start_drives_the_parallel_curve(ref):
    """(At the tightest corner the reference is grip-limited: 5 cm inside has no trim and reset()
    would redraw; so use a corner with grip to spare.)"""
    plant = make(ref)
    rl = ref.raceline
    spare = (np.abs(rl.kappa) > 0.2) & (np.abs(rl.a_lat) < 0.6 * plant.limits.ay_max)
    s0 = float(rl.s[np.flatnonzero(spare)[0]])
    kappa = float(ref.sample(s0)["kappa"])
    dn = 0.05 * np.sign(kappa)  # towards the inside of the corner: tighter curve
    x, ok = plant.start_state(s0, dn=dn)
    assert ok and x[R] == pytest.approx(x[VX] * kappa / (1.0 - dn * kappa))
    with pytest.raises(ValueError):
        plant.start_state(s0, dn=2.0 / kappa)  # beyond the centre of curvature


def test_falls_back_to_the_reference_if_no_noisy_start_is_feasible(ref):
    plant = make(ref, perturbation=Perturbation(mu_scale=0.7),
                 start_noise=StartNoise(n_std=0.05, psi_std=0.05, v_std=0.2))
    s_apex = float(ref.raceline.s[np.argmax(np.abs(ref.raceline.a_lat))])
    x, info = plant.reset(seed=0, s0=s_apex)
    assert info["start_draws"] == -1 and not info["trim_converged"]
    assert (info["n0"], info["dchi0"], info["dv0"]) == (0.0, 0.0, 0.0)
    assert x[VX] == pytest.approx(float(ref.sample(s_apex)["vx"]))


def test_invalid_start_noise():
    with pytest.raises(ValueError):
        StartNoise(dpsi_max=-0.1)
    with pytest.raises(ValueError):
        StartNoise(n_std=-0.1)


def test_heading_deviation_stays_within_the_limits(ref):
    """A dpsi_max beyond +-60 deg is cut to [MIN_DEV_FROM_REF, MAX_DEV_FROM_REF], and the body
    heading (heading error minus sideslip) stays inside that range too."""
    assert MIN_DEV_FROM_REF == pytest.approx(-np.pi / 3) and MAX_DEV_FROM_REF == pytest.approx(np.pi / 3)
    plant = make(ref, start_noise=StartNoise(psi_std=3.0, dpsi_max=np.pi))
    assert plant.start_bounds(0.0)["dpsi"] == (MIN_DEV_FROM_REF, MAX_DEV_FROM_REF)
    devs = []
    for seed in range(100):
        x, info = plant.reset(seed=seed)
        assert MIN_DEV_FROM_REF <= info["dchi0"] <= MAX_DEV_FROM_REF
        dev = plant.heading_deviation(x, info["s0"])
        assert MIN_DEV_FROM_REF <= dev <= MAX_DEV_FROM_REF
        devs.append(dev)
    assert max(np.abs(devs)) > np.deg2rad(45.0)  # the range is actually used


# ------------------------------------------------------------------------ 3.3 step


def test_step_is_one_interval_of_the_sim(ref):
    """step(u) = substeps Sim steps with u held, from the same state, on the true plant."""
    plant = make(ref, perturbation=Perturbation(1.1, 0.9))
    x0, _ = plant.reset(seed=0)
    agent = yaml.safe_load(load_agent_yaml(AGENTS, "agent_1", "SingleTrackAugDynModel"))
    by_hand = Sim(SIM_YAML.read_text(), yaml.safe_dump(agent_with_vehicle_params(agent, plant.true_params)))
    by_hand.set_state(x0)
    u = np.array([3.0, -0.4])
    for k in range(1, 4):
        x, info = plant.step(u)
        np.testing.assert_array_equal(x, by_hand.step(u, n=plant.substeps))
        assert info["t"] == pytest.approx(k * plant.dt)


def test_step_leaves_input_limits_to_the_cpp_plant(ref):
    """u is passed through; the C++ plant (clip_control_in_sim) clips it to vehicle_limits."""
    a, b = make(ref), make(ref)
    a.reset(seed=1)
    b.reset(seed=1)
    lim = a.limits
    xa, _ = a.step([10.0 * lim.max_jerk, -10.0 * lim.max_steer_rate])
    xb, _ = b.step([lim.max_jerk, -lim.max_steer_rate])
    np.testing.assert_array_equal(xa, xb)
    assert -lim.max_steer <= xa[DELTA] <= lim.max_steer
    assert -lim.max_decel <= xa[ACC] <= lim.max_accel


@pytest.mark.parametrize("s0", [0.0, 5.0, 11.3, 20.0])
def test_step_info_on_the_reference(ref, s0):
    """Started in trim with zero input, one step stays on the line: small Frenet errors, the
    progress is about v_ref * dt and dpsi is about minus the sideslip (velocity, not body, on
    the line; zero input does not follow curvature changes, hence the loose bound)."""
    plant = make(ref)
    plant.reset(s0=s0)
    x, info = plant.step(np.zeros(2))
    v0 = float(ref.sample(s0)["vx"])
    assert abs(info["n"]) < 0.01
    assert abs(info["dv"]) < 0.05 * v0
    assert info["ds"] == pytest.approx(v0 * plant.dt, rel=0.05)
    assert info["progress"] == info["ds"] == plant.progress
    assert info["dpsi"] == pytest.approx(plant.heading_deviation(x, info["s"]))
    assert info["dpsi"] == pytest.approx(-np.arctan2(x[VY], x[VX]), abs=0.05)
    assert info["v_ref"] == pytest.approx(float(ref.sample(info["s"])["vx"]))
    assert info["dv"] == pytest.approx(x[VX] - info["v_ref"])
    p = ref.project(x[0], x[1])
    assert (info["s"], info["n"]) == pytest.approx((p.s, p.n))


def test_progress_continues_across_the_start_line(ref):
    plant = make(ref)
    s0 = ref.length - 0.3
    plant.reset(s0=s0)
    total = 0.0
    for _ in range(3):
        _, info = plant.step(np.zeros(2))
        assert info["ds"] > 0.0
        total += info["ds"]
    assert info["s"] < s0  # wrapped into [0, length)
    assert info["progress"] == pytest.approx(total)
    assert info["progress"] == pytest.approx((info["s"] - s0) % ref.length)
    assert info["progress"] > 0.3
    plant.reset(s0=s0)  # reset clears progress and time
    assert plant.progress == 0.0 and plant.step(np.zeros(2))[1]["t"] == pytest.approx(plant.dt)


def test_same_seed_same_rollout(ref):
    noise = StartNoise(n_std=0.1, psi_std=0.05, v_std=0.3)
    rng = np.random.default_rng(3)
    inputs = rng.uniform([-10.0, -1.0], [10.0, 1.0], size=(5, 2))
    runs = []
    for _ in range(2):
        plant = make(ref, start_noise=noise, perturbation_range=PerturbationRange((0.9, 1.1), (0.9, 1.1)))
        plant.reset(seed=11)
        runs.append([plant.step(u) for u in inputs])
    for (xa, ia), (xb, ib) in zip(*runs):
        np.testing.assert_array_equal(xa, xb)
        assert ia == ib


def test_step_with_a_non_finite_state(ref):
    plant = make(ref)
    plant.reset(s0=3.0)
    plant.step(np.zeros(2))
    progress = plant.progress
    bad = plant.sim.get_state()
    bad[VY] = np.nan
    plant.sim.set_state(bad)
    x, info = plant.step(np.zeros(2))
    assert not np.all(np.isfinite(x))
    assert np.isnan(info["n"]) and np.isnan(info["s"]) and info["progress"] == progress
    assert info["diverged"] and info["terminated"]
    assert not (info["off_track"] or info["too_slow"] or info["heading_out"] or info["lap_done"])


def test_step_rejects_bad_calls(ref):
    plant = make(ref)
    with pytest.raises(RuntimeError):
        plant.step(np.zeros(2))
    plant.reset(seed=0)
    with pytest.raises(ValueError):
        plant.step(np.zeros(3))


def test_info_has_body_and_course_heading_errors(ref):
    """dpsi (body, the paper's cost term), dchi (course) and beta (sideslip): dpsi = dchi - beta.
    On a trimmed start in a corner the course error is 0 and the body error is -beta != 0."""
    plant = make(ref)
    x, info = plant.reset(s0=5.48)  # left corner, a_lat ~ 7.4 m/s^2 (the hairpin apex has no trim)
    assert info["trim_converged"] and info["dchi0"] == 0.0
    assert info["beta0"] == pytest.approx(np.arctan2(x[VY], x[VX]))
    assert info["dpsi0"] == pytest.approx(-info["beta0"], abs=1e-9)
    assert abs(info["beta0"]) > np.deg2rad(2.0)  # cornering needs sideslip
    for _ in range(3):
        x, info = plant.step(np.zeros(2))
        assert info["dpsi"] == pytest.approx(plant.heading_deviation(x, info["s"]))
        assert info["beta"] == pytest.approx(np.arctan2(x[VY], x[VX]))
        assert info["dpsi"] == pytest.approx(info["dchi"] - info["beta"], abs=1e-12)


def test_every_reference_point_has_a_trim(ref):
    """The reference is drivable by the nominal plant everywhere: a steady cornering state exists at
    every raceline point (within max_steer). Guards the scenario's limits: with friction_margin 0.85
    the hairpin apex (R = 1.03 m) had none (steps.md §5)."""
    plant = make(ref)
    failed = [round(float(s), 2) for s in ref.raceline.s if not plant.reset(s0=s)[1]["trim_converged"]]
    assert failed == [], f"no steady cornering state at s = {failed}"
    assert ref.raceline.vx.min() >= MIN_SPEED  # and the slowest point is inside the valid speed range


def test_mpc_model_trims_like_the_plant_on_the_reference(ref):
    """The MPC's model (CasADi, purely dynamic) can drive every reference point too, with the
    same steady state as the plant. With the plant's old 1-3 m/s blend band they differed at the
    hairpin apex (2.954 m/s) by the blend's kinematic share (steady steer -0.344 vs -0.348)."""
    casadi = pytest.importorskip("casadi")  # noqa: F841
    from diff_weight_mpc.mpc.model import MIN_VALID_SPEED, dynamics_function, parameter_values
    from plannertrack_py.plant import steady_cornering

    assert MIN_SPEED == MIN_VALID_SPEED
    plant = make(ref)
    vp, lim = plant.nominal_params, plant.limits
    f = dynamics_function(vp)
    model_xdot = lambda x, u: np.asarray(f(x, u, parameter_values(vp))).ravel()  # noqa: E731
    for s in ref.raceline.s:
        r = ref.sample(s)
        args = (float(r["vx"]), float(r["kappa"]), float(r["ax"]), vp.wheelbase, lim.max_steer)
        d_model, vy_model, _, ok_model = steady_cornering(model_xdot, *args)
        d_plant, vy_plant, _, ok_plant = steady_cornering(plant.sim.xdot, *args)
        assert ok_model and ok_plant, f"s = {s:.2f}"
        assert d_model == pytest.approx(d_plant, abs=1e-6) and vy_model == pytest.approx(vy_plant, abs=1e-6)


# ------------------------------------------------------------------ 3.4 episode flags


def _flags(info):
    return {k: info[k] for k in (*FAILURE_FLAGS, "terminated", "lap_done")}


def _step_from(plant, state):
    plant.sim.set_state(state)
    return plant.step(np.zeros(2))[1]


@pytest.mark.parametrize("s0", [0.0, 5.48, 12.0, 20.0])
def test_no_flags_on_the_reference(ref, s0):
    plant = make(ref)
    plant.reset(s0=s0)
    for _ in range(3):
        x, info = plant.step(np.zeros(2))
        assert not any(_flags(info).values()), _flags(info)
        assert not (info["input_clipped"] or info["steer_at_limit"] or info["acc_at_limit"])


def test_track_limits_are_the_bounds_minus_half_the_car(ref):
    plant = make(ref)
    half = 0.5 * plant.limits.vehicle_width
    for s in np.linspace(0.0, ref.length, 20, endpoint=False):
        r = ref.sample(s)
        n_left, n_right = plant.track_limits(s)
        assert n_left == pytest.approx(float(r["w_left"]) - half)
        assert n_right == pytest.approx(-(float(r["w_right"]) - half))
        assert n_right < 0.0 < n_left


@pytest.mark.parametrize("side", [+1, -1])
def test_off_track(ref, side):
    """Shifted 30 cm past the car-side limit (left or right; the track widens by up to ~16 cm within
    the step's 0.5 m of travel), the step reports off_track."""
    plant = make(ref)
    s0 = 12.0
    x, _ = plant.reset(s0=s0)
    psi = float(ref.sample(s0)["psi"])
    n_left, n_right = plant.track_limits(s0)
    shift = (n_left + 0.3) if side > 0 else (n_right - 0.3)
    x = x.copy()
    x[X] -= shift * np.sin(psi)
    x[Y] += shift * np.cos(psi)
    info = _step_from(plant, x)
    assert info["off_track"] and info["terminated"] and not info["diverged"]


def test_too_slow(ref):
    plant = make(ref)
    x, _ = plant.reset(s0=12.0)
    x = x.copy()
    x[VX] = MIN_SPEED - 0.3
    info = _step_from(plant, x)
    assert info["too_slow"] and info["terminated"]
    assert not info["off_track"]


@pytest.mark.parametrize("turn", [+1, -1])
def test_heading_out(ref, turn):
    """Body yawed 85 deg away from the reference (beyond the +-60 deg range even after the yaw rate
    turns it back during the 0.1 s step)."""
    plant = make(ref)
    x, _ = plant.reset(s0=12.0)
    x = x.copy()
    x[PSI] += turn * np.deg2rad(85.0)
    info = _step_from(plant, x)
    assert info["heading_out"] and info["terminated"]
    assert not MIN_DEV_FROM_REF <= info["dpsi"] <= MAX_DEV_FROM_REF


def test_lap_done_is_not_a_failure(ref):
    plant = make(ref)
    plant.reset(s0=12.0)
    plant._progress = ref.length - 0.01  # as if almost one lap had been driven
    _, info = plant.step(np.zeros(2))
    assert info["lap_done"] and not info["terminated"]


def test_actuator_flags(ref):
    plant = make(ref)
    assert plant.clips_control  # the RL scenario enables clip_control_in_sim
    lim = plant.limits
    plant.reset(s0=12.0)
    _, info = plant.step([2.0 * lim.max_jerk, 0.0])
    assert info["input_clipped"] and not info["steer_at_limit"]
    _, info = plant.step([lim.max_jerk, lim.max_steer_rate])  # within the limits: not clipped
    assert not info["input_clipped"]
    for _ in range(3):  # 0.3 s at max rate: steer 0.96 rad requested, held at 0.698
        _, info = plant.step([lim.max_jerk, lim.max_steer_rate])
    assert info["steer_at_limit"] and info["acc_at_limit"]
    plant.clips_control = False  # without clip_control_in_sim nothing is reported
    _, info = plant.step([2.0 * lim.max_jerk, 0.0])
    assert not (info["input_clipped"] or info["steer_at_limit"] or info["acc_at_limit"])
