# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""acados integrator from the CasADi model (steps.md Step 2.6).

In the MPC, acados discretizes the model itself (ERK/RK4 in generated C code). These tests run that
exact path once, in isolation: code generation and compilation, the parameter vector p = [mass, mu, Iz, cf, cr],
and the integrator settings. They also measure the discretization error of one MPC interval for
different numbers of RK4 sub-steps, to choose sim_method_num_steps in Step 4.
"""

from pathlib import Path

import numpy as np
import pytest
import yaml

acados_template = pytest.importorskip("acados_template")

from plannertrack_py import Sim, load_agent_yaml, load_raceline_limits  # noqa: E402
from plannertrack_py.vehicle import agent_with_vehicle_params, load_vehicle_params  # noqa: E402
from diff_weight_mpc.mpc.acados_model import export_acados_model, parameter_values  # noqa: E402
from diff_weight_mpc.mpc.model import MIN_VALID_SPEED  # noqa: E402

PARAMS = Path(__file__).resolve().parents[3] / "scenarios" / "ground_vehicle_racing_rl" / "params"
AGENTS, SIM_YAML, PLANNER = PARAMS / "agents.yaml", PARAMS / "sim.yaml", PARAMS / "min_curv_planner.yaml"
PLANT_SUBSTEP = 0.001  # s, the plant's RK4 step (agents.yaml integrationStepSize)
PLANT_DT = 0.01  # s, one Sim.step (sim.yaml simTimeStep)


@pytest.fixture(scope="module")
def vp():
    return load_vehicle_params(AGENTS, "agent_1")


@pytest.fixture(scope="module")
def integrator(vp, tmp_path_factory):
    """One compiled acados RK4 integrator (4 stages, 1 step per call). T and p are set per call,
    so k calls with T/k equal one call with num_steps = k."""
    out = tmp_path_factory.mktemp("acados_sim")
    sim = acados_template.AcadosSim()
    sim.model = export_acados_model(vp)
    sim.parameter_values = parameter_values(vp)
    sim.solver_options.T = PLANT_SUBSTEP
    sim.solver_options.integrator_type = "ERK"
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 1
    sim.code_gen_opts.code_export_directory = str(out / "c_generated_code")
    sim.code_gen_opts.json_file = str(out / "acados_sim.json")
    return acados_template.AcadosSimSolver(sim, verbose=False)


def acados_step(solver, x, u, p, T, k):
    """Integrate from x over T with constant u, as k RK4 steps of T/k."""
    solver.set("T", T / k)
    solver.set("p", p)
    for _ in range(k):
        x = solver.simulate(x=x, u=u)
    return x


def plant(vp_params=None):
    agent = yaml.safe_load(load_agent_yaml(AGENTS, "agent_1", "SingleTrackAugDynModel"))
    if vp_params is not None:
        agent = agent_with_vehicle_params(agent, vp_params)
    return Sim(SIM_YAML.read_text(), yaml.safe_dump(agent))


def random_case(rng, lim):
    x = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-np.pi, np.pi),
                  rng.uniform(MIN_VALID_SPEED + 1.0, 10.0), rng.uniform(-0.3, 0.3),
                  rng.uniform(-1.5, 1.5), rng.uniform(-0.3, 0.3),
                  rng.uniform(-0.5 * lim.max_decel, 0.5 * lim.max_accel)])
    u = np.array([rng.uniform(-10.0, 10.0), rng.uniform(-1.0, 1.0)])
    return x, u


def test_one_plant_step_matches_the_plant(integrator, vp):
    """acados RK4 with the plant's sub-step reproduces Sim.step: codegen, model and RK4 agree."""
    sim = plant()
    lim = load_raceline_limits(AGENTS, "agent_1", PLANNER)
    rng = np.random.default_rng(0)
    for _ in range(20):
        x0, u = random_case(rng, lim)
        sim.set_state(x0)
        ref = sim.step(u)
        got = acados_step(integrator, x0, u, parameter_values(vp), PLANT_DT, int(round(PLANT_DT / PLANT_SUBSTEP)))
        np.testing.assert_allclose(got, ref, rtol=0, atol=1e-10)


@pytest.mark.parametrize("scales", [{"mass": 1.2}, {"mu": 0.8}, {"Iz": 1.3}, {"cf": 0.8}, {"cr": 1.25}])
def test_parameter_vector_matches_a_perturbed_plant(integrator, vp, scales):
    """p = [mass, mu, Iz, cf, cr] reaches the dynamics in that order: perturbed p = plant with the same values."""
    perturbed = vp.with_overrides(**{k: s * getattr(vp, k) for k, s in scales.items()})
    sim = plant(perturbed)
    x0 = np.array([0.0, 0.0, 0.2, 6.0, 0.1, 0.8, 0.15, -2.0])
    u = np.array([5.0, 0.4])
    sim.set_state(x0)
    ref = sim.step(u)
    got = acados_step(integrator, x0, u, parameter_values(perturbed), PLANT_DT, 10)
    np.testing.assert_allclose(got, ref, rtol=0, atol=1e-10)
    nominal = acados_step(integrator, x0, u, parameter_values(vp), PLANT_DT, 10)
    assert not np.allclose(nominal, ref, atol=1e-6)  # the parameters actually matter


def test_discretization_error_of_one_mpc_interval(integrator, vp):
    """One MPC interval of 0.1 s predicted with k RK4 steps vs the plant (100 x 1 ms).

    Measured (2026-10-17): 1 step 0.46, 2 steps 3.4e-2, 5 steps 4.6e-4, 10 steps 2.4e-5 (worst
    state error over 30 cases). A single 100 ms step is far too coarse (fast lateral tire dynamics);
    use >= 5 RK4 steps per 0.1 s interval in the MPC (sim_method_num_steps).
    """
    T = 0.1
    sim = plant()
    lim = load_raceline_limits(AGENTS, "agent_1", PLANNER)
    rng = np.random.default_rng(1)
    ks = (1, 2, 5, 10)
    worst = {k: 0.0 for k in ks}
    worst_state = {k: 0 for k in ks}
    for _ in range(30):
        x0, u = random_case(rng, lim)
        sim.set_state(x0)
        ref = sim.step(u, n=int(round(T / PLANT_DT)))
        for k in ks:
            err = np.abs(acados_step(integrator, x0, u, parameter_values(vp), T, k) - ref)
            if err.max() > worst[k]:
                worst[k], worst_state[k] = float(err.max()), int(err.argmax())
    names = ("x", "y", "yaw", "vx", "vy", "yaw_rate", "steer", "acc")
    print("\none 0.1 s MPC interval, worst |x_acados - x_plant| over 30 cases:")
    for k in ks:
        print(f"  {k:2d} RK4 step(s) of {1000 * T / k:5.1f} ms: {worst[k]:.2e}  (largest in {names[worst_state[k]]})")
    errs = [worst[k] for k in ks]
    assert all(a > b for a, b in zip(errs, errs[1:]))  # converges
    assert worst[5] / worst[10] > 10.0  # 4th order: halving the step gives ~16x
    assert worst[5] < 1e-3 and worst[10] < 1e-4
