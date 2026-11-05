/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

<!--
 Author: Prajwal Thakur <prajwalthakur98@gmail.com>
-->
# rl_framework — learning-based control on top of PlannerTrack

Everything learning-related lives here, separate from the classical packages in the rest of `src/`.
The C++ simulator is reused in-process through pybind11 bindings
(`plannertrack_py`), so training runs faster than real time without a ROS graph.

Design notes: [../project_docs/learning_based_controls/](../project_docs/learning_based_controls/)
(`readme.md` for the learned-dynamics project, `python_wrapper.md` for learned MPC weights).

## Layout

| Path | Contents |
|---|---|
| `plannertrack_py/` | ament_cmake package: pybind11 core (`Sim`, side-effect-free plant dynamics; minimum-curvature raceline from `planning/min_curv_planner`) + Python layer (`config`, `raceline`; later track/Frenet utils, delay buffer, domain randomization, gymnasium envs). |
| `mpc/` | acados OCP definitions in Python (weights as runtime parameters). Single source of truth for training and for generated C code used at deployment. |
| `configs/` | Experiment configs: seeds, randomization ranges, env and policy settings. |
| `experiments/weight_mpc/` | Learned MPC cost weights: fixed → rule-based → Diff-WMPC → PPO → SG-RL. |
| `experiments/residual_dynamics/` | Learned residual dynamics: data collection, training, multi-step and closed-loop evaluation. |
| `experiments/behavior_cloning/` | BC → DAgger → RL policy (pure pursuit as teacher). |
| `data/` | Logged rollouts and datasets. Not built by colcon (`COLCON_IGNORE`). |
| `runs/` | Checkpoints, TensorBoard logs, evaluation outputs. Not built by colcon (`COLCON_IGNORE`). |

## Build

`colcon build` from the workspace root (`/workspace/ros_ws`) finds `plannertrack_py` under
`src/rl_framework/` like any other package:

```bash
cd /workspace/ros_ws
colcon build --symlink-install --packages-up-to plannertrack_py --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash   # required: pluginlib finds the vehicle-model plugins via AMENT_PREFIX_PATH
python3 -m pytest src/rl_framework/plannertrack_py/test   # or: colcon test --packages-select plannertrack_py
```

Always build in Release: colcon's default (no build type) compiles without optimization,
and the simulator runs ~15x slower (~8k vs ~120-150k steps/s per core).

```python
from plannertrack_py import make_sim, STATE_NAMES
sim = make_sim("src/scenarios/ground_vehicle_racing_rl/params/sim.yaml",
               "src/scenarios/ground_vehicle_racing_rl/params/agents.yaml", "agent_1")
x = sim.step([1.0, 0.0], n=5)   # [jerk, steer_rate] held for 5 x 10 ms
```

The learning scenario is `scenarios/ground_vehicle_racing_rl` (a copy of `ground_vehicle_racing`):
agent_1 runs `SingleTrackAugDynModel` (state `[x, y, yaw, vx, vy, yaw_rate, steer, acc]`, input
`[jerk, steer_rate]`, steps.md §1.3) and the reference is the C++ minimum-curvature raceline.
`make_sim` builds that plant by default from any single-track agent (also from
`ground_vehicle_racing`, whose ROS controllers run `SingleTrackDynStateModel` with input `[acc,
steer_rate]`); `dynamics_plugin=None` keeps the plugin named in `agents.yaml`.

ROS side: `ros2 launch scenarios ground_vehicle_racing_rl.launch.py` starts the simulator with the
jerk plant, the raceline planner and RViz, but no controller (`controller: "none"` in
`agents.yaml`; pure pursuit / MPPI command acceleration). The input comes from an external node on
`/agent_1/control` (`project_utils_msgs/EigenVectorStamped`, `[jerk, steer_rate]`).

The reference trajectory comes from the same C++ code and scenario files as the ROS plugin
`MinCurvPlanner`, so training uses exactly the line the simulated controllers follow:

```python
from plannertrack_py import compute_raceline
params = "src/scenarios/ground_vehicle_racing_rl/params"
rl = compute_raceline(f"{params}/agents.yaml", "agent_1", f"{params}/min_curv_planner.yaml")
rl.s, rl.x, rl.y, rl.psi, rl.kappa, rl.vx, rl.ax, rl.a_lat, rl.w_right, rl.w_left
rl.lap_time, rl.iqp_converged
rl.save_npz("reference_racing.npz")   # steps.md §3.5 reference file

from plannertrack_py import ReferenceTrack
ref = ReferenceTrack(rl)                 # or ReferenceTrack.from_npz(path)
p = ref.project(x, y)                    # s0 and lateral error n (> 0 left)
stages = ref.stage_references(p.s, n_stages=25, dt=0.1)   # by arc length, psi continuous
v5, kappa5 = ref.policy_lookahead(p.s)   # Diff-WMPC policy input (5 + 5 over 2.55 s)
```

MPC side (steps.md Steps 2 and 4, in progress): `sim.xdot(x, u)` gives the plant's continuous-time
derivative (the function it integrates); `plannertrack_py.mpc.VehicleParams` / `load_vehicle_params()` read
the vehicle parameters from the same `agents.yaml` the plant uses, and `agent_with_vehicle_params()` builds
plants with perturbed mass or μ; `plannertrack_py.mpc.model` (CasADi prediction model) is still a stub.

Step 1 deliverables (npz, validation plot and log) for a scenario:
`python3 src/rl_framework/experiments/weight_mpc/make_reference.py` → `rl_framework/data/references/`.

Vehicle parameters come from the agent's entry in `agents.yaml` (incl. `vehicle_limits`), tuning
from the `min_curv_planner` block of the planner config; the centerline is the JSON file that
config points to (or pass `centerline=` as an (N, 4) array). Derivations:
`project_docs/learning_based_controls/min_curv_*.md`.

## Python environment

No venv: the dev container sets `PIP_BREAK_SYSTEM_PACKAGES=1`, so packages install straight
into the container's Python (torch is already there):

```bash
pip install gymnasium stable-baselines3 casadi pyyaml pandas pyarrow
# acados + acados_template: build from source (see acados docs), then pip install -e <acados>/interfaces/acados_template
```

- Check `pip install --dry-run ...` first: if it wants to upgrade `numpy` or other packages that
  ROS installed via apt, pin them rather than letting pip replace them.
- Anything installed this way is lost when the container is rebuilt; add it to the Dockerfile
  once the set of packages settles.

Run training scripts from a shell where `install/setup.bash` is sourced.
