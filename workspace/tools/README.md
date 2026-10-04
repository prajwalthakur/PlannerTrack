/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

<!--
 Author: Prajwal Thakur <prajwalthakur98@gmail.com>
-->
# ros_ws — Plugin-Based Heterogeneous Multi-Agent Motion Planning & Racing Stack

A ROS 2 workspace for autonomous-driving **planning, control, and simulation** research,
built around a plugin architecture so dynamics models, planners, controllers, and
trajectory optimizers can be swapped per-agent from YAML. It powers two lines of work:

1. **A classical planning/racing stack** — heterogeneous multi-agent simulator
   (`agent_sim`), pluggable vehicle models, a Spatio-Temporal Semantic Corridor (SSC)
   planner, a family of path/trajectory-tracking controllers, and nav2-based route
   planning. Used for an F1TENTH-scale sim-racing entry (placed 12/79 at ICRA 2026 with
   pure pursuit) and for intersection / urban-driving planning demos.
2. **A learned planning layer** (design stage) — an imitation-learning → RL pipeline
   (BC → DAgger → PPO) that drops in as a replacement for pure pursuit for the
   AutoDRIVE RoboRacer Sim Racing League @ IROS 2026. See
   [src/project_docs/bhevior_cloning/behavior_cloning.md](src/project_docs/bhevior_cloning/behavior_cloning.md).

> This is a personal research workspace. Many `package.xml` descriptions are still
> `TODO`; the authoritative running notes are [src/todo.md](src/todo.md) and
> [src/project_docs/](src/project_docs/).

---

## Signal chain

```
                        ┌─────────────────── agent_sim (multi-agent simulator) ───────────────────┐
route/waypoints  ──►  Planner (SSC / raceline)  ──►  Controller (pure pursuit / MPPI / LQR / …)
   (nav2_route,                                              │
    map_generator)                                           ▼
                                                    (v_ref, δ_ref)  ──►  PID (100 Hz)  ──►  (acc, steer_rate)
                                                                                                  │
                                                                                                  ▼
                                              VehicleModelFactory → AgentModel per agent:
                                              DynamicModel + GeometricModel + CollisionFootPrint + SensorModel
```

The learned policy (thread 2) replaces the **Controller** box only; it consumes a
track-relative Frenet observation (no global pose) and emits the same `(v_ref, δ_ref)`
interface the PID already expects.

---

## Workspace layout

| Package | Kind | Role |
|---|---|---|
| **`project_utils`** | C++ lib | Core utilities: types, poses, geometry, integrators (RK4), unique IDs, parameters, logging, polling subscribers, resampling, RNG. |
| **`project_utils_msgs`** | msgs | Custom messages/actions: `EigenVector(Stamped)`, ackermann/differential control & planning msgs, `f110` waypoint msgs, route actions, debug/profiling msgs. Also holds shared `maps/`, `config/` xacro + rviz. |
| **`project_utils_testing`** | C++ | gtest helpers shared across packages. |
| **`interpolation_utils`** | C++ lib | Linear + cubic spline interpolation (tridiagonal solver), used for raceline / reference-path resampling. |
| **`mpl_utils/`** | cmake | `mpl_cmake` build helpers + lint config (vendored from Autoware-style tooling). |
| **`mpl_qp_interface` / `mpl_osqp_interface`** | C++ lib | Thin wrappers over QP solvers (OSQP, ProxSuite) for the trajectory optimizers / SSC Bezier QP. |
| **`motion_model_base`** | C++ lib | Plugin **base interfaces** + `VehicleModelFactory` — owns `pluginlib::ClassLoader`s for `DynamicModel`, `GeometricModel`, `CollisionFootPrint`, `SensorModel`; `create(simConfig, agentConfig, id) -> AgentModel`. |
| **`motion_model_ground_vehicles`** | plugin | `BicycleKinematicModel`, `SingleTrackDynStateModel` (7-state, load-transfer, steering-actuator lag) `DynamicModel` plugins. |
| **`motion_model_shapes`** | plugin | `RectangularGeometry` (`GeometricModel`), `EllipseCollisionFootPrint` (`CollisionFootPrint`). |
| **`motion_model_sensors`** | plugin | `LidarSensorModel` — brute-force 2D ray-casting lidar, skips the calling agent's own shape. |
| **`agent_sim`** | node | The simulator. Loads `sim.yaml` + `agents.yaml`, builds one `AgentModel` per agent via `VehicleModelFactory`, steps dynamics, publishes state / TF / lidar, subscribes per-agent control (`EigenVector`). Single-threaded executor; ~50-agent target. |
| **`planning/planner_base`** | plugin base + node | `PlannerBase` plugin interface, `WorldSnapshot` / `InputData`, and `PlannerNode` — a generic live host (one agent per process, two-phase `configure()` init). |
| **`planning/ssc_planner`** | plugin | **SSC planner** (Ding et al., RA-L 2019): rasterize agents' predicted occupancy into a space-time grid → inflate collision-free `(s,d,t)` cubes along the Frenet reference lane → optimize a piecewise-Bezier trajectory inside the corridor via QP. Includes `ssc_visualizer`. |
| **`controller/`** | nodes + plugins | Path/trajectory tracking controllers — see below. |
| **`trajectory_optimizer_base` / `trajectory_optimizer_ground_vehicle`** | plugin | Takes waypoints → generates a low-level dynamically-feasible reference trajectory (`GroundVehicleTrajectoryOptimizer` plugin). |
| **`velocity_smoother_ground_vehicle`** | C++ lib | Speed-profile smoothing (`smoother_base` + `velocity_smoother`) for reference trajectories. |
| **`mpl_route_planner`** | node | Custom route server: loads a graph, search-based routing, corner smoothing, path conversion; exposes `ComputeRoute` / `ComputeAndTrackRoute` actions. |
| **`nav2_route`** | node | Vendored Nav2 Route Server (Open Navigation) — graph-based routing with pluggable edge scorers & route operations. Reference / integration target. |
| **`map_generator`** | py node | Generates nav2-compatible occupancy grids from corridor definitions (`generate_map`) and 3D drone-racing gate courses (`generate_drone_course`). |
| **`scenarios`** | launch/params/maps | Per-scenario wiring — see below. |
| **`EPSILON-master/`** | vendored | `COLCON_IGNORE`d. Reference implementation of EPSILON (Ding et al., T-RO 2021) — behavior planner, MPDM, simulator. Source material, not built. |
| **`spatiotemporal_semantic_corridor-master/`** | vendored | `COLCON_IGNORE`d. Original HKUST SSC planner (RA-L 2019) — reference for `planning/ssc_planner`. |

### `controller/` family

| Package | Type | Status |
|---|---|---|
| `trajectory_follower_base` | plugin base (`Lateral` / `Longitudinal` / `Hybrid` controller interfaces) | core |
| `trajectory_follower_node` | host node that loads a controller plugin per agent | core |
| `regulated_pure_pursuit` | lateral (path tracking) | working — ICRA 2026 racing entry |
| `pid_controller` | longitudinal, 100 Hz (`simple_pid`) | working, in the racing loop |
| `mppi_controller` | hybrid — Nav2-style MPPI with critic stack + ackermann model | substantial implementation |
| `lqr_controller` | lateral — DARE / Riccati state-feedback | in progress |
| `lat_based_lqr_controller` | lateral — Apollo `LatController` port | stub / vendored shim |
| `mpcc_controller` | hybrid — MPCC + CBF obstacle avoidance | planned (see `src/todo.md`) |
| `dummy_lateral_controller` / `dummy_longitudinal_controller` | no-op reference impls | test scaffolding |

### `scenarios/`

| Scenario | Launch | What it exercises |
|---|---|---|
| `ground_vehicle_racing` | `ground_vehicle_racing.launch.py` | Multi-agent F1TENTH-scale racing: `agent_sim` + `map_server` + per-agent controller (chosen by each agent's `controller:` field in `agents.yaml`) + waypoint publisher + RViz. |
| `intersection` | `intersection.launch.py`, `ssc_closed_loop_demo.launch.py`, `ssc_corridor_debug.launch.py` | Unprotected left turn, 4 agents; nav2 `route_server` + `map_server` (lifecycle) feeding the SSC planner; corridor-generation debug/visualization. |
| `drone_racing` | `generate_drone_course` + course publisher | 3D gate-course generation and camera-sensor testing. |
| `industrial_yard` | params/maps only | Warehouse-style map (`roboracer_r1`). |

Each scenario dir carries its own `params/` (`sim.yaml`, `agents.yaml`, `params.yaml`,
per-controller yaml), `map/`, and helper `scripts/` (route/lane graph generation,
marker publishers, state reset).

---

## Plugin model

`agents.yaml` is the single source of truth for agent count and composition. Per agent:

```yaml
agents:
  agent_1:
    dynamics_plugin: SingleTrackDynStateModel      # DynamicModel
    dynamics_params: { mass: 3.47, Iz: 0.04712, lf: 0.17, lr: 0.17, mu: 1.0489, ... }
    geometry_plugin: RectangularGeometry            # GeometricModel
    collision_plugin: EllipseCollisionFootPrint     # CollisionFootPrint
    sensor_plugin: LidarSensorModel                 # SensorModel (composed inside AgentModel)
    controller: "pure_pursuit"                      # picked up by the scenario launch file
```

`sim.yaml` is intentionally minimal (`simTimeStep`, integration step). Both files are
plain YAML loaded via `YAML::LoadFile()` — **not** `ros__parameters` files.

---

## Build & run

```bash
# ROS 2 (rclcpp). External deps: Eigen3, yaml-cpp, OSQP (osqp_vendor), ProxSuite,
# pluginlib, nav2 (map_server, lifecycle_manager, costmap_2d, route), f110_msgs,
# nlohmann-json, OpenCV. EPSILON-master/ and spatiotemporal_*-master/ are COLCON_IGNOREd.

cd /workspace/ros_ws
colcon build --symlink-install
source install/setup.bash

# Multi-agent racing
ros2 launch scenarios ground_vehicle_racing.launch.py

# SSC planner — intersection
ros2 launch scenarios intersection.launch.py
ros2 launch scenarios ssc_corridor_debug.launch.py     # corridor visualization only
```

`tools/` holds clang-format helpers (`clang_format_with_separators.sh`,
`insert_function_separators.py`); `.clang-format` is the style.

---

## Current status (Sep 2026)

- **`agent_sim` plugin rework** — *in progress*. `VehicleModelFactory` builds and loads
  all four plugin types per agent. Known blockers logged in [src/todo.md](src/todo.md):
  `SingleTrackDynStateModel`'s `shared_from_this()` throws `bad_weak_ptr` under pluginlib
  (integrator creation currently commented out); `agent_sim::addAgents()` index/`AgentData`
  rewiring incomplete; old `core/vehicleModel` duplicates pending deletion.
- **SSC planner** — `computeSSCCorridor()` done and verified live (RViz + raw topic echo)
  in the intersection scenario; `computeBezierTrajectory()` is an empty stub (next work:
  OSQP QP over Bezier control points). Route/waypoint generation done. Behavior prediction
  is scripted, not live. Details: [src/project_docs/ssc_planner_status.md](src/project_docs/ssc_planner_status.md).
- **Controllers** — pure pursuit + PID working (racing entry); MPPI substantially built;
  LQR in progress; MPCC+CBF planned.
- **Learned policy (BC→RL)** — design only; the `bc_racing/` package in the design doc
  does not exist in this workspace yet. Data collection / training / inference nodes and
  domain-randomization plan are specified in
  [src/project_docs/bhevior_cloning/behavior_cloning.md](src/project_docs/bhevior_cloning/behavior_cloning.md).

---

## Documentation

| Doc | Contents |
|---|---|
| [src/todo.md](src/todo.md) | Live running notes for the `motion_model` / `agent_sim` plugin migration; performance backlog; cleanup list. |
| [src/project_docs/ssc_planner_status.md](src/project_docs/ssc_planner_status.md) | SSC planner goals, two-stage pipeline, current status, visualization infra, issues hit. |
| [src/project_docs/bezier_curve_code_explaination.md](src/project_docs/bezier_curve_code_explaination.md) | Walkthrough of the Bezier trajectory-generation code. |
| [src/project_docs/bhevior_cloning/behavior_cloning.md](src/project_docs/bhevior_cloning/behavior_cloning.md) | Full BC → DAgger → RL design: architecture, observation vector, two-simulator strategy, data collection, training, deployment, evaluation, 2-week schedule, references. |
| [src/interpolation_utils/README.md](src/interpolation_utils/README.md), [src/interpolation_utils/spline_intro.md](src/interpolation_utils/spline_intro.md) | Spline interpolation math and API. |
| [src/nav2_route/README.md](src/nav2_route/README.md) | Nav2 Route Server (upstream docs). |

## Related repositories

- [Apex-Autonomy](https://github.com/prajwalthakur/Apex-Autonomy) — the classical racing stack this workspace extends.
- [PlannerTrack](https://github.com/prajwalthakur/PlannerTrack) — single-track bicycle-model sim used for BC data collection / RL training.
- [AutoDRIVE RoboRacer Sim Racing League @ IROS 2026](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/) — target competition for the learned policy.
