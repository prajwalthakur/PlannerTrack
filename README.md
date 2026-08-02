# PlannerTrack

![PlannerTrack](images/obs_avoidance.gif)


*Dynamic Obstacle Avoidance through vanilla mppi controller
— [full demo on YouTube](https://www.youtube.com/watch?v=Jcm3qd_RIX4).*

**A ROS2 (Jazzy) testbed for multi-agent planning and control** — heterogeneous
agents, swappable vehicle models, live control telemetry.

PlannerTrack is a simulation platform for developing and testing autonomous
vehicle algorithms, behavior, multi-agent coordination, planning, and
control, built around a heterogeneous multi-agent simulator core, with a
plugin architecture designed to support multiple vehicle types (ground
vehicles, aerial vehicles planned) without hardcoding vehicle-specific
simulation code.

## Updates — 2026-08-03

- **Dynamic (nonlinear) single-track bicycle model** added to the vehicle
  dynamics plugin layer (`SingleTrackDynStateModel`) — agents can now be
  simulated with a lateral-tire-force dynamic model, alongside the existing
  kinematic bicycle model.
- **MPPI controller** integrated as a selectable hybrid controller mode,
  with a full critic stack (obstacle, cost, path-align, goal, constraint)
  driving **dynamic obstacle avoidance** — see the demo GIF above.

## Key Features

- **Heterogeneous multi-agent simulation** — each agent's dynamics,
  geometry, and collision model are independently swappable ROS2
  `pluginlib` plugins, resolved by name at runtime. Adding a new vehicle
  type (ground, and eventually aerial) means writing a new plugin package —
  zero changes to the simulator core.
- **Config-driven scenarios** — which agents exist, which vehicle models
  they use, and their parameters are all defined in YAML, not hardcoded.
  Different multi-agent scenarios are a config change, not a code change.
- **Clean plant/controller separation** — the simulator consumes only
  already-computed control commands over generic, vehicle-agnostic ROS2
  messages; planning and control logic live in independent, swappable
  components, not inside the simulator itself.
- **RViz-based multi-agent visualization**
- Built for **ROS2 Jazzy**, with a Dockerized, bind-mounted dev environment.


## Architecture (current design)

- **`motion_model_base`** — the core plugin interfaces: `DynamicModel`,
  `GeometricModel`, `CollisionFootPrint` (each a `pluginlib` base class),
  plus `AgentModel` (composes one instance of each into a single simulated
  agent) and `VehicleModelFactory` (builds an `AgentModel` from a YAML
  config block by loading the three named plugins at runtime — no compile-time
  knowledge of any concrete vehicle type).
- **`motion_model_shapes`** — geometry/collision plugin implementations
  (e.g. rectangular geometry, ellipse collision footprint), shared across
  vehicle families since shape is independent of how a vehicle moves.
- **`motion_model_ground_vehicles`** — vehicle dynamics plugins for ground
  vehicles: a kinematic bicycle model and a dynamic (nonlinear, lateral
  tire-force) single-track model. A future `motion_model_aerial_vehicles`
  would hold drone dynamics.
- **`agent_sim`** — the ROS2 orchestrator node. Owns the simulation loop,
  reads per-agent YAML config, and is the only package that talks to
  `pluginlib` or config files directly — it never references a concrete
  vehicle type.


## Getting Started

Build the dev image and start a development container (workspace is
bind-mounted, not baked into the image):

```bash
./scripts/.build/.build.sh
./scripts/.deploy/devel.sh        # CPU; pass -c explicitly if needed
```

Inside the container, build with `colcon` against ROS2 Jazzy in the usual
way (`colcon build --packages-select <package>` from `/workspace/ros_ws`).

To run the racing demo
```bash
  ros2 launch scenarios ground_vehicle_racing.launch.py
```
## License
MIT  


## Use Case
I initiated this project to independently study algorithms and software development for autonomous vehicle systems. This repository is also available for your personal use in studying, education, research, or development.

If this project supports your work or contributes to your tasks, please feel free to inform me by starring the repository.


## Contribution
Any contribution by creating an issue or sending a pull request is welcome!! 
<!-- Please check [this document about how to contribute](/HOWTOCONTRIBUTE.md).   -->



## Author
[Prajwal Thakur](https://github.com/prajwalthakur) 


## Independent Work Declaration

PlannerTrack was initiated and developed independently by Prajwal Thakur
prior to current employment.

This project is maintained as a personal open-source initiative,
developed outside the scope of employment, using personal time and
equipment. It is based solely on publicly available research and
standard algorithms.

This repository does not contain any proprietary, confidential, or
employer-owned intellectual property.


## Acknowledgments

This project adapts code and build tooling from the
[Autoware](https://github.com/autowarefoundation) project (Apache License
2.0):

- **`interpolation_utils`** — interpolation algorithms (linear, spline,
  spherical linear, zero-order hold), adapted from Autoware's
  `interpolation_utils` package, © Tier IV, Inc.
- **`project_utils/parameter.hpp`, `project_utils/validation_utils.hpp`**,
  and **`mpl_rclcpp_utils`** — ROS2 parameter and subscriber utilities,
  adapted from Autoware common utilities, © Tier IV, Inc.
- **`mpl_cmake`** — build tooling patterned after Autoware's CMake
  conventions.

Original copyright and license notices are retained in each adapted file.

Beyond the packages above, some additional functions elsewhere in this
repository were also adapted from Autoware, without a per-file upstream
notice.
