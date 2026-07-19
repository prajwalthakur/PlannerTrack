# PlannerTrack-V2

A ROS2 (Jazzy) simulation platform for developing and testing autonomous
vehicle algorithms , behavior, multi-agent coordination, planning, and
control, built around a heterogeneous multi-agent simulator core, with a
plugin architecture designed to support multiple vehicle types (ground
vehicles , aerial vehicles (planned)) without hardcoding vehicle-specific
simulation code.

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
- **Motion planning components** — sampling-based path planning and
  trajectory generation for Ackermann-style vehicles (`planning/`),
  developed alongside the simulator.
- **RViz-based multi-agent visualization** — see it running:
  [`video/multi_agent_rviz.mp4`](video/multi_agent_rviz.mp4).
- Built for **ROS2 Jazzy**, with a Dockerized, bind-mounted dev environment.

## Status: actively under restructuring (WIP)

This branch is mid-rework of the simulator's core: `workspace/ros_ws/src`'s
vehicle model layer is being rebuilt around ROS2 `pluginlib`, so that a new
vehicle's dynamics, geometry, and collision model can be added as a plugin
package without editing or recompiling the simulator itself. 

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
  vehicles (currently a single-track/Ackermann bicycle model). A future
  `motion_model_aerial_vehicles` would hold drone dynamics.
- **`agent_sim`** — the ROS2 orchestrator node. Owns the simulation loop,
  reads per-agent YAML config, and is the only package that talks to
  `pluginlib` or config files directly — it never references a concrete
  vehicle type.

Adding a new vehicle type is meant to mean: write a new plugin package,
reference it by name in a YAML config file — no changes to `agent_sim` or
`motion_model_base`.

## Getting Started

Build the dev image and start a development container (workspace is
bind-mounted, not baked into the image):

```bash
./scripts/.build/.build.sh
./scripts/.deploy/devel.sh        # CPU; pass -c explicitly if needed
```

Inside the container, build with `colcon` against ROS2 Jazzy in the usual
way (`colcon build --packages-select <package>` from `/workspace/ros_ws`).

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
