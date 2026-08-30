# SSC Planner — Goal & Status

## Goal

Reproduce the *Spatio-Temporal Semantic Corridor* (SSC) approach to motion planning around
dynamic agents. Instead of planning a single trajectory directly, the planner first carves out
a provably collision-free tube through space-time — a sequence of `(s, d, t)` boxes ("cubes")
in Frenet coordinates along the ego's reference lane — by rasterizing every other agent's
predicted occupancy into a space-time grid and inflating free cubes around the ego's own
space-time path. Once that corridor exists, a trajectory is optimized *inside* it (a Bezier
curve via QP), turning the safe region into an actual drivable path.

This is `SscPlanner::computeTrajectory()`'s two-stage pipeline
(`planning/ssc_planner/src/ssc_planner.cpp:162-170`):

```cpp
computeSSCCorridor();
computeBezierTrajectory();
```

## Current status

- **`computeSSCCorridor()`** (`ssc_planner.cpp:281`) — **done, verified live.** Occupancy
  rasterization, cube-inflation chaining, and corridor finalization all run correctly against
  real ROS2 topics in the `intersection` scenario (4 agents, unprotected left turn). Verified
  both visually (RViz) and by inspecting raw `ros2 topic echo` cube data.
- **`computeBezierTrajectory()`** (`ssc_planner.cpp:307`) — **empty stub, not started.** This is
  the next concrete piece of work: an osqp/osqp-eigen QP over Bezier control points, constrained
  per-segment by the corridor cubes already computed above.
- Route/waypoint generation for the scenario (nav2-based `route_server` + `map_server`,
  lifecycle-managed) is done and feeding the planner correctly.
- **Deferred, not part of the current push**: real MPDM-style behavior prediction — other
  agents currently use scripted/static predicted trajectories, not live prediction.

## Live visualization infrastructure (built to verify the corridor pipeline)

Before this work, none of the corridor-generation code had ever been run live — no node hosted
`PlannerBase` plugins, and there was no way to see whether a generated corridor was sane. Built:

- **`PlannerNode`** (`planning/planner_base/{include,src}/planner_node.{hpp,cpp}` +
  `src/main.cpp`) — a generic live host for any `PlannerBase` plugin, one agent per process.
  Two-phase init: the constructor only reads params and loads the plugin via
  `pluginlib::ClassLoader<PlannerBase>`; a separate `configure()` (called from `main.cpp` right
  after `std::make_shared<PlannerNode>(...)`) hands the plugin a real `shared_from_this()` and
  starts the timer — `enable_shared_from_this` is only valid once the node is actually owned by
  a `shared_ptr`. Deliberately a plain executable, not `rclcpp_components` (nesting pluginlib
  loading inside a component's dlopen wrapper breaks plugin-factory lookup — a known failure
  mode already avoided elsewhere in this codebase).
- **`ssc_visualizer`** (`planning/ssc_planner/{include,src}/ssc_visualizer.{hpp,cpp}`) — two
  views: an abstract Frenet-space debug view (`(s, d, t)` drawn literally as `(x, y, z)`), and
  the main map-frame overlay used for the portfolio GIF — the corridor projected back onto the
  real reference lane and rendered as a red ribbon (`buildCorridorMarkersCartesian`), alongside
  the map, lane markers, route graph, and all 4 agents.
- **`scenarios/intersection/launch/ssc_corridor_debug.launch.py`** +
  **`scenarios/intersection/params/ssc_corridor_debug.rviz`** — debug launch/RViz config, kept
  separate from the production `intersection.launch.py` since `computeBezierTrajectory()` isn't
  implemented yet.

### Issues hit, and how each was resolved

1. **Stale build → ABI crash.** `vehicle_interface_node` crashed with `undefined symbol:
   AgentModel::step(WorldSnapshot const&)` after an interface changed upstream.
   → Full `colcon build --packages-up-to agent_sim` to resync `project_utils_msgs` →
   `project_utils` → `motion_model_base` → `agent_sim`.
2. **New executable failed to launch.** `planner_node_exe` failed with "cannot open shared
   object file" — its support library was built `SHARED` but never installed.
   → Changed to `STATIC` in `planner_base/CMakeLists.txt`.
3. **Time drift — corridor sank below the ground plane (z ≈ -30 to -46).** Root cause:
   `computeTrajectory()` re-ran on a timer, re-basing `mStartTime` off the ego's *live* odom
   clock each cycle — but the scenario's `reference_trajectory` is published once and latched,
   so its `header.stamp` stays frozen. Every re-plan cycle widened the gap, pushing
   `FrenetState::t` further negative.
   → One-shot compute: the timer cancels itself after the first successful
   `computeTrajectory()` call, gated by a startup delay so it doesn't fire on a partially-ready
   first tick, with matching latched (`Transient Local`) publishers/subscribers throughout.
4. **Spline out-of-domain crash.** `getSplineInterpolatedPointAt(s)` throws when `s` falls
   outside the reference path's actual data range, which the corridor's `s`-extent can
   legitimately exceed (kinematic reachability grows beyond how much path data exists).
   → Clamp `s` to `[getAccumulatedLength(0), getAccumulatedLength(size-1)]` before projecting.
5. **Triangle fill invisible in RViz.** → Emit both triangle winding orders (defends against
   backface culling) and raise alpha.
6. **Overlapping cube outlines looked wrong.** Individually rendering every cube's rectangle
   produced a messy jagged stack. Verified via raw `ros2 topic echo` that the underlying math
   was correct — SSC cubes are *designed* to overlap heavily in `s` (needed for the downstream
   QP's continuity constraint), especially pronounced with a stationary ego.
   → Redesigned the visualizer to compute one union envelope per corridor
   (`min(s_lb)..max(s_ub)` × `min(d_lb)..max(d_ub)`) and sample it finely (60 steps) along `s`,
   projecting through the reference lane — one smooth ribbon that curves with the road instead
   of a stack of boxes.
7. **RViz kept crashing/disappearing.** Traced to the `Map` display's GLSL shader failing to
   link against the environment's GL driver.
   → Disabled the `Map` display in the RViz config; everything else unaffected.

## Output

This pipeline produced the demo assets referenced in the top-level README:
`images/plannertrack_intersection.gif` and `images/safe_corridor.png`.
