// Header-only interfaces package (for now: just the base interface).
//
// This translation unit exists only so `planner_base` can be a regular
// (non-INTERFACE) CMake library target: ament_target_dependencies() in this
// ament_cmake version does not support INTERFACE targets. As a side effect it
// also checks that the public header below is self-contained (compiles
// standalone). Mirrors trajectory_optimizer_base/src/trajectory_optimizer_base.cpp.
//
#include "planner_base/planner_base.hpp"
