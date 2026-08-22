/** \file
 * \brief Header-only interfaces package -- this translation unit exists
 * only so `motion_model_base` can be a regular (non-`INTERFACE`) CMake
 * library target: `ament_target_dependencies()` in this `ament_cmake`
 * version does not support `INTERFACE` targets (it internally calls
 * `target_include_directories()`/`target_link_libraries()` with `PUBLIC`,
 * which is illegal on `INTERFACE` targets). As a side effect it also checks
 * that the public headers below are self-contained (compile standalone).
 */

#include "motion_model_base/collision_model/collision_footprint.hpp"
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "motion_model_base/agent_model.hpp"
#include "motion_model_base/vehicle_model_factory.hpp"