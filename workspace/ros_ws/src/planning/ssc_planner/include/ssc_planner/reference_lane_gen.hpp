/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "interpolation_utils/spline_interpolation_points_2d.hpp"
#include "planner_base/input_data.hpp"
#include "project_utils/logger.hpp"

#include <utility>

namespace ssc_planner
{

// Builds the ego reference lane -- a queryable spline through InputData's
// seed path -- used as the corridor's seed geometry. mEgoPath is assumed
// already densified/corner-smoothed (see input_data.hpp, and
// mpl_route_planner's path_converter, which is what's expected to produce
// it).
void constructReferenceLane(const InputData & inputData,
    mpl::interpolation::SplineInterpolationPoints2d * refLane, mpl::rclcpp_utils::Logger & logger);

// Projects a world-frame (x, y) point onto the reference lane, returning
// (arc_length_s, lateral_offset_d) -- position-level Frenet coordinates.
// Thin wrapper over SplineInterpolationPoints2d::projectPointOntoSpline
std::pair<double, double> projectOntoReferenceLane(
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, double x, double y);

}  // namespace ssc_planner
