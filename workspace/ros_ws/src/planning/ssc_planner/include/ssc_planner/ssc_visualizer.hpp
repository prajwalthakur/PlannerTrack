/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 * Adapted from EPSILON's ssc_visualizer.cc's core idea, not its code (that
 * file is ROS1 and a detached debug tool, never called from SscPlanner's own
 * loop there). Ours is deliberately wired directly into the live planner --
 * see SscPlanner::computeSSCCorridor() -- since seeing the corridor update
 * every planning cycle is more useful for iterative development than a
 * separate offline tool.
 */
#pragma once
#include "interpolation_utils/spline_interpolation_points_2d.hpp"
#include "project_utils/types.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include "ssc_planner/ssc_map.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace ssc_planner
{

// Frenet (s, d) -> Cartesian (x, y[, z]) via the reference lane -- x =
// x_ref(s) - d*sin(yaw(s)), y = y_ref(s) + d*cos(yaw(s)), matching
// SplineInterpolationPoints2d::projectPointOntoSpline's own eY convention.
// s is clamped to refLane's actual data domain first (see .cpp for why --
// a corridor/spline's s-extent can legitimately outgrow the reference
// path's own data).
geometry_msgs::msg::Point toCartesian(
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, double s, double d,
    double z = 0.0);

// Treats Frenet (s, d, t) as literal Cartesian (x, y, z) -- matches
// EPSILON's own visualizer's approach -- so RViz renders the corridor as a
// "tower" of stacked semi-transparent cubes going up in the time axis.
// frameId is expected to be a standalone, non-physical debug frame (not
// "map") since (s,d,t) has no real spatial relationship to the world.
visualization_msgs::msg::MarkerArray buildCorridorMarkers(
    const mt::vec_E<mt::vec_E<SpatioTemporalSemanticCubeNd<2>>> & finalCorridor,
    const std::vector<int> & ifCorridorValid, const std::string & frameId,
    const rclcpp::Time & stamp);

// Projects each cube's (s, d) footprint (dropping t) back through the
// reference lane into real Cartesian (x, y) -- same inverse transform as
// SplineInterpolationPoints2d::projectPointOntoSpline's own eY convention
// (x = x_ref(s) - d*sin(yaw(s)), y = y_ref(s) + d*cos(yaw(s))) -- so it can
// be overlaid directly on the real map/agents/routes, unlike
// buildCorridorMarkers's abstract debug frame. Returns a filled ribbon
// (TRIANGLE_LIST, two triangles per cube) plus a thin outline (LINE_LIST)
// showing the individual chained cubes.
visualization_msgs::msg::MarkerArray buildCorridorMarkersCartesian(
    const mt::vec_E<mt::vec_E<SpatioTemporalSemanticCubeNd<2>>> & finalCorridor,
    const std::vector<int> & ifCorridorValid,
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, const std::string & frameId,
    const rclcpp::Time & stamp);

// LINE_STRIP through the solved Bezier trajectory's own waypoints (already
// world-frame Cartesian, straight from buildTrajectoryFromSpline's output
// -- no re-projection needed). This is the actual deliverable of the
// Bezier stage, on its own publisher/namespace so it can be toggled
// independently of the corridor ribbon in RViz.
visualization_msgs::msg::MarkerArray buildBezierTrajectoryMarker(
    const project_utils_msgs::msg::Trajectory & trajectory, const std::string & frameId,
    const rclcpp::Time & stamp);

}  // namespace ssc_planner
