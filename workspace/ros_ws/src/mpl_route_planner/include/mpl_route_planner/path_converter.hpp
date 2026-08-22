/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route (path_converter.hpp), with
// ReroutingState/partial-edge handling dropped (not built yet -- no live
// tracking/rerouting loop exists). configure() takes plain values instead
// of a ROS node, same reasoning as the rest of this package staying
// ROS-node-independent until route_server (which does have a node).
#pragma once

#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>

#include "mpl_route_planner/types.hpp"

namespace mpl_route
{

/**
 * @class mpl_route::PathConverter
 * @brief Converts a sparse Route (a sequence of edges) into a dense
 * nav_msgs/Path, optionally smoothing the corner at each interior node
 * with a CornerArc fillet instead of passing straight through it.
 */
class PathConverter
{
  public:
	PathConverter() = default;
	~PathConverter() = default;

	/**
	 * @param density Target spacing (m) between consecutive dense path points
	 * @param smooth_corners Whether to fit a CornerArc at each interior node
	 * @param smoothing_radius Requested fillet radius (m); CornerArc declines
	 *        (falls back to passing straight through) if it doesn't fit
	 *        within the adjacent edges' lengths
	 * @param smoothing_angle_threshold Corners with an interior angle (rad)
	 *        greater than this are treated as already-straight and skipped
	 */
	void configure(
	    float density, bool smooth_corners, float smoothing_radius, float smoothing_angle_threshold);

	/**
	 * @brief Convert a Route into a dense, oriented path
	 */
	nav_msgs::msg::Path densify(
	    const Route & route, const std::string & frame, const rclcpp::Time & now);

	/**
	 * @brief Linearly interpolate a straight segment at ~density_ spacing,
	 * appending to poses (including its start point, excluding its end --
	 * callers chain edges so the next call's start point is this end).
	 */
	void interpolateEdge(
	    float x0, float y0, float x1, float y1, std::vector<geometry_msgs::msg::PoseStamped> & poses);

  protected:
	float density_{0.05f};
	float smoothing_radius_{0.8f};
	float smoothing_angle_threshold_{2.9f};
	bool smooth_corners_{true};
};

}  // namespace mpl_route
