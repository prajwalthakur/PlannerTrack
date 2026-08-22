/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route (utils.hpp) -- only the
// pieces actually used by corner_smoothing.hpp/path_converter.hpp/
// route_server.hpp in this package. GoalIntentExtractor-only helpers
// (normalizedDot/findClosestPoint, used for pose-based start/goal pruning)
// are dropped -- not built yet, see route_planner.hpp's scope note.
#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mpl_route_planner/types.hpp"

namespace mpl_route
{
namespace utils
{

/**
 * @brief Convert an (x, y) position into a PoseStamped (position only)
 */
inline geometry_msgs::msg::PoseStamped toMsg(const float x, const float y)
{
	geometry_msgs::msg::PoseStamped pose;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.orientation.w = 1.0;
	return pose;
}

/**
 * @brief Convert the route graph into a MarkerArray for RViz -- nodes as a
 * SPHERE_LIST, edges as a LINE_LIST.
 */
inline visualization_msgs::msg::MarkerArray::UniquePtr toMsg(
    const Graph & graph, const std::string & frame, const rclcpp::Time & now)
{
	auto msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

	visualization_msgs::msg::Marker nodes_marker;
	nodes_marker.header.frame_id = frame;
	nodes_marker.header.stamp = now;
	nodes_marker.action = visualization_msgs::msg::Marker::ADD;
	nodes_marker.ns = "route_graph_nodes";
	nodes_marker.id = 0;
	nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
	nodes_marker.pose.orientation.w = 1.0;
	nodes_marker.scale.x = 0.08;
	nodes_marker.scale.y = 0.08;
	nodes_marker.scale.z = 0.08;
	nodes_marker.color.r = 1.0;
	nodes_marker.color.a = 1.0;
	nodes_marker.points.reserve(graph.size());

	visualization_msgs::msg::Marker edges_marker;
	edges_marker.header.frame_id = frame;
	edges_marker.header.stamp = now;
	edges_marker.action = visualization_msgs::msg::Marker::ADD;
	edges_marker.ns = "route_graph_edges";
	edges_marker.id = 0;
	edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	edges_marker.pose.orientation.w = 1.0;
	edges_marker.scale.x = 0.03;
	edges_marker.color.g = 1.0;
	edges_marker.color.a = 0.6;

	geometry_msgs::msg::Point p;
	for (const auto & node : graph) {
		p.x = node.coordinates.x;
		p.y = node.coordinates.y;
		nodes_marker.points.push_back(p);

		for (const auto & edge : node.neighbors) {
			geometry_msgs::msg::Point start, end;
			start.x = node.coordinates.x;
			start.y = node.coordinates.y;
			end.x = edge.end->coordinates.x;
			end.y = edge.end->coordinates.y;
			edges_marker.points.push_back(start);
			edges_marker.points.push_back(end);
		}
	}

	msg->markers.push_back(edges_marker);
	msg->markers.push_back(nodes_marker);
	return msg;
}

}  // namespace utils
}  // namespace mpl_route
