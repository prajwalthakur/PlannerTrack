/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "mpl_route_planner/path_converter.hpp"

#include <cmath>

#include "mpl_route_planner/corner_smoothing.hpp"
#include "mpl_route_planner/utils.hpp"

namespace mpl_route
{

//////////////////////////////////////////////////////////////////////////

void PathConverter::configure(
    float density, bool smooth_corners, float smoothing_radius, float smoothing_angle_threshold)
{
	density_ = density;
	smooth_corners_ = smooth_corners;
	smoothing_radius_ = smoothing_radius;
	smoothing_angle_threshold_ = smoothing_angle_threshold;
}

//////////////////////////////////////////////////////////////////////////

nav_msgs::msg::Path PathConverter::densify(
    const Route & route, const std::string & frame, const rclcpp::Time & now)
{
	nav_msgs::msg::Path path;
	path.header.frame_id = frame;
	path.header.stamp = now;

	if (route.edges.empty()) {
		if (route.startNode) {
			path.poses.push_back(
			    utils::toMsg(route.startNode->coordinates.x, route.startNode->coordinates.y));
		}
		return path;
	}

	Coordinates start = route.edges.front()->start->coordinates;
	Coordinates end;

	// Walk every interior node (where edge i meets edge i+1), smoothing the
	// corner there if requested and if it geometrically fits.
	for (std::size_t i = 0; i + 1 < route.edges.size(); ++i) {
		const EdgePtr edge = route.edges[i];
		const EdgePtr next_edge = route.edges[i + 1];
		end = edge->end->coordinates;

		CornerArc corner_arc(
		    start, end, next_edge->end->coordinates, smoothing_radius_, smoothing_angle_threshold_);

		if (smooth_corners_ && corner_arc.isCornerValid()) {
			Coordinates arc_start = corner_arc.getCornerStart();
			interpolateEdge(start.x, start.y, arc_start.x, arc_start.y, path.poses);
			corner_arc.interpolateArc(density_ / smoothing_radius_, path.poses);
			start = corner_arc.getCornerEnd();
		} else {
			interpolateEdge(start.x, start.y, end.x, end.y, path.poses);
			start = end;
		}
	}

	// Final edge: straight to the goal, then push the exact goal point
	// (interpolateEdge intentionally stops one step short of its endpoint --
	// see its doc comment).
	const EdgePtr last_edge = route.edges.back();
	interpolateEdge(
	    start.x, start.y, last_edge->end->coordinates.x, last_edge->end->coordinates.y, path.poses);
	path.poses.push_back(
	    utils::toMsg(last_edge->end->coordinates.x, last_edge->end->coordinates.y));

	// Orientation: face each point toward the next one.
	for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
		double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
		double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
		double yaw = std::atan2(dy, dx);
		path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
		path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
	}
	if (path.poses.size() >= 2) {
		path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
	}

	return path;
}

//////////////////////////////////////////////////////////////////////////

void PathConverter::interpolateEdge(
    float x0, float y0, float x1, float y1, std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
	const float mag = hypotf(x1 - x0, y1 - y0);
	if (mag < 1e-6f) {
		poses.push_back(utils::toMsg(x0, y0));
		return;
	}

	const unsigned int num_pts = static_cast<unsigned int>(std::ceil(mag / density_));
	if (num_pts < 1) {
		return;
	}
	const float step = mag / static_cast<float>(num_pts);
	const float ux = (x1 - x0) / mag;
	const float uy = (y1 - y0) / mag;

	float x = x0;
	float y = y0;
	poses.push_back(utils::toMsg(x, y));

	for (unsigned int i = 1; i < num_pts; ++i) {
		x += ux * step;
		y += uy * step;
		poses.push_back(utils::toMsg(x, y));
	}
}

}  // namespace mpl_route
