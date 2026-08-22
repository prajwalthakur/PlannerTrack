/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Integration smoke test: load a real .geojson, plan a route between two
// node file-ids, densify it with corner smoothing, and sanity-check the
// result. No ROS node needed for graph_loader/route_planner/path_converter
// themselves -- rclcpp::Clock() below is only used to get a timestamp for
// the output nav_msgs/Path header.
//
// Usage: ./test_route_planner <path-to-.geojson> <start-file-id> <goal-file-id>

#include <cmath>
#include <iostream>

#include <rclcpp/clock.hpp>

#include "mpl_route_planner/graph_loader.hpp"
#include "mpl_route_planner/path_converter.hpp"
#include "mpl_route_planner/route_planner.hpp"

int main(int argc, char ** argv)
{
	if (argc != 4) {
		std::cerr << "Usage: " << argv[0] << " <path-to-route_graph.geojson> <start-id> <goal-id>\n";
		return 1;
	}
	const std::string filepath = argv[1];
	const unsigned int start_file_id = static_cast<unsigned int>(std::stoul(argv[2]));
	const unsigned int goal_file_id = static_cast<unsigned int>(std::stoul(argv[3]));

	mpl_route::Graph graph;
	mpl_route::GraphToIDMap id_map;
	mpl_route::GraphLoader loader;
	if (!loader.loadGraphFromFile(graph, id_map, filepath)) {
		std::cerr << "Failed to load graph from " << filepath << '\n';
		return 1;
	}
	std::cout << "Loaded " << graph.size() << " nodes.\n";

	if (id_map.find(start_file_id) == id_map.end() || id_map.find(goal_file_id) == id_map.end()) {
		std::cerr << "start/goal file-id not found in graph\n";
		return 1;
	}

	mpl_route::RoutePlanner planner;
	mpl_route::Route route;
	try {
		route = planner.findRoute(graph, id_map.at(start_file_id), id_map.at(goal_file_id));
	} catch (const std::exception & ex) {
		std::cerr << "findRoute failed: " << ex.what() << '\n';
		return 1;
	}

	std::cout << "Route: " << route.edges.size() << " edges, cost=" << route.routeCost << '\n';
	std::cout << "  start node " << route.startNode->nodeId << " (" << route.startNode->coordinates.x
	          << ", " << route.startNode->coordinates.y << ")\n";
	for (const auto & edge : route.edges) {
		std::string cls_default = "unlabeled";
		std::string cls = edge->metaData.getValue<std::string>("class", cls_default);
		std::cout << "  -> node " << edge->end->nodeId << " (" << edge->end->coordinates.x << ", "
		          << edge->end->coordinates.y << ") via edge " << edge->edgeId << " [" << cls
		          << "]\n";
	}

	// Sanity: route must actually start/end where requested.
	if (route.startNode->nodeId != start_file_id) {
		std::cerr << "BUG: route does not start at the requested node\n";
		return 1;
	}
	if (route.edges.back()->end->nodeId != goal_file_id) {
		std::cerr << "BUG: route does not end at the requested node\n";
		return 1;
	}

	// Densify with corner smoothing at a radius matching the vehicle's real
	// minimum turning radius (~0.67-0.76m from s_max=0.4189 rad, wheelbase
	// 0.30-0.34m -- see pure_pursuit_controller.yaml/agents.yaml), with a
	// touch of margin.
	mpl_route::PathConverter converter;
	converter.configure(/*density=*/0.05f, /*smooth_corners=*/true, /*smoothing_radius=*/0.8f,
	    /*smoothing_angle_threshold=*/2.9f);
	rclcpp::Clock clock;
	nav_msgs::msg::Path path = converter.densify(route, "map", clock.now());

	std::cout << "Dense path: " << path.poses.size() << " points\n";
	if (path.poses.size() < 2) {
		std::cerr << "BUG: dense path has too few points\n";
		return 1;
	}

	// Sanity: no consecutive points should be farther apart than a couple
	// multiples of density_ -- a large gap would mean interpolation/chaining
	// between edges silently dropped a segment.
	float max_gap = 0.0f;
	for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
		float dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
		float dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
		max_gap = std::max(max_gap, std::hypotf(dx, dy));
	}
	std::cout << "Max gap between consecutive dense points: " << max_gap << " m\n";
	if (max_gap > 0.15f) {
		std::cerr << "BUG: unexpectedly large gap in dense path (chaining bug?)\n";
		return 1;
	}

	// Sanity: the dense path's start/end should match the route's node
	// coordinates (within a small tolerance).
	auto near = [](float a, float b) { return std::fabs(a - b) < 1e-3f; };
	if (!near(path.poses.front().pose.position.x, route.startNode->coordinates.x) ||
	    !near(path.poses.front().pose.position.y, route.startNode->coordinates.y)) {
		std::cerr << "BUG: dense path does not start at the route's start node\n";
		return 1;
	}
	if (!near(path.poses.back().pose.position.x, route.edges.back()->end->coordinates.x) ||
	    !near(path.poses.back().pose.position.y, route.edges.back()->end->coordinates.y)) {
		std::cerr << "BUG: dense path does not end at the route's goal node\n";
		return 1;
	}

	std::cout << "All checks passed.\n";
	return 0;
}
