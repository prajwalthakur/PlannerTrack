/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Smoke test: load a .geojson route graph and report what came out --
// no ROS node needed, just `./test_graph_loader <path-to-.geojson>`.

#include <iostream>

#include "mpl_route_planner/graph_loader.hpp"

int main(int argc, char ** argv)
{
	if (argc != 2) {
		std::cerr << "Usage: " << argv[0] << " <path-to-route_graph.geojson>\n";
		return 1;
	}

	mpl_route::Graph graph;
	mpl_route::GraphToIDMap id_map;
	mpl_route::GraphLoader loader;

	if (!loader.loadGraphFromFile(graph, id_map, argv[1])) {
		std::cerr << "Failed to load graph from " << argv[1] << '\n';
		return 1;
	}

	unsigned int edge_count = 0;
	for (const auto & node : graph) {
		edge_count += static_cast<unsigned int>(node.neighbors.size());
	}

	std::cout << "Loaded " << argv[1] << ": " << graph.size() << " nodes, " << edge_count
	          << " edges\n";

	for (const auto & node : graph) {
		for (const auto & edge : node.neighbors) {
			if (edge.start->nodeId != node.nodeId) {
				std::cerr << "BUG: edge stored under wrong node\n";
				return 1;
			}
			if (edge.end == nullptr) {
				std::cerr << "BUG: edge " << edge.edgeId << " has null end node\n";
				return 1;
			}
		}
	}
	std::cout << "All edges point to valid nodes.\n";

	// Spot-check metadata round-tripping on node 4 (known: north incoming,
	// lane 1, terminal, t=1.0) and one of its outgoing edges' class.
	for (const auto & node : graph) {
		if (node.nodeId != 4) {
			continue;
		}
		std::string arm_default = "MISSING";
		std::string arm = node.metaData.getValue<std::string>("arm", arm_default);
		float t_default = -1.0f;
		float t = node.metaData.getValue<float>("t", t_default);
		std::cout << "node 4 metadata: arm=" << arm << " t=" << t << '\n';
		if (arm != "north" || t != 1.0f) {
			std::cerr << "BUG: metadata did not round-trip as expected\n";
			return 1;
		}
		for (const auto & edge : node.neighbors) {
			std::string cls_default = "MISSING";
			std::string cls = edge.metaData.getValue<std::string>("class", cls_default);
			std::cout << "  edge " << edge.edgeId << " -> node " << edge.end->nodeId
			          << " class=" << cls << '\n';
		}
	}

	return 0;
}
