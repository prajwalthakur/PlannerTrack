/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route (geojson_graph_file_loader +
// graph_loader), collapsed into one concrete loader: no plugin factory (one
// file format for now), no TF frame transform (every node in our graphs is
// already "map" frame -- see mpl_route_planner/README or the intersection
// scenario's route_graph.geojson).
#pragma once

#include <string>

#include "mpl_route_planner/types.hpp"

namespace mpl_route
{

/**
 * @class mpl_route::GraphLoader
 * @brief Parses a GeoJSON route graph file into a Graph. Point features
 * become Nodes, LineString/MultiLineString features (with startid/endid
 * properties) become DirectionalEdges -- matching nav2_route's own
 * GeoJsonGraphFileLoader convention, so files stay interchangeable between
 * the two.
 */
class GraphLoader
{
  public:
	GraphLoader() = default;
	~GraphLoader() = default;

	/**
	 * @brief Load a graph from a GeoJSON filepath
	 * @param graph The graph to populate (resized to the node count, then
	 *        filled in place -- edges hold raw Node* into this vector, so
	 *        it must not be reallocated afterward)
	 * @param graph_to_id_map Maps a node's file "id" to its index in graph
	 * @param filepath Path to the .geojson file
	 * @return true if the graph was successfully loaded
	 */
	bool loadGraphFromFile(Graph & graph, GraphToIDMap & graph_to_id_map, const std::string & filepath);
};

}  // namespace mpl_route
