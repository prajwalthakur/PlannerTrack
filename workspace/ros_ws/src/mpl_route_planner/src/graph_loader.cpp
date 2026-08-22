/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route (geojson_graph_file_loader.cpp)

#include "mpl_route_planner/graph_loader.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <nlohmann/json.hpp>

namespace mpl_route
{

using Json = nlohmann::json;

namespace
{

bool doesFileExist(const std::string & filepath)
{
	return std::filesystem::exists(filepath);
}

// Buckets each GeoJSON feature by geometry type -- Point -> node, otherwise
// (LineString/MultiLineString) -> edge. Matches nav2_route's own
// GeoJsonGraphFileLoader::getGraphElements() exactly, including the fact
// that an edge's own "geometry" is never read past this classification --
// edge position always comes from its startid/endid node lookup.
void getGraphElements(const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges)
{
	for (const auto & feature : features) {
		const std::string type = feature.at("geometry").at("type").get<std::string>();
		if (type == "Point") {
			nodes.push_back(feature);
		} else if (type == "LineString" || type == "MultiLineString") {
			edges.push_back(feature);
		}
	}
}

OperationTrigger triggerFromString(const std::string & trigger)
{
	if (trigger == "ON_ENTER") {
		return OperationTrigger::ON_ENTER;
	}
	if (trigger == "ON_EXIT") {
		return OperationTrigger::ON_EXIT;
	}
	return OperationTrigger::Node;
}

// Recursively converts a JSON object's key/value pairs into a Metadata,
// supporting nested objects, arrays, and primitive (number/bool/string)
// values -- same structure as nav2_route's convertMetaDataFromJson, adapted
// to this package's Metadata::setValue(key, T&) (non-const ref) signature.
Metadata convertMetaDataFromJson(const Json & properties, const std::string & key = "metadata")
{
	Metadata metadata;
	if (!properties.contains(key)) {
		return metadata;
	}

	for (const auto & item : properties[key].items()) {
		const Json & value = item.value();

		if (value.is_object()) {
			Metadata nested = convertMetaDataFromJson(properties[key], item.key());
			metadata.setValue(item.key(), nested);
			continue;
		}

		if (value.is_number_unsigned()) {
			unsigned int v = value.get<unsigned int>();
			metadata.setValue(item.key(), v);
		} else if (value.is_number_integer()) {
			int v = value.get<int>();
			metadata.setValue(item.key(), v);
		} else if (value.is_number()) {
			float v = value.get<float>();
			metadata.setValue(item.key(), v);
		} else if (value.is_boolean()) {
			bool v = value.get<bool>();
			metadata.setValue(item.key(), v);
		} else if (value.is_string()) {
			std::string v = value.get<std::string>();
			metadata.setValue(item.key(), v);
		}
		// arrays and anything else: skipped for now -- none of our graph
		// files use them yet (see generate_route_graph.py's metadata
		// convention: scalars only).
	}

	return metadata;
}

Operations convertOperationsFromJson(const Json & properties)
{
	Operations operations;
	if (!properties.contains("operations")) {
		return operations;
	}

	for (const auto & item : properties["operations"].items()) {
		const Json & json_operation = item.value();
		Operation operation;
		operation.type = json_operation.value("type", item.key());
		operation.trigger = triggerFromString(json_operation.value("trigger", std::string("NODE")));
		operation.metaData = convertMetaDataFromJson(json_operation);
		operations.push_back(operation);
	}
	return operations;
}

EdgeCost convertEdgeCostFromJson(const Json & properties)
{
	EdgeCost edge_cost;
	if (properties.contains("cost")) {
		edge_cost.cost = properties["cost"].get<float>();
	}
	if (properties.contains("overridable")) {
		edge_cost.overrridable = properties["overridable"].get<bool>();
	}
	return edge_cost;
}

Coordinates convertCoordinatesFromJson(const Json & node)
{
	Coordinates coords;
	const auto & properties = node["properties"];
	if (properties.contains("frame")) {
		coords.frameId = properties["frame"].get<std::string>();
	}
	const auto & coordinates = node["geometry"]["coordinates"];
	coords.x = coordinates[0].get<float>();
	coords.y = coordinates[1].get<float>();
	return coords;
}

void addNodesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, const std::vector<Json> & nodes)
{
	unsigned int idx = 0;
	for (const auto & node : nodes) {
		const auto & properties = node["properties"];
		graph[idx].nodeId = properties.at("id").get<unsigned int>();
		graph_to_id_map[graph[idx].nodeId] = idx;
		graph[idx].coordinates = convertCoordinatesFromJson(node);
		graph[idx].operations = convertOperationsFromJson(properties);
		graph[idx].metaData = convertMetaDataFromJson(properties);
		idx++;
	}
}

bool addEdgesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, const std::vector<Json> & edges)
{
	for (const auto & edge : edges) {
		const auto & properties = edge["properties"];
		unsigned int id = properties.at("id").get<unsigned int>();
		unsigned int start_id = properties.at("startid").get<unsigned int>();
		unsigned int end_id = properties.at("endid").get<unsigned int>();

		auto start_it = graph_to_id_map.find(start_id);
		if (start_it == graph_to_id_map.end()) {
			std::cerr << "GraphLoader: start id " << start_id << " does not exist for edge id " << id
			          << '\n';
			return false;
		}
		auto end_it = graph_to_id_map.find(end_id);
		if (end_it == graph_to_id_map.end()) {
			std::cerr << "GraphLoader: end id " << end_id << " does not exist for edge id " << id
			          << '\n';
			return false;
		}

		EdgeCost edge_cost = convertEdgeCostFromJson(properties);
		Operations operations = convertOperationsFromJson(properties);
		Metadata metadata = convertMetaDataFromJson(properties);

		graph[start_it->second].addEdge(edge_cost, &graph[end_it->second], id, metadata, operations);
	}
	return true;
}

}  // namespace

bool GraphLoader::loadGraphFromFile(
    Graph & graph, GraphToIDMap & graph_to_id_map, const std::string & filepath)
{
	if (!doesFileExist(filepath)) {
		std::cerr << "GraphLoader: file does not exist: " << filepath << '\n';
		return false;
	}

	std::ifstream graph_file(filepath);
	Json json_graph;
	try {
		json_graph = Json::parse(graph_file);
	} catch (Json::parse_error & ex) {
		std::cerr << "GraphLoader: failed to parse " << filepath << ": " << ex.what() << '\n';
		return false;
	}

	std::vector<Json> nodes, edges;
	getGraphElements(json_graph["features"], nodes, edges);

	if (nodes.empty() || edges.empty()) {
		std::cerr << "GraphLoader: " << filepath
		          << " is malformed -- does not contain both nodes and edges\n";
		return false;
	}

	graph.clear();
	graph.resize(nodes.size());
	addNodesToGraph(graph, graph_to_id_map, nodes);
	return addEdgesToGraph(graph, graph_to_id_map, edges);
}

}  // namespace mpl_route
