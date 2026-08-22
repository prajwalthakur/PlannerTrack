/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted Open Navigation LLC
#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <any>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mpl_route
{

struct Metadata
{
	Metadata() {}

	// For retrieving metadata at run-time via plugins
	template <typename T>
	T getValue(const std::string & key, T & default_val) const
	{
		auto it = data.find(key);
		if (it == data.end()) {
			return default_val;
		}
		return std::any_cast<T>(it->second);
	}

	// For populating metadata from file
	template <typename T>
	void setValue(const std::string & key, T & value)
	{
		data[key] = value;
	}

	std::unordered_map<std::string, std::any> data;
};

struct Node;
using NodePtr = Node *;
using NodeVector = std::vector<Node>;
using Graph = NodeVector;
using GraphToIDMap = std::unordered_map<unsigned int, unsigned int>;
using GraphToIncomingEdgesMap = std::unordered_map<unsigned int, std::vector<unsigned int>>;
using NodePtrVector = std::vector<NodePtr>;
using NodeElement = std::pair<float, NodePtr>;
using NodeExtents = std::pair<unsigned int, unsigned int>;

struct NodeComparator
{
	bool operator()(const NodeElement & a, const NodeElement & b) const
	{
		return a.first > b.first;
	}
};

using NodeQueue = std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator>;
/**
 * @struct mpl_route::EdgeCost
 * @brief An object to store edge cost or cost metadata for scoring
 */
struct EdgeCost
{
	float cost{0.0};
	bool overrridable{true};  // If overridable, may use plugin edge cost scorers
};
/**
 * @enum mpl_route::OperationTrigger
 * @brief The triggering events for an operation
 */
enum class OperationTrigger {
	Node = 0,
	ON_ENTER = 1,
	ON_EXIT = 2
};

/**
 * @struct mpl_route::Operation
 * @brief An object to store operations to perform on events with types and metadata
 */
struct Operation
{
	std::string type;
	OperationTrigger trigger;
	Metadata metaData;
};

using Operations = std::vector<Operation>;
using OPerationsPtrs = std::vector<Operation *>;
/**
 * @struct nav2_route::OperationsResult
 * @brief Result information from the operations manager
 */
struct OperationsResult
{
	std::vector<std::string> operations_triggered;
	bool reroute{false};
	std::vector<unsigned int> blocked_ids;
};

/**
 * @struct nav2_route::DirectionalEdge
 * @brief An object representing edges between nodes
 */
struct DirectionalEdge
{
	unsigned int edgeId;
	NodePtr start{nullptr};
	NodePtr end{nullptr};
	EdgeCost edgeCost;
	Metadata metaData;
	Operations operations;
	float getEdgeLength();
};
using EdgePtr = DirectionalEdge *;
using EdgeVector = std::vector<DirectionalEdge>;
using EdgePtrVector = std::vector<EdgePtr>;
/**
 * @struct nav2_route::SearchState
 * @brief An object to store state related to graph searching of nodes
 * This is an internal class users should not modify.
 */
struct SearchState
{
	EdgePtr parentEdge{nullptr};
	float integratedCost{std::numeric_limits<float>::max()};
	float traversalCost{std::numeric_limits<float>::max()};
	void reset()
	{
		integratedCost = std::numeric_limits<float>::max();
		traversalCost = std::numeric_limits<float>::max();
		parentEdge = nullptr;
	}
};
/**
 * @struct nav2_route::Coordinates
 * @brief An object to store Node coordinates in different frames
 */
struct Coordinates
{
	std::string frameId{"map"};
	float x{0.0}, y{0.0};
};
/**
 * @struct nav2_route::Node
 * @brief An object to store the nodes in the graph file
 */
struct Node
{
	unsigned int nodeId;
	Coordinates coordinates;
	EdgeVector neighbors;
	Metadata metaData;  // Any metadata stored in the graph file of interest
	Operations operations;
	SearchState searchState;  // State maintained by route search algorithm
	void addEdge(EdgeCost & cost, NodePtr node, unsigned int edgeId, Metadata metaData = {},
	    Operations opData = {})
	{
		neighbors.push_back({edgeId, this, node, cost, metaData, opData});
	}
};

//////////////////////////////////////////////////////////////////////////

inline float DirectionalEdge::getEdgeLength()
{
	return hypotf(
	    end->coordinates.x - start->coordinates.x, end->coordinates.y - start->coordinates.y);
}
/**
 * @struct nav2_route::Route
 * @brief An ordered set of nodes and edges corresponding to the planned route
 */
struct Route
{
	NodePtr startNode;
	EdgePtrVector edges;
	float routeCost{0.0};
};
/**
 * @struct nav2_route::RouteRequest
 * @brief An object to store salient features of the route request including its start
 * and goal node ids, start and goal pose, and a flag to indicate if the start and goal
 * poses are relevant
 */
struct RouteRequest
{
	unsigned int startNodeid;  // node id of start node
	unsigned int goalNodeid;  // node id of goal node
	geometry_msgs::msg::PoseStamped startPose;  // pose of start
	geometry_msgs::msg::PoseStamped goalPose;  // pose of goal
	bool usePoses;  // whether the start and goal poses are used
};
/**
 * @enum nav2_route::TrackerResult
 * @brief Return result of the route tracker to the main server for processing
 */
enum class TrackerResult { EXITED = 0, INTERRUPTED = 1, COMPLETED = 2 };
/**
 * @struct nav2_route::RouteTrackingState
 * @brief Current state management of route tracking class
 */
struct RouteTrackingState
{
	NodePtr lastNode{nullptr}, nextNode{nullptr};
	EdgePtr currentEdge{nullptr};
	int routeEdgesIdx{-1};
	bool withinEadius{false};
};

/**
 * @struct nav2_route::ReroutingState
 * @brief State shared to objects to communicate important rerouting data
 * to avoid rerouting over blocked edges, ensure reroute from the current
 * appropriate starting point along the route, and state of edges if pruned
 * for seeding the Tracker's state. Admittedly, this is a bit complex, so more
 * context is provided inline.
 */
struct ReroutingState
{
	// Communicate edges identified as blocked by the operational plugins like collision checkers.
	// This is fully managed by the route tracker when populated.
	std::vector<unsigned int> blocked_ids;

	// Used to determine if this is the first planning iteration in the goal intent extractor
	// to bypass pruning. Fully managed in the goal intent extractor.
	bool first_time{true};

	// Used to mark current edge being tracked by the route, if progress was made before rerouting.
	// It is reset in the goal intent extractor if the previous progressed edge is different from
	// the new edge from planning. Otherwise, used in the path converter to create new dense path
	// with partial progress information and in the tracker to seed the state to continue.
	// It is managed by both the goal intent extractor and the route tracker.
	EdgePtr curr_edge{nullptr};
	Coordinates closest_pt_on_edge;

	// Used to mark the route tracking state before rerouting was requested.
	// When route tracking made some progress, the Start ID and pose are populated
	// and used by the goal intent extractor to override the initial request's
	// start, current pose along the edge, and pruning criteria. Otherwise, the initial request
	// information is used. This is managed by the route tracker but used by goal intent extractor.
	unsigned int rerouting_start_id{std::numeric_limits<unsigned int>::max()};
	geometry_msgs::msg::PoseStamped rerouting_start_pose;

	void reset()
	{
		rerouting_start_id = std::numeric_limits<unsigned int>::max();
		blocked_ids.clear();
		first_time = true;
		curr_edge = nullptr;
		closest_pt_on_edge = Coordinates();
		rerouting_start_pose = geometry_msgs::msg::PoseStamped();
	}
};

/**
 * @enum nav2_route::EdgeType
 * @brief An enum class describing what type of edge connecting two nodes is
 */
enum class EdgeType { NONE = 0, START = 1, END = 2 };
}  // namespace mpl_route