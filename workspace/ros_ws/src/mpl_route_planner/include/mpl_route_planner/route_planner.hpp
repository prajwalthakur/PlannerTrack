/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route (route_planner.hpp), with
// the EdgeScorer plugin factory, TF, and RouteRequest/blocked-node
// classification dropped -- edge weight is simply DirectionalEdge::
// getEdgeLength() (Euclidean length) for now. No ROS node dependency:
// this is plain Dijkstra over a Graph, same reasoning as GraphLoader
// staying ROS-free.
#pragma once

#include <limits>
#include <stdexcept>
#include <vector>

#include "mpl_route_planner/types.hpp"

namespace mpl_route
{

/**
 * @brief Thrown when findRoute() is given an empty graph.
 */
class NoValidGraph : public std::runtime_error
{
  public:
	explicit NoValidGraph(const std::string & msg) : std::runtime_error(msg) {}
};

/**
 * @brief Thrown when Dijkstra's search exhausts the queue without reaching the goal.
 */
class NoValidRouteCouldBeFound : public std::runtime_error
{
  public:
	explicit NoValidRouteCouldBeFound(const std::string & msg) : std::runtime_error(msg) {}
};

/**
 * @class mpl_route::RoutePlanner
 * @brief Dijkstra's algorithm over a Graph: given a start and goal node
 * index, returns the lowest-cost Route between them.
 */
class RoutePlanner
{
  public:
	RoutePlanner() = default;
	virtual ~RoutePlanner() = default;

	/**
	 * @brief Find the route from start to goal on the graph
	 * @param graph Graph to search (search state on every node is reset first)
	 * @param start_index Index (not file id -- see GraphToIDMap) of the start node
	 * @param goal_index Index of the goal node
	 * @param blocked_ids Node or edge file-ids to treat as unusable
	 * @return Route object containing the navigation graph route
	 */
	virtual Route findRoute(Graph & graph, unsigned int start_index, unsigned int goal_index,
	    const std::vector<unsigned int> & blocked_ids = {});

  protected:
	/**
	 * @brief Reset the search state of every node in the graph
	 */
	inline void resetSearchStates(Graph & graph);

	/**
	 * @brief Dijkstra's algorithm search on the graph
	 */
	void findShortestGraphTraversal(Graph & graph, const NodePtr start_node,
	    const NodePtr goal_node, const std::vector<unsigned int> & blocked_ids);

	/**
	 * @brief Gets the traversal cost for an edge (its Euclidean length), unless blocked
	 * @return false if this edge should not be expanded (blocked)
	 */
	inline bool getTraversalCost(
	    const EdgePtr edge, float & score, const std::vector<unsigned int> & blocked_ids);

	inline NodeElement getNextNode();
	inline void addNode(const float cost, const NodePtr node);
	inline EdgeVector & getEdges(const NodePtr node);
	inline void clearQueue();
	inline bool isGoal(const NodePtr node);
	inline bool isStart(const NodePtr node);

	int max_iterations_{std::numeric_limits<int>::max()};
	unsigned int start_id_{0};
	unsigned int goal_id_{0};
	NodeQueue queue_;
};

}  // namespace mpl_route
