/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "mpl_route_planner/route_planner.hpp"

#include <algorithm>

namespace mpl_route
{

//////////////////////////////////////////////////////////////////////////

Route RoutePlanner::findRoute(Graph & graph, unsigned int start_index, unsigned int goal_index,
    const std::vector<unsigned int> & blocked_ids)
{
	if (graph.empty()) {
		throw NoValidGraph("Graph is invalid for routing!");
	}

	// It is important that start_node/goal_node are the underlying pointers
	// into `graph` (not copies) -- Route::edges holds Node*/DirectionalEdge*
	// that must stay valid after this function returns.
	const NodePtr start_node = &graph.at(start_index);
	const NodePtr goal_node = &graph.at(goal_index);
	findShortestGraphTraversal(graph, start_node, goal_node, blocked_ids);

	EdgePtr parent_edge = goal_node->searchState.parentEdge;
	if (!parent_edge) {
		throw NoValidRouteCouldBeFound("Could not find a route to the requested goal!");
	}

	Route route;
	while (parent_edge) {
		route.edges.push_back(parent_edge);
		parent_edge = parent_edge->start->searchState.parentEdge;
	}
	std::reverse(route.edges.begin(), route.edges.end());
	route.startNode = start_node;
	route.routeCost = goal_node->searchState.integratedCost;
	return route;
}

//////////////////////////////////////////////////////////////////////////

void RoutePlanner::resetSearchStates(Graph & graph)
{
	for (auto & node : graph) {
		node.searchState.reset();
	}
}

//////////////////////////////////////////////////////////////////////////

void RoutePlanner::findShortestGraphTraversal(Graph & graph, const NodePtr start_node,
    const NodePtr goal_node, const std::vector<unsigned int> & blocked_ids)
{
	resetSearchStates(graph);
	start_id_ = start_node->nodeId;
	goal_id_ = goal_node->nodeId;
	start_node->searchState.integratedCost = 0.0f;
	addNode(0.0f, start_node);

	float traversal_cost = 0.0f;
	int iterations = 0;
	while (!queue_.empty() && iterations < max_iterations_) {
		iterations++;

		auto [curr_cost, node] = getNextNode();

		// Stale queue entry (a cheaper path to this node was already found
		// and pushed later) -- std::priority_queue has no decrease-key, so
		// this is how superseded entries get discarded cheaply.
		if (curr_cost != node->searchState.integratedCost) {
			continue;
		}

		if (isGoal(node)) {
			clearQueue();
			return;
		}

		for (auto & edge : getEdges(node)) {
			NodePtr neighbor = edge.end;

			if (!getTraversalCost(&edge, traversal_cost, blocked_ids)) {
				continue;
			}

			float potential_cost = curr_cost + traversal_cost;
			if (potential_cost < neighbor->searchState.integratedCost) {
				neighbor->searchState.parentEdge = &edge;
				neighbor->searchState.integratedCost = potential_cost;
				neighbor->searchState.traversalCost = traversal_cost;
				addNode(potential_cost, neighbor);
			}
		}
	}

	if (iterations >= max_iterations_) {
		clearQueue();
		throw NoValidRouteCouldBeFound("Maximum iterations was exceeded!");
	}
}

//////////////////////////////////////////////////////////////////////////

bool RoutePlanner::getTraversalCost(
    const EdgePtr edge, float & score, const std::vector<unsigned int> & blocked_ids)
{
	auto is_blocked = std::find_if(blocked_ids.begin(), blocked_ids.end(),
	    [&](unsigned int id) { return id == edge->edgeId || id == edge->end->nodeId; });
	if (is_blocked != blocked_ids.end()) {
		return false;
	}

	score = edge->getEdgeLength();
	return true;
}

//////////////////////////////////////////////////////////////////////////

NodeElement RoutePlanner::getNextNode()
{
	NodeElement data = queue_.top();
	queue_.pop();
	return data;
}

//////////////////////////////////////////////////////////////////////////

void RoutePlanner::addNode(const float cost, const NodePtr node)
{
	queue_.emplace(cost, node);
}

//////////////////////////////////////////////////////////////////////////

EdgeVector & RoutePlanner::getEdges(const NodePtr node)
{
	return node->neighbors;
}

//////////////////////////////////////////////////////////////////////////

void RoutePlanner::clearQueue()
{
	NodeQueue empty;
	std::swap(queue_, empty);
}

//////////////////////////////////////////////////////////////////////////

bool RoutePlanner::isGoal(const NodePtr node)
{
	return node->nodeId == goal_id_;
}

//////////////////////////////////////////////////////////////////////////

bool RoutePlanner::isStart(const NodePtr node)
{
	return node->nodeId == start_id_;
}

}  // namespace mpl_route
