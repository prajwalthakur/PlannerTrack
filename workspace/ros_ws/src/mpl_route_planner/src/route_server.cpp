/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "mpl_route_planner/route_server.hpp"

#include "mpl_route_planner/utils.hpp"

namespace mpl_route
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//////////////////////////////////////////////////////////////////////////

RouteServer::RouteServer(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("route_server", "", options)
{
}

//////////////////////////////////////////////////////////////////////////

CallbackReturn RouteServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	graph_filepath_ = declare_parameter("graph_filepath", std::string(""));
	route_frame_ = declare_parameter("route_frame", std::string("map"));
	float path_density = static_cast<float>(declare_parameter("path_density", 0.05));
	bool smooth_corners = declare_parameter("smooth_corners", true);
	float smoothing_radius = static_cast<float>(declare_parameter("smoothing_radius", 0.8));
	float smoothing_angle_threshold =
	    static_cast<float>(declare_parameter("smoothing_angle_threshold", 2.9));

	converter_.configure(path_density, smooth_corners, smoothing_radius, smoothing_angle_threshold);

	GraphLoader loader;
	if (!loader.loadGraphFromFile(graph_, id_to_graph_map_, graph_filepath_)) {
		RCLCPP_ERROR(
		    get_logger(), "Failed to load graph from %s -- see stderr above for the parser error",
		    graph_filepath_.c_str());
		return CallbackReturn::FAILURE;
	}
	RCLCPP_INFO(get_logger(), "Loaded %zu nodes from %s", graph_.size(), graph_filepath_.c_str());

	path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1));
	graph_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
	    "route_graph", rclcpp::QoS(1).transient_local());

	action_server_ = rclcpp_action::create_server<ComputeRoute>(this, "compute_route",
	    std::bind(&RouteServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
	    std::bind(&RouteServer::handleCancel, this, std::placeholders::_1),
	    std::bind(&RouteServer::handleAccepted, this, std::placeholders::_1));

	return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////

CallbackReturn RouteServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
	RCLCPP_INFO(get_logger(), "Activating");
	path_pub_->on_activate();
	graph_vis_pub_->on_activate();
	graph_vis_pub_->publish(*utils::toMsg(graph_, route_frame_, now()));
	is_active_ = true;
	return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////

CallbackReturn RouteServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
	RCLCPP_INFO(get_logger(), "Deactivating");
	is_active_ = false;
	path_pub_->on_deactivate();
	graph_vis_pub_->on_deactivate();
	return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////

CallbackReturn RouteServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");
	action_server_.reset();
	path_pub_.reset();
	graph_vis_pub_.reset();
	graph_.clear();
	id_to_graph_map_.clear();
	return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////

CallbackReturn RouteServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
	RCLCPP_INFO(get_logger(), "Shutting down");
	return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////

rclcpp_action::GoalResponse RouteServer::handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ComputeRoute::Goal> /*goal*/)
{
	if (!is_active_) {
		return rclcpp_action::GoalResponse::REJECT;
	}
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

//////////////////////////////////////////////////////////////////////////

rclcpp_action::CancelResponse RouteServer::handleCancel(
    const std::shared_ptr<GoalHandleComputeRoute> /*goal_handle*/)
{
	return rclcpp_action::CancelResponse::ACCEPT;
}

//////////////////////////////////////////////////////////////////////////

void RouteServer::handleAccepted(const std::shared_ptr<GoalHandleComputeRoute> goal_handle)
{
	// Route execution can take a little while (graph lock contention if
	// multiple agents request at once -- see the mutex note in execute())
	// and must not block the executor's callback thread, same reason
	// rclcpp_action's own tutorials spawn a thread here.
	std::thread{std::bind(&RouteServer::execute, this, std::placeholders::_1), goal_handle}
	    .detach();
}

//////////////////////////////////////////////////////////////////////////

void RouteServer::execute(const std::shared_ptr<GoalHandleComputeRoute> goal_handle)
{
	const auto goal = goal_handle->get_goal();
	auto result = std::make_shared<ComputeRoute::Result>();
	const rclcpp::Time start_time = now();

	// The planner mutates search_state in place on the shared `graph_` --
	// concurrent requests must not interleave their searches over it.
	// Fine for one route request at a time; genuine concurrent multi-agent
	// planning needs per-request search-state scratch space instead of
	// this coarse lock (flagged when this package's structure was chosen).
	std::lock_guard<std::mutex> lock(graph_mutex_);

	auto start_it = id_to_graph_map_.find(goal->start_id);
	auto goal_it = id_to_graph_map_.find(goal->goal_id);
	if (start_it == id_to_graph_map_.end() || goal_it == id_to_graph_map_.end()) {
		result->error_code = ComputeRoute::Result::INDETERMINANT_NODES_ON_GRAPH;
		result->error_msg = "start_id or goal_id does not exist in the loaded graph";
		RCLCPP_WARN(get_logger(), "%s", result->error_msg.c_str());
		goal_handle->abort(result);
		return;
	}

	Route route;
	try {
		route = planner_.findRoute(graph_, start_it->second, goal_it->second);
	} catch (const NoValidGraph & ex) {
		result->error_code = ComputeRoute::Result::NO_VALID_GRAPH;
		result->error_msg = ex.what();
		RCLCPP_WARN(get_logger(), "%s", ex.what());
		goal_handle->abort(result);
		return;
	} catch (const NoValidRouteCouldBeFound & ex) {
		result->error_code = ComputeRoute::Result::NO_VALID_ROUTE;
		result->error_msg = ex.what();
		RCLCPP_WARN(get_logger(), "%s", ex.what());
		goal_handle->abort(result);
		return;
	} catch (const std::exception & ex) {
		result->error_code = ComputeRoute::Result::UNKNOWN;
		result->error_msg = ex.what();
		RCLCPP_WARN(get_logger(), "%s", ex.what());
		goal_handle->abort(result);
		return;
	}

	nav_msgs::msg::Path path = converter_.densify(route, route_frame_, now());

	result->path = path;
	result->planning_time = now() - start_time;
	result->error_code = ComputeRoute::Result::NONE;
	goal_handle->succeed(result);

	if (path_pub_->is_activated() && path_pub_->get_subscription_count() > 0) {
		path_pub_->publish(path);
	}
}

}  // namespace mpl_route
