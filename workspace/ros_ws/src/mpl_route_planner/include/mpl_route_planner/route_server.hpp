/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route (route_server.hpp), using
// plain rclcpp_lifecycle::LifecycleNode + rclcpp_action::Server instead of
// nav2::LifecycleNode/nav2::SimpleActionServer -- same lifecycle-node +
// action-server *pattern* (driven externally by nav2_lifecycle_manager,
// exactly like map_server already is in intersection.launch.py), without a
// dependency on the nav2_ros_common package. ComputeAndTrackRoute (live
// route tracking/rerouting) is not implemented -- only ComputeRoute.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <project_utils_msgs/action/compute_route.hpp>

#include "mpl_route_planner/graph_loader.hpp"
#include "mpl_route_planner/path_converter.hpp"
#include "mpl_route_planner/route_planner.hpp"
#include "mpl_route_planner/types.hpp"

namespace mpl_route
{

/**
 * @class mpl_route::RouteServer
 * @brief A lifecycle node exposing route planning over a loaded graph as
 * the ComputeRoute action: {start_id, goal_id} in, a dense nav_msgs/Path
 * out (already corner-smoothed by PathConverter).
 */
class RouteServer : public rclcpp_lifecycle::LifecycleNode
{
  public:
	using ComputeRoute = project_utils_msgs::action::ComputeRoute;
	using GoalHandleComputeRoute = rclcpp_action::ServerGoalHandle<ComputeRoute>;

	explicit RouteServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~RouteServer() override = default;

  protected:
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
	    const rclcpp_lifecycle::State & state) override;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
	    const rclcpp_lifecycle::State & state) override;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
	    const rclcpp_lifecycle::State & state) override;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
	    const rclcpp_lifecycle::State & state) override;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
	    const rclcpp_lifecycle::State & state) override;

	rclcpp_action::GoalResponse handleGoal(
	    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeRoute::Goal> goal);
	rclcpp_action::CancelResponse handleCancel(
	    const std::shared_ptr<GoalHandleComputeRoute> goal_handle);
	void handleAccepted(const std::shared_ptr<GoalHandleComputeRoute> goal_handle);
	void execute(const std::shared_ptr<GoalHandleComputeRoute> goal_handle);

  private:
	std::string graph_filepath_;
	std::string route_frame_;

	Graph graph_;
	GraphToIDMap id_to_graph_map_;

	RoutePlanner planner_;
	PathConverter converter_;

	rclcpp_action::Server<ComputeRoute>::SharedPtr action_server_;
	rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
	rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
	    graph_vis_pub_;

	std::mutex graph_mutex_;
	bool is_active_{false};
};

}  // namespace mpl_route
