/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// A plain LifecycleNode executable, driven externally (configure/activate)
// the same way map_server already is in intersection.launch.py -- via
// nav2_lifecycle_manager, not self-activated the way agent_sim's
// vehicle_interface_node is.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mpl_route_planner/route_server.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<mpl_route::RouteServer>();
	rclcpp::spin(node->get_node_base_interface());
	rclcpp::shutdown();
	return 0;
}
