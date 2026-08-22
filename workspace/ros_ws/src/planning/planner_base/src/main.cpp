/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "planner_base/planner_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<PlannerNode>(rclcpp::NodeOptions());
	node->configure();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
