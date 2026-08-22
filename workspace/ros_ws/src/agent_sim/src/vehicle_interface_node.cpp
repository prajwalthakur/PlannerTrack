// Author Prajwal Thakur
#include <rclcpp/rclcpp.hpp>
#include "agent_sim/vehicle_interface_class.hpp"

/** \file
 * \brief Entry point for the `vehicle_interface_node` executable: constructs
 * one \ref AgentInterface node, runs its two-phase configure/activate
 * startup, then spins it.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AgentInterface>();
  node->onConfigure();
  node->onActivate();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
