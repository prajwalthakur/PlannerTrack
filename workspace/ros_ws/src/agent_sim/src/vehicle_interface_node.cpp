// Author Prajwal Thakur 
#include <rclcpp/rclcpp.hpp>
#include "agent_sim/vehicle_interface_class.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleInterface>();
  node->onConfigure();
  node->onActivate();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
