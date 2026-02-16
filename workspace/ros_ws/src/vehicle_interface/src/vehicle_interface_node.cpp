#include "rclcpp/rclcpp.hpp"
#include "vehicle_interface/vehicle_interface_class.h"
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto vehicle_interface_node = std::make_shared<VehicleInterface>();
    vehicle_interface_node->on_activate();
    rclcpp::spin(vehicle_interface_node);
    return 0;
}