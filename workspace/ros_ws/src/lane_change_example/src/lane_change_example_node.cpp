#include "rclcpp/rclcpp.hpp"
#include "lane_change_example/lane_change_example.h"
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto lane_change_node = std::make_shared<LaneChangeExample>();
    lane_change_node->on_activate();
    rclcpp::spin(lane_change_node);
    return 0;
} 