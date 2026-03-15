cmake_minimum_required(VERSION 3.8)
project(trajectory_follower_node)
find_package(mpl_cmake REQUIRED)
mpl_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)


set(CONTROLLER_NODE controller_node)

ament_auto_add_library(${CONTROLLER_NODE} SHARED
  src/controller_node.cpp 
)

rclcpp_components_register_node(${CONTROLLER_NODE}
  PLUGIN "mpl::control::trajectory_follower_node::Controller"
  EXECUTABLE ${CONTROLLER_NODE}_exe
)


ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)












