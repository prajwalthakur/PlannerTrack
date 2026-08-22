cmake_minimum_required(VERSION 3.8)
project(trajectory_follower_node)
find_package(mpl_cmake REQUIRED)
mpl_package()

#find_package(f110_msgs)
find_package(dummy_longitudinal_controller)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)


set(CONTROLLER_NODE controller_node)

ament_auto_add_library(${CONTROLLER_NODE} SHARED
  src/controller_node.cpp 
)

# Deliberately a plain executable, not an rclcpp_components_register_node()
# one -- see docs/pages/plugin_architecture.md (\ref plugin_architecture,
# section 5) for why.
add_executable(${CONTROLLER_NODE}_exe src/main.cpp)
target_link_libraries(${CONTROLLER_NODE}_exe ${CONTROLLER_NODE})

install(TARGETS controller_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_subdirectory(trajectory_follower_node_unit_tests)
  # add_subdirectory(benchmark)
endif()


ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  trajectory_follower_node_unit_tests
  param
)









