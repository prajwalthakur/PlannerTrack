cmake_minimum_required(VERSION 3.14)
project(dummy_longitudinal_controller)

find_package(mpl_cmake REQUIRED)
mpl_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_longitudinal_controller SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/DummyLongitudinalController.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_longitudinal_controller
  PLUGIN "dummy_longitudinal_controller::DummyLongitudinalController"
  EXECUTABLE ${PROJECT_NAME}_longitudinal_controller_exe
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
  launch
  config
)