cmake_minimum_required(VERSION 3.14)
project(dummy_lateral_controller)

find_package(mpl_cmake REQUIRED)
mpl_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)
#${PROJECT_NAME}_lateral_controller
ament_auto_add_library( ${PROJECT_NAME} SHARED
  src/dummy_lateral_controller.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "dummy_lateral_controller::DummyLateralController"
  EXECUTABLE dummy_lateral_controller_exe
)
#ament_auto_package()
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

# ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
#   launch
#   config
# )