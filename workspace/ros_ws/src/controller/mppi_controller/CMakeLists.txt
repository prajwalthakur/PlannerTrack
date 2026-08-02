cmake_minimum_required(VERSION 3.8)
project(mppi_controller)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# vanilla_mppi_class is a static lib that other packages (e.g. trajectory_follower_node)
# link into their own shared libraries, so it must be position-independent.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_definitions(USE_ROS2)

#add_compile_definitions(USE_ROS1)

# Enable strict compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Find Dependencies
# =============================
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    eigen3_cmake_module
    Eigen3
    rclcpp
    std_msgs
    nav2_costmap_2d
    nav2_util
    tf2_ros
    nav_msgs
    sensor_msgs
    geometry_msgs
    project_utils_msgs
    project_utils
    trajectory_follower_base

)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# =============================
# Source Files Configuration
# =============================

set(LIB_SOURCES
  # Core Logic
  src/vanilla_mppi_class.cpp
  src/mppi_controller.cpp


  # Critics
  src/critics/critics_manager.cpp
  src/critics/common/goal_critic.cpp
  src/critics/common/goal_angle_critic.cpp
  src/critics/common/path_align_critic.cpp
  src/critics/common/path_follow_critic.cpp
  src/critics/common/prefer_forward_critic.cpp
  src/critics/common/path_angle_critic.cpp
  src/critics/common/constraint_critic.cpp
  src/critics/common/cost_critic.cpp
  src/critics/common/twirling_critic.cpp
  src/critics/common/velocity_deadband_critic.cpp

  # Models (Common)
  src/models/model_factory.cpp

  # Models (Ackermann)
  src/models/ackermann/ackermann.cpp
  src/models/ackermann/ackermann_constraints.cpp
  src/models/ackermann/ackermann_control_sequence.cpp
  src/models/ackermann/ackermann_noise_generator.cpp
  src/models/ackermann/ackermann_state.cpp

  # utils
  src/utils/nav2_utils/feasible_path_handler.cpp
  src/utils/nav2_utils/path_utils.cpp
  src/utils/nav2_utils/robot_utils.cpp
  src/utils/mppi_utils/trajectory_visualizer.cpp
)

add_library(vanilla_mppi_class ${LIB_SOURCES})

target_include_directories(vanilla_mppi_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vanilla_mppi_class  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  vanilla_mppi_class
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(vanilla_mppi_class)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

