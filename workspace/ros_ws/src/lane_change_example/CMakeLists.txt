cmake_minimum_required(VERSION 3.8)
project(lane_change_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
    eigen3_cmake_module
    Eigen3
    project_utils
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    ackermann_msgs
    tf2_ros
    visualization_msgs
    yaml-cpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(lane_change_example
  src/lane_change_example.cpp
  src/CreateLane.cpp
)

target_include_directories(lane_change_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lane_change_example PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(lane_change_example
  PUBLIC
    project_utils::core
    project_utils::utils
    project_utils::vehicleModel
)

add_executable(lane_change_example_node src/lane_change_example_node.cpp)

target_link_libraries(lane_change_example_node lane_change_example )


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# =============================
# Install Targets
# =============================
install(TARGETS
  lane_change_example_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
