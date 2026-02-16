cmake_minimum_required(VERSION 3.8)
project(gym_ros_cpp)

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
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(vehicle_interface_class
  src/vehicle_interface_class.cpp
)

target_include_directories(vehicle_interface_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vehicle_interface_class PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(vehicle_interface_class
  PUBLIC
    project_utils::core
    project_utils::utils
    project_utils::vehicleModel
)

add_executable(vehicle_interface_node src/vehicle_interface_node.cpp)

target_link_libraries(vehicle_interface_node vehicle_interface_class )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# =============================
# Install Targets
# =============================
install(TARGETS
  vehicle_interface_node
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
