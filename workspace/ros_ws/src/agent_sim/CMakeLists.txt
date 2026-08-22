cmake_minimum_required(VERSION 3.8)
project(agent_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    eigen3_cmake_module
    Eigen3
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    project_utils
    project_utils_msgs
    motion_model_base
    pluginlib
    visualization_msgs
)

find_package(yaml-cpp REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ── Library: all agent_sim components ─────────────────────────────────────────
set(LIB_SOURCES
  src/vehicle_interface_class.cpp
)

add_library(vehicle_interface_class ${LIB_SOURCES})

target_include_directories(vehicle_interface_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vehicle_interface_class PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(vehicle_interface_class PUBLIC yaml-cpp::yaml-cpp)

# ── Executable ─────────────────────────────────────────────────────────────────
add_executable(vehicle_interface_node src/vehicle_interface_node.cpp)
target_link_libraries(vehicle_interface_node vehicle_interface_class)
ament_target_dependencies(vehicle_interface_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  vehicle_interface_class
  vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(vehicle_interface_class)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
