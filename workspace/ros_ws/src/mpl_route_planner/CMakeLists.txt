cmake_minimum_required(VERSION 3.8)
project(mpl_route_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(project_utils_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# ── Core library: graph loading, Dijkstra planning, path densification.
# types.hpp/utils.hpp/corner_smoothing.hpp are header-only. Deliberately free
# of the action-server/lifecycle-node dependencies below -- usable as a
# plain library with no ROS node needed. ──────────────────────────────────
add_library(mpl_route_planner_core
  src/graph_loader.cpp
  src/route_planner.cpp
  src/path_converter.cpp
)
# Static by default; needs -fPIC to be linkable into a SHARED consumer
# (e.g. ssc_planner, a pluginlib plugin -- pluginlib plugins must be shared
# libraries).
set_target_properties(mpl_route_planner_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(mpl_route_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mpl_route_planner_core PUBLIC
  geometry_msgs
  nav_msgs
  visualization_msgs
  rclcpp
)
target_link_libraries(mpl_route_planner_core PUBLIC nlohmann_json::nlohmann_json)

# ── route_server library + executable: the ComputeRoute action server ────
add_library(route_server src/route_server.cpp)
target_include_directories(route_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(route_server PUBLIC
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  project_utils_msgs
)
target_link_libraries(route_server PUBLIC mpl_route_planner_core)

add_executable(route_server_node src/route_server_main.cpp)
target_link_libraries(route_server_node route_server)

# ── Smoke-test executables ───────────────────────────────────────────────
add_executable(test_graph_loader test/test_graph_loader.cpp)
target_link_libraries(test_graph_loader mpl_route_planner_core)

add_executable(test_route_planner test/test_route_planner.cpp)
target_link_libraries(test_route_planner mpl_route_planner_core)

# ── Install ──────────────────────────────────────────────────────────────
# Libraries need their own EXPORT set + ARCHIVE/LIBRARY/RUNTIME destinations
# so downstream packages' find_package(mpl_route_planner) can actually
# locate and link mpl_route_planner_core/route_server (a single flat
# `DESTINATION lib/${PROJECT_NAME}` -- correct for the executables below --
# put the .a files where no exported CMake target pointed, breaking every
# consumer outside this package).
install(
  TARGETS mpl_route_planner_core route_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  route_server_node
  test_graph_loader
  test_route_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(mpl_route_planner_core route_server)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs nav_msgs visualization_msgs rclcpp rclcpp_action rclcpp_lifecycle
  project_utils_msgs nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
