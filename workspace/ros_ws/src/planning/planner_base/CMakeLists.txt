cmake_minimum_required(VERSION 3.8)
project(planner_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(project_utils REQUIRED)
find_package(project_utils_msgs REQUIRED)
find_package(mpl_route_planner REQUIRED)
find_package(motion_model_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# Effectively a header-only interfaces package (base interface + world
# snapshot, for now); src/planner_base.cpp exists only so this can be a
# regular (non-INTERFACE) CMake library target -- see trajectory_optimizer_base
# for the same pattern and why.
add_library(planner_base
  src/planner_base.cpp)

add_library(planner_base::planner_base ALIAS planner_base)
target_compile_features(planner_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(planner_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  planner_base
  PUBLIC
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "project_utils"
  "project_utils_msgs"
  "mpl_route_planner"
  "motion_model_base"
  "pluginlib"
)
target_link_libraries(planner_base PUBLIC yaml-cpp::yaml-cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(planner_base PRIVATE "PLANNER_BASE_BUILDING_LIBRARY")

# STATIC, not the workspace-default SHARED: this is only ever linked into
# planner_node_exe below, so there's no reuse case that needs a runtime-
# loadable .so -- and building it SHARED without a matching install() rule
# left libplanner_node_lib.so undiscoverable at runtime ("cannot open shared
# object file"), since nothing installs it onto the executable's library
# search path.
add_library(planner_node_lib STATIC src/planner_node.cpp)
target_link_libraries(planner_node_lib planner_base)

# Deliberately a plain executable (main.cpp directly constructs PlannerNode),
# not the rclcpp_components_register_node()-generated one -- same reasoning
# as trajectory_follower_node's controller_node: PlannerNode itself does
# pluginlib-based plugin loading (of PlannerBase), and nesting that inside a
# second dlopen scope breaks plugin-factory lookup. Direct linking avoids the
# extra dlopen layer entirely.
add_executable(planner_node_exe src/main.cpp)
target_link_libraries(planner_node_exe planner_node_lib)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS planner_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS planner_node_exe
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
