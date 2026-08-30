cmake_minimum_required(VERSION 3.8)
project(ssc_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(planner_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(project_utils REQUIRED)
find_package(project_utils_msgs REQUIRED)
find_package(mpl_route_planner REQUIRED)
find_package(interpolation_utils REQUIRED)
find_package(motion_model_base REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mpl_qp_interface REQUIRED)
find_package(eigen-quadprog REQUIRED)

add_library(ssc_planner SHARED
  src/ssc_planner.cpp
  src/reference_lane_gen.cpp
  src/ssc_map_config.cpp
  src/ssc_grid.cpp
  src/ssc_map.cpp
  src/ssc_visualizer.cpp
  src/bezier_traj_gen/bezier_curve.cpp
)
add_library(ssc_planner::ssc_planner ALIAS ssc_planner)
target_compile_features(ssc_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ssc_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  ssc_planner
  "planner_base"
  "pluginlib"
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "project_utils"
  "project_utils_msgs"
  "mpl_route_planner"
  "interpolation_utils"
  "motion_model_base"
  "visualization_msgs"
  "mpl_qp_interface"
)
target_link_libraries(ssc_planner yaml-cpp::yaml-cpp ${OpenCV_LIBS} eigen-quadprog::eigen-quadprog)

pluginlib_export_plugin_description_file(planner_base plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ssc_planner PRIVATE "SSC_PLANNER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ssc_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
ament_export_libraries(
  ssc_planner
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
