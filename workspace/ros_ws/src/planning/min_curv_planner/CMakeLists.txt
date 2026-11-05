cmake_minimum_required(VERSION 3.8)
project(min_curv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dense linear algebra: unoptimized builds are much slower.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen-quadprog REQUIRED)  # QP solver (same algorithm as Python quadprog)
find_package(planner_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(project_utils REQUIRED)
find_package(project_utils_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# =============================
# Core: the algorithm, no ROS (Eigen + yaml-cpp only)
# =============================
add_library(min_curv_planner_core SHARED
  src/closed_spline.cpp
  src/min_curvature.cpp
  src/min_curv_optimizer.cpp
  src/params.cpp
  src/track_io.cpp
  src/track_preprocessing.cpp
  src/velocity_profile.cpp
)
target_compile_features(min_curv_planner_core PUBLIC cxx_std_17)
target_include_directories(min_curv_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(min_curv_planner_core PUBLIC Eigen3::Eigen yaml-cpp::yaml-cpp
  PRIVATE eigen-quadprog::eigen-quadprog)

# =============================
# Plugin: MinCurvPlanner (PlannerBase)
# =============================
add_library(min_curv_planner SHARED
  src/min_curv_planner.cpp
)
target_compile_features(min_curv_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(min_curv_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  min_curv_planner
  "planner_base"
  "pluginlib"
  "rclcpp"
  "project_utils"
  "project_utils_msgs"
  "visualization_msgs"
)
target_link_libraries(min_curv_planner min_curv_planner_core yaml-cpp::yaml-cpp)

pluginlib_export_plugin_description_file(planner_base plugins.xml)

# =============================
# Install
# =============================
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS min_curv_planner_core min_curv_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =============================
# Tests
# =============================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_min_curv_optimizer
    test/test_closed_spline.cpp
    test/test_min_curvature.cpp
    test/test_track_io.cpp
    test/test_min_curv_optimizer.cpp
    test/test_pipeline.cpp
  )
  target_link_libraries(test_min_curv_optimizer min_curv_planner_core)
  target_compile_definitions(test_min_curv_optimizer PRIVATE
    RACING_WAYPOINTS_JSON="${CMAKE_CURRENT_SOURCE_DIR}/../../scenarios/ground_vehicle_racing/map/global_waypoints.json"
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  yaml-cpp
  eigen-quadprog
  planner_base
  pluginlib
  rclcpp
  project_utils
  project_utils_msgs
  visualization_msgs
)
ament_package()
