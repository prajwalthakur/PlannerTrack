cmake_minimum_required(VERSION 3.8)
project(motion_model_shapes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(project_utils REQUIRED)
find_package(motion_model_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(yaml-cpp REQUIRED)

# SHARED is required, not just conventional: pluginlib discovers this class via
# dlopen() at runtime, which cannot load a static archive.
add_library(motion_model_shapes SHARED
  src/collision_model/ellipse_collision_footprint.cpp
  src/geometric_model/rectangular_geometric_model.cpp
)
add_library(motion_model_shapes::motion_model_shapes ALIAS motion_model_shapes)
target_compile_features(motion_model_shapes PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(motion_model_shapes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  motion_model_shapes
  "project_utils"
  "motion_model_base"
  "pluginlib"
  "Eigen3"
  "eigen3_cmake_module"
)
target_link_libraries(motion_model_shapes yaml-cpp::yaml-cpp)

pluginlib_export_plugin_description_file(motion_model_base plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(motion_model_shapes PRIVATE "MOTION_MODEL_SHAPES_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS motion_model_shapes
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
  motion_model_shapes
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
