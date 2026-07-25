cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(project_utils REQUIRED)
find_package(project_utils_msgs REQUIRED)
find_package(motion_model_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(pid_controller_lib SHARED
  src/pid_controller.cpp
  src/simple_pid.cpp
)

target_include_directories(pid_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Deliberately does NOT depend on motion_model_ground_vehicles (or any other
# concrete plugin package): this node only ever talks to DynamicModel's base
# interface (AgentModel::dynamicModel().packAccelSteerRate()), same as
# VehicleModelFactory itself, so pluginlib remains the only thing that
# actually loads whichever concrete .so a given agent's YAML names. Directly
# linking a concrete plugin package here would break pluginlib's own runtime
# loading of it (see git history/PR discussion if this changes again: the OS
# loader would map the plugin .so eagerly at process startup, before
# pluginlib::ClassLoader::loadLibrary() runs, so its later dlopen() becomes a
# no-op refcount bump and the plugin's factory never gets associated with
# that ClassLoader instance).
ament_target_dependencies(pid_controller_lib
  rclcpp
  rclcpp_components
  project_utils
  project_utils_msgs
  motion_model_base
  pluginlib
  nav_msgs
)

target_link_libraries(pid_controller_lib yaml-cpp)

rclcpp_components_register_node(pid_controller_lib
  PLUGIN "mpl::control::pid_controller::PidControllerNode"
  EXECUTABLE pid_controller_node_exe
)

install(
  TARGETS pid_controller_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pid_controller_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  project_utils
  project_utils_msgs
  motion_model_base
  pluginlib
  nav_msgs
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
