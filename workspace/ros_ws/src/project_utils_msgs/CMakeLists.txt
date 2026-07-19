cmake_minimum_required(VERSION 3.8)
project(project_utils_msgs)

# =============================
# Compiler options
# =============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Dependencies
# =============================
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rosidl_default_generators
  geometry_msgs
  std_msgs
  nav_msgs
  builtin_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# =============================
# Messages
# =============================
set(MSG_FILES
  "msg/EigenVector.msg"
  "msg/ackermann/control/Control.msg"
  "msg/ackermann/control/ControlHorizon.msg"
  "msg/ackermann/control/Lateral.msg"
  "msg/ackermann/control/Longitudinal.msg"
  "msg/ackermann/planning/PathPoint.msg"
  "msg/ackermann/planning/Path.msg"
  "msg/ackermann/planning/TrajectoryPoint.msg"
  "msg/ackermann/planning/Trajectory.msg"
  "msg/ackermann/vehicle/SteeringReport.msg"
  "msg/common/Float64Stamped.msg"
  "msg/debug_msgs/Float32MultiArrayStamped.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
    builtin_interfaces
)



install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()


