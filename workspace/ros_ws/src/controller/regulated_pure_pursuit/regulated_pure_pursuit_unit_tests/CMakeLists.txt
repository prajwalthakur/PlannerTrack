find_package(mpl_cmake REQUIRED)
find_package(project_utils REQUIRED)
find_package(project_utils_testing REQUIRED)
find_package(rclcpp REQUIRED)

project_add_gtest()


target_link_libraries(
  regulated_pure_pursuit_unit_tests
  regulated_pure_pursuit
)

ament_target_dependencies(
  regulated_pure_pursuit_unit_tests
  rclcpp
  project_utils
  project_utils_testing
)

install(
  DIRECTORY test_config/
  DESTINATION share/${PROJECT_NAME}/test_config
)
get_target_property(INC_DIRS regulated_pure_pursuit_unit_tests INCLUDE_DIRECTORIES)
message(STATUS "Include dirs: ${INC_DIRS}")

