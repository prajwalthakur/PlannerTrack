find_package(mpl_cmake REQUIRED)
find_package(project_utils REQUIRED)
find_package(project_utils_testing REQUIRED)
project_add_gtest()

# project_target_link_libraries(
#   project_utils
#   project_utils_testing
# )

# ament_target_dependencies(
#   trajectory_follower_node_unit_tests
#   project_utils_testing
# )

ament_target_dependencies(
  trajectory_follower_node_unit_tests
  project_utils
  project_utils_testing
)
get_target_property(INC_DIRS trajectory_follower_node_unit_tests INCLUDE_DIRECTORIES)
message(STATUS "Include dirs: ${INC_DIRS}")