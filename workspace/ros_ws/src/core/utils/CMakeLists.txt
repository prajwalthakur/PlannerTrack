cmake_minimum_required(VERSION 3.14)
project_add_library()
find_package(Eigen3 QUIET)
project_target_link_libraries(core)
