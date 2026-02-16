cmake_minimum_required(VERSION 3.14)
project_add_library()
# Optional: Eigen is already a dependency of pinocchio target, but finding it explicitly is harmless
find_package(Eigen3 QUIET)
find_package(yaml-cpp REQUIRED)
project_target_link_libraries(yaml-cpp)
project_add_subdirectories()