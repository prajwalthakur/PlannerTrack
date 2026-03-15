cmake_minimum_required(VERSION 3.14)
find_package(Eigen3 QUIET)
find_package(rclcpp REQUIRED)
project_add_subdirectories()
project_add_library()
project_target_link_libraries(
    core
    rclcpp::rclcpp
    Eigen3::Eigen
)

