cmake_minimum_required(VERSION 3.14)
project(mpl_rclcpp_utils)

find_package(mpl_cmake REQUIRED)
mpl_package()

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_auto_add_gtest(test_${PROJECT_NAME} ${test_files})
endif()

ament_auto_package()