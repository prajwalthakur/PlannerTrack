cmake_minimum_required(VERSION 3.14)
project(trajectory_follower_base)

find_package(mpl_cmake REQUIRED)
mpl_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/longitudinal_controller_base.cpp
  src/lateral_controller_base.cpp
)

ament_auto_package()