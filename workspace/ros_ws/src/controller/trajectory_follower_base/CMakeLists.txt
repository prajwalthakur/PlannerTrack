cmake_minimum_required(VERSION 3.14)
project(trajectory_follower_base)
find_package(mpl_cmake REQUIRED)
mpl_package()



find_package(Eigen3 REQUIRED)
include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)



ament_auto_add_library(${PROJECT_NAME} SHARED
  src/longitudinal_controller_base.cpp
  src/lateral_controller_base.cpp
  src/hybrid_controller_base.cpp
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)