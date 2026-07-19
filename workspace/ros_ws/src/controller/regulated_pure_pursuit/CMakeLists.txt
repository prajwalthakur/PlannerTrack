cmake_minimum_required(VERSION 3.8)
project(regulated_pure_pursuit)
find_package(mpl_cmake REQUIRED)
mpl_package()

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/regulated_pure_pursuit.cpp
  src/optimizer.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "regulatedpp_controller::RegulatedPP"
  EXECUTABLE regulatedpp_controller_exe
)

if(BUILD_TESTING)
  add_subdirectory(regulated_pure_pursuit_unit_tests)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
