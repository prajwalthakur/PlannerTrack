cmake_minimum_required(VERSION 3.8)
project(project_utils)




# =============================
# Compiler options
# =============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Dependencies
# =============================
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED)
# find_package(project_utils_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  Boost
  eigen3_cmake_module
  Eigen3
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_geometry_msgs
  project_utils_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# =============================
# Library
# =============================
add_library(project_utils SHARED
  src/main.cpp
  src/integrator.cpp
  src/common_utils.cpp
  src/pose_definition.cpp
  src/unique_id.cpp
  src/unique_id_generator.cpp
  src/random_type.cpp
  src/random.cpp
  src/planning_utils.cpp
)

target_link_libraries(project_utils
  Eigen3::Eigen
)
target_include_directories(project_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(project_utils
  rclcpp
  project_utils_msgs
  tf2_geometry_msgs
  nav_msgs
)
# =============================
# Install
# =============================
install(TARGETS project_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )



# =============================
# Export
# =============================
ament_export_include_directories(include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  project_utils_msgs
  tf2_geometry_msgs
  nav_msgs
)

ament_package()