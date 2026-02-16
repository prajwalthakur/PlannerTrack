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
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  eigen3_cmake_module
  Eigen3
  rosidl_default_generators
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# =============================
# Messages
# =============================
set(MSG_FILES
  "msg/EigenVector.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

# =============================
# project_utils library
# =============================
add_library(project_utils_lib SHARED
  src/main.cpp
)

target_include_directories(project_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(project_utils_lib
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =============================
# Build core (pure CMake project)
# =============================
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../core
  ${CMAKE_BINARY_DIR}/core_build
)

# =============================
# INSTALL + EXPORT CORE TARGETS  
# =============================

# Install core libraries AND export them as CMake targets
install(TARGETS
  core
  utils
  vehicleModel
  EXPORT coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT coreTargets
  NAMESPACE project_utils::
  DESTINATION share/${PROJECT_NAME}/cmake
)
# =============================
# INSTALL CORE HEADERS
# =============================
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../core/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# =============================
# Install project_utils headers
# =============================
install(
  DIRECTORY include/
  DESTINATION include
)

# =============================
# Install configs & maps
# =============================
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# =============================
# Install project_utils library
# =============================
install(TARGETS project_utils_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# =============================
# EXPORTS (for downstream packages)
# =============================

# Export installed include directory
ament_export_include_directories(include)

#Export REAL CMake targets (NOT raw -l flags)
ament_export_targets(coreTargets HAS_LIBRARY_TARGET)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# =============================
# Testing
# =============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
