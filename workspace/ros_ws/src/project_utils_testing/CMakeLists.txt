cmake_minimum_required(VERSION 3.8)
project(project_utils_testing)

# find_package(mpl_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
# find_package(GTest)

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)


# Export
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
# ament_export_targets(${PROJECT_NAME})
ament_package()



