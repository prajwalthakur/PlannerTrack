cmake_minimum_required(VERSION 3.8)
project(mpl_qp_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Find Dependencies
# =============================
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  eigen3_cmake_module
  Eigen3
  rclcpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(osqp REQUIRED)
get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

# =============================
# Library
# =============================
set(QP_INTERFACE_LIB_SRC
  src/qp_interface.cpp
  src/osqp_interface.cpp
  src/osqp_csc_matrix_conv.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${QP_INTERFACE_LIB_SRC}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  SYSTEM PUBLIC
    "${OSQP_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
)

target_link_libraries(${PROJECT_NAME}
  osqp::osqp
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =============================
# Install
# =============================
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# =============================
# Export
# =============================
ament_export_include_directories(include "${OSQP_INCLUDE_DIR}")
ament_export_libraries(${PROJECT_NAME} osqp::osqp)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# =============================
# Tests
# =============================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_qp_interface
    test/test_osqp_interface.cpp
    test/test_csc_matrix_conv.cpp
    test/test_qp_interface.cpp
  )
  target_link_libraries(test_qp_interface ${PROJECT_NAME})
endif()

ament_package()