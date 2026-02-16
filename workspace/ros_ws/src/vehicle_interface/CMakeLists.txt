cmake_minimum_required(VERSION 3.8)
project(vehicle_interface)


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Find Dependencies
# =============================
set(THIS_PACKAGE_INCLUDE_DEPENDS
    eigen3_cmake_module
    Eigen3
    project_utils
    ament_cmake
    rclcpp
    std_msgs

)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core
#                  ${CMAKE_BINARY_DIR}/core_build)

# =============================
# Build Vehicle Interface Class
# =============================
add_library(vehicle_interface_class
  src/vehicle_interface_class.cpp
)


target_include_directories(vehicle_interface_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(vehicle_interface_class PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(vehicle_interface_class
  PUBLIC
    project_utils::core
    project_utils::utils
    project_utils::vehicleModel
)

# =============================
# Build Vehicle Node
# =============================
add_executable(vehicle_interface_node src/vehicle_interface_node.cpp)

target_link_libraries(vehicle_interface_node vehicle_interface_class )



# =============================
# Build MPC Node
# =============================


add_executable(mpc_node src/mpc_node.cpp)
# # Include HPIPM and BLASFEO if available
target_include_directories(mpc_node PRIVATE
  /hpipm/include
  /blasfeo/include
)

target_link_libraries(mpc_node
  PUBLIC
    vehicle_interface_class
  PRIVATE
    /hpipm/lib/libhpipm.so
    /blasfeo/lib/libblasfeo.so
)





# =============================
# Install Targets
# =============================
install(TARGETS
  vehicle_interface_node
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

# =============================
# Install Launch Files and Config Files
# =============================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# =============================
# Testing Configuration
# =============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip linter warnings (if needed)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

# =============================
# Export Package
# =============================
ament_package()
