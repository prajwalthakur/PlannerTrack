cmake_minimum_required(VERSION 3.14)
project(plannertrack_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# motion_model_base's exported target references these packages' targets, so
# they must be found before linking against it.
find_package(project_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(motion_model_base REQUIRED)
# Minimum-curvature raceline (ROS-free core library of min_curv_planner).
find_package(min_curv_planner REQUIRED)

# Compiled core: plannertrack_py._core
pybind11_add_module(_core
  src/sim.cpp
  src/bindings.cpp
  src/raceline_bindings.cpp
)
target_compile_features(_core PRIVATE cxx_std_17)
target_include_directories(_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(_core PRIVATE
  motion_model_base::motion_model_base
  min_curv_planner::min_curv_planner_core
  yaml-cpp::yaml-cpp
)

# Pure-Python layer + compiled core installed side by side in site-packages.
ament_python_install_package(${PROJECT_NAME})
install(TARGETS _core
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Tests import the *installed* package, so run `colcon build` before `colcon test`.
  ament_add_pytest_test(test_plannertrack_py test
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
    TIMEOUT 120
  )
endif()

ament_package()
