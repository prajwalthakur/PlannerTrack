#
# src/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.14)
project(core LANGUAGES CXX)

# option(BUILD_DOC "Build doxygen documentation" OFF)
option(BUILD_EXAMPLES "Build Examples" OFF)
option(BUILD_UNIT_TESTS "Build Unit tests" OFF)
set(PROJECT_DIR_ROOT ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "Root directory" FORCE)
set(PROJECT_DIR_SRC ${PROJECT_DIR_ROOT} CACHE STRING "Root 'Src' directory" FORCE)
#MESSAGE(WARNING " PROJECT_DIR_SRC ${PROJECT_DIR_SRC} ")
# Path to helper functions
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CmakeUtils)

# Add modules
add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(vehicleModel)








