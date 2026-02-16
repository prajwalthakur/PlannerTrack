###########################################################
#          CMAKE Utility functions
###########################################################

###############################################################################################

# 1. Derive target name from current directory
#completed
function(project_get_dir_name out_var)
    get_filename_component(_dir_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    set(${out_var} ${_dir_name} PARENT_SCOPE)
endfunction()

###############################################################################################

#2. A unit test folder is generally inside of an another folder
# in this case, the cmake will build a dr to build the unit test
#completed
function(project_get_target_name out_target_name)
    project_get_dir_name(curr_dir_name)
    if($(curr_dir_name) MATCHES "unitTests")
        get_filename_component(a_second_last_dir "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
        get_filename_component(a_second_last_last_dir_name "{a_Second_last_dir}" NAME)
        set(${out_target_name} "${a_second_last_last_dir_name}UnitTests" PARENT_SCOPE)
    else()
        set(${out_target_name} ${curr_dir_name} PARENT_SCOPE)
    endif()
endfunction()

#####################################################################################################

#completed
function(project_get_curr_dir_path out_var)
    get_filename_component(_dir_path ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
    set(${out_var} ${_dir_path} PARENT_SCOPE)
endfunction()


###############################################################################################

#completed
function(project_get_subdirectory_list out_var)
    project_get_curr_dir_path(_curr_dir)
    # Get all entries (files + dirs), relative to _curr_dir
    file(GLOB _children * )
    set(subdirs "")
    foreach(_child IN ITEMS ${_children} )
        if(IS_DIRECTORY ${_child})
            list(APPEND subdirs ${_child})
        endif()
    endforeach()
    set(${out_var} ${subdirs} PARENT_SCOPE)
endfunction()


###############################################################################################

#completed
function(project_add_subdirectories)
    project_get_subdirectory_list(_subdir_list)
    foreach(_subdir IN ITEMS ${_subdir_list})
        if(EXISTS ${_subdir}/CMakeLists.txt)
            add_subdirectory(${_subdir})
        endif()
    endforeach()
endfunction()

###############################################################################################

#completed
function(project_add_prep _out_sources_and_headers)
  project_get_target_name(_target_name)

  set(_sources_and_headers)

  # First, collect everything recursively
  file(GLOB_RECURSE _all_sources_and_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.tcc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  )
  # Then, filter out files that are inside subdirectories
  # with their own CMakeLists.txt
  project_get_subdirectory_list(_subdirectory_list)
  foreach(subdir IN ITEMS ${_subdirectory_list})
    #message(FATAL_ERROR "parsing subdirlist  ${subdir} ")
    if(EXISTS  ${subdir}/CMakeLists.txt)
      # Remove any file that belongs to this subdir
      list(FILTER _all_sources_and_headers EXCLUDE REGEX "^${subdir}/.*" )
      #message( "not adding  ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} ")
    endif()
  endforeach()

    #message(STATUS "########## target  ${_target_name} ")
    
    # foreach(_src_header IN ITEMS ${_all_sources_and_headers})
    #     message(STATUS "########## ADDING   ${_src_header} ")  
    # endforeach()

  set(${_out_sources_and_headers} ${_all_sources_and_headers} PARENT_SCOPE)
endfunction()

###############################################################################################

function(project_add_executable_prep is_unit_test)
    #gather sources/headers for this executable
    project_add_prep(_sources_and_headers)
    list(LENGTH _sources_and_headers num_sources)
    if(num_sources EQUAL 0)
        message(FATAL_ERROR
        "project_add_executable_prep: no sources found/provided in ${CMAKE_CURRENT_SOURCE_DIR}")
    endif()   
    # Derive target name from directory
    project_get_target_name(_target_name)

    # Create the executable
    add_executable("${_target_name}" ${_sources_and_headers})

    # Project-specific post steps (link libs, include dirs, opts, etc.)
    #project_add_post()

endfunction()

###############################################################################################

function(project_add_example)
    if(NOT BUILD_EXAMPLES)
    message(FATAL_ERROR
    "project_add_example: BUILD_EXAMPLES not set found/provided in ${CMAKE_CURRENT_SOURCE_DIR}")
        set_target_properties("${_target_name}" PROPERTIES FOLDER "Examples")
        return()
    endif()
    project_get_target_name(_target_name)
    # Build the real example executable (respects SUBSYSTEM, etc.)
    project_add_executable_prep(FALSE )
    # Organize in IDE and improve debugging experience
    # (no need to recompute _target_name; it’s derived from this dir)
    set_target_properties("${_target_name}" PROPERTIES FOLDER "Examples")
    set_property(TARGET "${_target_name}" PROPERTY ENABLE_EXPORTS ON)
    
endfunction()
###############################################################################################



###############################################################################################


## creates library for all source and header including the subdirectories ( excluding the one which has CMAKELISTs.txt)
## library will be named using the current directory name
## files will be grouped in the project based on their path relative to current dir
function(project_add_library)
    project_add_prep(_sources_and_headers)
    project_get_target_name(_target_name)
    add_library(${_target_name} SHARED ${_sources_and_headers})
    
    project_get_dir_name(curr_dir_name)
    target_include_directories( ${_target_name} PUBLIC ${PROJECT_DIR_SRC}/${curr_dir_name})
    target_include_directories( ${_target_name} PUBLIC ${PROJECT_DIR_ROOT})
    #message(WARNING "########## ADDING  BEFORE  ${PROJECT_DIR_SRC}/${curr_dir_name}")
    #project_add_post(${ARGN})
endfunction()

###############################################################################################

# check if a target exists or not
function(project_is_target_exist out_var)
    project_get_target_name(_target_name)
    if(NOT TARGET ${_target_name})
        set(${out_var} OFF PARENT_SCOPE)
        return()
    endif()
    set(${out_var} ON PARENT_SCOPE)
    return()
endfunction()


###############################################################################################

#completed
function(project_target_link_libraries)
    project_get_target_name(_target_name)
    project_is_target_exist(_is_target_exist)
    

    if(NOT ${_is_target_exist})
       return()
    endif()
    foreach(_other_target IN ITEMS ${ARGN} )
        target_link_libraries(${_target_name}  ${_other_target})
        # Include the directories.
        #message(WARNING "########## ADDING  BEFORE  ${PROJECT_DIR_SRC}/${_other_target}")  
        #target_include_directories(${_target_name} PUBLIC ${PROJECT_DIR_SRC}/${_other_target}) 
        if(EXISTS ${PROJECT_DIR_SRC}/${_other_target})
            target_include_directories(${_target_name} PUBLIC ${PROJECT_DIR_SRC}/${_other_target})
            #message(WARNING "########## ADDING   ${PROJECT_DIR_SRC}/${_other_target}")  
        endif()
    endforeach()
endfunction()

###############################################################################################

function(project_add_unit_tests)
    if( NOT BUILD_UNIT_TESTS)
        return()
    endif()
    project_get_target_name(_target_name)
    project_add_executable_prep(TRUE)
    add_custom_command(TARGET ${_target_name} POST_BUILD COMMAND ${_target_name} --gtest_filter=-*Extended)
    project_target_link_libraries(coreTest)
    set_target_properties(${_target_name} PROPERTIES FOLDER UnitTests)
endfunction()



