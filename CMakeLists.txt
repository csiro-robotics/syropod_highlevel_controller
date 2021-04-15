# Copyright (c) 2020
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Fletcher Talbot

cmake_minimum_required(VERSION 3.5)
project(syropod_highlevel_controller)

set(PACKAGE_VERSION 0.5.11)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/package.xml")
  file(READ package.xml PACKAGE_XML)
  string(REGEX REPLACE ".*<version>[\\t ]*(.*)[\\t ]*<\\/version>.*" "\\1" PACKAGE_VERSION "${PACKAGE_XML}")
endif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/package.xml")
set(syropod_highlevel_controller_VERSION "${PACKAGE_VERSION}")

# C++ standards setup.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Ensure -fPIC is added.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Additional CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CLANG_TIDY_PROJECT_PREFIX SHC) # clang-tidy option prefix
include(compilerSetup) # Setup compiler warnings.
include(clang-tidy) # clang-tidy support

# Find external depedencies.
# Generally, we should specify either CONFIG to use config style scripts, or MODULE for FindPackage scripts.
find_package(Eigen3 REQUIRED)

# Alias eigen include dirs for catkin/version interopability
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Need message generation commands.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  tf2
  tf2_ros
 )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(config/Dynamic.cfg)

#Generate messages in the 'msg' folder
 add_message_files(
   FILES
   LegState.msg
   TipState.msg
   TargetTipPose.msg
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    dynamic_reconfigure
  DEPENDS
    Eigen3
)

##################################
# Build setup
##################################

# Configure the project config header.
configure_file(shc_config.in.h "${CMAKE_CURRENT_BINARY_DIR}/shc_config.h")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Eigen3_INCLUDE_DIRS}
)

# Project source files. There are two options here:
# 1. Populate the following varibles inline here:
#   - SOURCES for source files to be built and header files associated with built sources.
#   - PUBLIC_HEADERS for all headers to be marshalled as part of the public API.
#     May double up with SOURCES
# 2. Create a file alongside CMakeLists.txt called "sourcelist.cmake" and populate
#    the same varaibles in that file instead, then use "incldue(sourcelist.cmake)" here.
#include(sourcelist.cmake)
# For executables we don't need to concern outselves with PUBLIC_HEADERS as we can assume noone will link to the
# executable. Cases where linking to the executable is requried (e.g., plugins) are beyond the scope of this exercise.
set(SOURCES
  src/admittance_controller.cpp
  src/debug_visualiser.cpp
  src/main.cpp
  src/model.cpp
  src/pose_controller.cpp
  src/state_controller.cpp
  src/walk_controller.cpp
#   include/${PROJECT_NAME}/admittance_controller.h
#   include/${PROJECT_NAME}/debug_visualiser.h
#   include/${PROJECT_NAME}/model.h
#   include/${PROJECT_NAME}/parameters_and_states.h
#   include/${PROJECT_NAME}/pose.h
#   include/${PROJECT_NAME}/pose_controller.h
#   include/${PROJECT_NAME}/standard_includes.h
#   include/${PROJECT_NAME}/state_controller.h
#   include/${PROJECT_NAME}/walk_controller.h
  shc_config.in.h
)



# Build generated header list.
set(GENERATED_FILES
  "${CMAKE_CURRENT_BINARY_DIR}/shc_config.h"
)

# Generate the executable.
add_executable(${PROJECT_NAME}_node include ${SOURCES} ${GENERATED_FILES})
# CMake does not automatically propagate CMAKE_DEBUG_POSTFIX to executables. We do so to avoid confusing link issues
# which can would when building release and debug exectuables to the same path.
# set_target_properties(waypoint_gui_node PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")

# Add dependencies for catkin exports and exports from this project.
# Variables may be empty, so these lines may need to be disabled. For example, in this case
# ${PROJECT_NAME}_EXPORTED_TARGETS is only availabe because we have generated messages for this package.
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)


# Add include directories to the target
# For executables, all includes can be private (see set(SOURCES ...) above).
target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    # Include path for generated files during build.
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    # Add parent directory to support include pattern: #include <project_dir/header.h>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..>
  )

# Add catkin include directories and system include directories.
# Always add ${catkin_INCLUDE_DIRS} with the SYSTEM argument
# These dependenties should be private as much as possible.
target_include_directories(${PROJECT_NAME}_node SYSTEM
  PRIVATE
    "${catkin_INCLUDE_DIRS}"
  )

# Link dependencies.
# Properly defined targets will also have their include directories and those of dependencies added by this command.
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# Enable clang-tidy
clang_tidy_target(${PROJECT_NAME}_node EXCLUDE_MATCHES ".*\\.in($|\\..*)")

# Setup folder display with the target for Visual Studio. This should always be done to match
# the on disk layout of the source files.
source_group("generated" FILES ${GENERATED_FILES})
source_group("source" REGULAR_EXPRESSION ".*$")
# With CMake 3.8+ you can do the following:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# Setup installation.
# Binary installation.
install(TARGETS ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# Header installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# Config and Launch file installation
install(DIRECTORY config launch rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
