cmake_minimum_required(VERSION 3.0.2)
project(baxter_roscpp_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(BAXTER_INTERFACE_RESOURCES_DIR "${CMAKE_CURRENT_LIST_DIR}/files")
file(MAKE_DIRECTORY ${BAXTER_INTERFACE_RESOURCES_DIR})
set(BAXTER_INTERFACE_LOGGING_DIR "${CMAKE_CURRENT_LIST_DIR}/logs")
file(MAKE_DIRECTORY ${BAXTER_INTERFACE_LOGGING_DIR})
set(BAXTER_INTERFACE_STATE_RECORDING_DIR "${CMAKE_CURRENT_LIST_DIR}/logs/state_recordings")
file(MAKE_DIRECTORY ${BAXTER_INTERFACE_STATE_RECORDING_DIR})

set(BAXTER_INTERFACE_INCLUDE_DIR "${CATKIN_DEVEL_PREFIX}/include/")
configure_file("include/${PROJECT_NAME}/logging_resources.h.in" "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/logging_resources.h")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  rospy
  std_msgs
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES baxter_roscpp_interface
  CATKIN_DEPENDS baxter_core_msgs roscpp rospy std_msgs trajectory_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BAXTER_INTERFACE_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/limb.cpp
  src/gripper.cpp
  src/state_recorder.cpp
)

add_executable(baxter_state_recorder_test src/state_recorder_test.cpp)
target_link_libraries(baxter_state_recorder_test ${catkin_LIBRARIES} ${PROJECT_NAME})
