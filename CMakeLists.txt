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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS baxter_core_msgs roscpp rospy std_msgs trajectory_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/limb.cpp
  src/gripper.cpp
  src/state_recorder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

