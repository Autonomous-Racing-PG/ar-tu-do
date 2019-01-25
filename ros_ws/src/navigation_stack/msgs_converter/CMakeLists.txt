cmake_minimum_required(VERSION 2.8.3)
project(msgs_converter)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  rospy
  std_msgs
  vesc_msgs
  nav_msgs
  vesc_sim
)


###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS drive_msgs roscpp rospy std_msgs vesc_msgs nav_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare joystick controller node executable
add_executable(msgs_converter
src/main.cpp
src/msgs_converter.cpp
)

target_link_libraries(msgs_converter ${catkin_LIBRARIES})
add_dependencies(msgs_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
