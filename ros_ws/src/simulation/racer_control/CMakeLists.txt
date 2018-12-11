cmake_minimum_required(VERSION 2.8.3)
project(racer_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  rospy
  std_msgs
)


###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS drive_msgs roscpp rospy std_msgs
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
add_executable(drive_param_converter src/drive_param_converter.cpp)
target_link_libraries(drive_param_converter ${catkin_LIBRARIES})
add_dependencies(drive_param_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
