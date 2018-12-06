cmake_minimum_required(VERSION 2.8.3)
project(teleoperation)

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

# Declare remote joy node executable
add_executable(remote_joy src/remote_joy.cpp)
target_link_libraries(remote_joy ${catkin_LIBRARIES})
add_dependencies(remote_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare remote keyboard node executable
add_executable(remote_keyboard src/remote_keyboard.cpp)
target_link_libraries(remote_keyboard ${catkin_LIBRARIES})
add_dependencies(remote_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})