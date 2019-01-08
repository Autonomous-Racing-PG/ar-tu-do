cmake_minimum_required(VERSION 2.8.3)
project(autonomous)

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

# wall following node 
add_executable(wall_following src/wall_following.cpp)
target_link_libraries(wall_following ${catkin_LIBRARIES})
add_dependencies(wall_following ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# autonomous control
add_executable(autonomous_control src/autonomous_control.cpp)
target_link_libraries(autonomous_control ${catkin_LIBRARIES})
add_dependencies(autonomous_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})