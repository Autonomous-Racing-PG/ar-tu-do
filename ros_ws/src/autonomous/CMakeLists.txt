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
add_executable(wall_following src/wall_following.cpp src/pid_controller.cpp src/rviz_geometry_publisher.cpp)
target_link_libraries(wall_following ${catkin_LIBRARIES})
add_dependencies(wall_following ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# emergency stop node
add_executable(emergency_stop src/emergency_stop.cpp)
target_link_libraries(emergency_stop ${catkin_LIBRARIES})
add_dependencies(emergency_stop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
