cmake_minimum_required(VERSION 2.8.3)
project(car_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1y)

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
  CATKIN_DEPENDS roscpp rospy std_msgs drive_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare car control node executable
add_executable(car_control src/car_control.cpp)
target_link_libraries(car_control ${catkin_LIBRARIES})
add_dependencies(car_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_car_control.cpp)
