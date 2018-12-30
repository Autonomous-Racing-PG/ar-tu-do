cmake_minimum_required(VERSION 2.8.3)
project(racer_sensors)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1y)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_msgs geometry_msgs nav_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare car controller node executable
add_executable(racer_sensors
  src/racer_odometry.cpp
  src/main_racer_odometry.cpp)
target_link_libraries(racer_sensors ${catkin_LIBRARIES})
add_dependencies(racer_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
