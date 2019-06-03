cmake_minimum_required(VERSION 2.8.3)
project(vesc_sim)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  roslib
  rospy
  std_msgs
  vesc_msgs
  nav_msgs
  geometry_msgs
  tf
)
find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS drive_msgs roscpp roslib rospy std_msgs vesc_msgs nav_msgs geometry_msgs tf
  DEPENDS yaml-cpp
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Declare joystick controller node executable
add_executable(vesc_sim_driver
src/main.cpp
src/vesc_sim_driver.cpp
src/vesc_sim.cpp
)

target_link_libraries(vesc_sim_driver ${catkin_LIBRARIES})
add_dependencies(vesc_sim_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(params_to_yaml src/params_to_yaml.cpp)
target_link_libraries(params_to_yaml ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(params_to_yaml ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
