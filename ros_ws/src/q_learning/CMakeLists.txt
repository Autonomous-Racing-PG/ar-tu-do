cmake_minimum_required(VERSION 2.8.3)
project(q_learning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  drive_msgs
)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1y)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Declare crash detector node executable
add_executable(crash_detector src/crash_detector.cpp)
target_link_libraries(crash_detector ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(crash_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})