cmake_minimum_required(VERSION 2.8.3)
project(neuralnetwork)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -lfann -lm)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  message_generation
)

find_package(gazebo REQUIRED)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  net_param.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS
  drive_msgs
  roscpp
  rospy
  std_msgs
  message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

find_library(FANNLIB
    NAMES fann
)

# ai driver node
add_executable(ai_driver src/ai_driver.cpp)
target_link_libraries(ai_driver ${catkin_LIBRARIES} ${FANNLIB})
add_dependencies(ai_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ai trainer node
add_executable(ai_trainer src/ai_trainer.cpp)
target_link_libraries(ai_trainer ${catkin_LIBRARIES} ${FANNLIB})
add_dependencies(ai_trainer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})