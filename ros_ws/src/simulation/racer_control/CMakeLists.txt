cmake_minimum_required(VERSION 2.8.3)
project(racer_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    drive_msgs
)


###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS rospy std_msgs drive_msgs
)
