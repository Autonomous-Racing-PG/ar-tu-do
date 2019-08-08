cmake_minimum_required(VERSION 2.8.3)
project(q_learning)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_msgs
  drive_msgs
  message_generation
)

add_message_files(
  FILES
  EpisodeResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)
