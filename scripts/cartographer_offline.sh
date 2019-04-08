#!/bin/bash
#
#   Cartographer Offline Script
#   Must be run in the ros.package directory
#
currentdir=$(pwd)
echo -n "Creating .urdf file..."
xacro -o $currentdir/ros_ws/src/navigation_stack/car_cartographer/files/racer.urdf --inorder $currentdir/ros_ws/src/simulation/racer_description/urdf/racer.xacro use_gpu:=true visualize_lidar:=true >/dev/null 2>/dev/null
echo " Done."
echo -n "Create .pbstream..."
roslaunch car_cartographer cartographer_offline_fast.launch bag_filenames:=$1 ros_version:=$ROS_DISTRO >/dev/null 2>/dev/null
echo " Done."
echo -n "Create map..."
roslaunch car_cartographer cartographer_make_map.launch bag_filenames:=$1 pose_graph_filename:=$1.pbstream >/dev/null 2>/dev/null
echo " Done."