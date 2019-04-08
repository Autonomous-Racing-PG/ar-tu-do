#!/bin/bash
#
#   Cartographer Online Script
#   Must be run in the ros.package directory
#
currentdir=$(pwd)
echo -n "Creating .urdf file..."
xacro -o $currentdir/ros_ws/src/navigation_stack/car_cartographer/files/racer.urdf --inorder $currentdir/ros_ws/src/simulation/racer_description/urdf/racer.xacro use_gpu:=true visualize_lidar:=true >/dev/null 2>/dev/null
echo " Done."
echo "Recording topics..."
rosbag record -j -O ros_ws/src/navigation_stack/car_cartographer/files/recording.bag scan imu __name:=rosbag_recording >/dev/null 2>/dev/null &
echo "Please drive with the car now (1.5 laps minimum, 3-5 laps recommended)."
read -p "Press RETURN to stop recording."
rosnode kill /rosbag_recording >/dev/null
wait
echo "Stopped recording. Done."
echo -n "Create .pbstream..."
roslaunch car_cartographer cartographer_offline_fast.launch bag_filenames:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag ros_version:=$ROS_DISTRO >/dev/null 2>/dev/null
echo " Done."
echo -n "Create map..."
roslaunch car_cartographer cartographer_make_map.launch bag_filenames:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag pose_graph_filename:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag.pbstream >/dev/null 2>/dev/null
echo " Done."