#!/bin/bash
#
#   Cartographer Online Script
#   Must be run in the ros.package directory
#
currentdir=$(pwd)
logpath=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/cartographer.log

echo -n "Creating .urdf file... " | tee $logpath
echo "" >> $logpath
xacro -o $currentdir/ros_ws/src/navigation_stack/car_cartographer/files/racer.urdf --inorder $currentdir/ros_ws/src/simulation/racer_description/urdf/racer.xacro use_gpu:=true visualize_lidar:=true 2>&1 | sed -r "s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g" >> $logpath
echo "Done." | tee -a $logpath

echo "Recording topics... " | tee -a $logpath
rosbag record -j -O ros_ws/src/navigation_stack/car_cartographer/files/recording.bag scan imu __name:=rosbag_recording 2>&1 | sed -r "s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g" >> $logpath &
echo "Please drive with the car now (1.5 laps minimum, 3-5 laps recommended)."
read -p "Press RETURN to stop recording."
rosnode kill /rosbag_recording 2>&1 | sed -r "s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g" >> $logpath
wait
echo "Stopped recording. Done." | tee -a $logpath

if [ $ROS_DISTRO == "kinetic" ] ; then
    echo -n "Create map... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_offline_fast.launch bag_filenames:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag ros_version:=$ROS_DISTRO 2>&1 | sed -r "s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g" >> $logpath
    echo "Done." | tee -a $logpath
fi

if [ $ROS_DISTRO == "melodic" ] ; then
    echo -n "Create .pbstream... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_offline_fast.launch bag_filenames:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag ros_version:=$ROS_DISTRO 2>&1 | sed -r "s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g" >> $logpath
    echo "Done." | tee -a $logpath

    echo -n "Create map... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_make_map.launch bag_filenames:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag pose_graph_filename:=$currentdir/ros_ws/src/navigation_stack/car_cartographer/files/recording.bag.pbstream 2>&1 | sed -r "s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g" >> $logpath
    echo "Done." | tee -a $logpath
fi
