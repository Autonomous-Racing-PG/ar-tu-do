#!/bin/bash
#
#   Cartographer Online Script
#   Must be run in the ros.package directory
#
filterstring="s/([[:cntrl:]]\[[0-9]{1,3}m)|(..;?)|//g"

scriptdir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
logpath=$scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/cartographer.log

source $scriptdir/../ros_ws/devel/setup.bash

echo -n "Create .urdf file... " | tee $logpath
echo "" >> $logpath
xacro -o $scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/racer.urdf --inorder $scriptdir/../ros_ws/src/simulation/racer_description/urdf/racer.xacro 2>&1 | sed -r $filterstring >> $logpath
echo "Done." | tee -a $logpath

echo "Record topics & building map... " | tee -a $logpath
rosbag record -j -O $scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/map.bag scan imu tf __name:=rosbag_recording 2>&1 | sed -r $filterstring >> $logpath &
roslaunch car_cartographer cartographer_online.launch ros_version:=$ROS_DISTRO 2>&1 | sed -r $filterstring >> $logpath &
echo "Please drive with the car now (1.5 laps minimum, 3-5 laps recommended)."
read -p "Press RETURN to stop recording and finalize map."
rosservice call /finish_trajectory 0 2>&1 | sed -r $filterstring >> $logpath

if [ $ROS_DISTRO == "kinetic" ] ; then
    echo -n "Write map... " | tee -a $logpath
    rosservice call /write_assets "$scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/map" | sed -r $filterstring >> $logpath
    echo "Done." | tee -a $logpath
fi
if [ $ROS_DISTRO == "melodic" ] ; then
    echo -n "Write map stream... " | tee -a $logpath
    rosservice call /write_state "{filename: '$scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/map.bag.pbstream'}" | sed -r $filterstring >> $logpath
    echo "Done." | tee -a $logpath
fi

sleep 5

rosnode kill /rosbag_recording 2>&1 | sed -r $filterstring >> $logpath
rosnode kill /cartographer_node 2>&1 | sed -r $filterstring >> $logpath
rosnode kill /rviz_cartographer 2>&1 | sed -r $filterstring >> $logpath

if [ $ROS_DISTRO == "melodic" ] ; then
    echo -n "Write map... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_make_map.launch bag_filenames:=$scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/map.bag pose_graph_filename:=$scriptdir/../ros_ws/src/navigation_stack/car_cartographer/files/map.bag.pbstream 2>&1 | sed -r $filterstring >> $logpath
    echo "Done." | tee -a $logpath
fi
