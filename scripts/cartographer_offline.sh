#!/bin/bash
#
#   Cartographer Offline Script
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

if [ $ROS_DISTRO == "kinetic" ] ; then
    echo -n "Create map... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_offline_fast.launch bag_filenames:=$1 ros_version:=$ROS_DISTRO 2>&1 | sed -r $filterstring >> $logpath
    echo "Done." | tee -a $logpath
fi

if [ $ROS_DISTRO == "melodic" ] ; then
    echo -n "Create map stream... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_offline_fast.launch bag_filenames:=$1 ros_version:=$ROS_DISTRO 2>&1 | sed -r $filterstring >> $logpath
    echo "Done." | tee -a $logpath

    echo -n "Create map... " | tee -a $logpath
    echo "" >> $logpath
    roslaunch car_cartographer cartographer_make_map.launch bag_filenames:=$1 pose_graph_filename:=$1.pbstream 2>&1 | sed -r $filterstring >> $logpath
    echo "Done." | tee -a $logpath
fi
