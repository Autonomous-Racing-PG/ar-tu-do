#!/usr/bin/env bash

# set -x

# Code is stored in the VM itself
# CODE_ROOT=/home/vagrant

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

rosdep update

# Code is stored in a directory shared between the VM and the host.
CODE_ROOT=/vagrant
WORKING_DIR_NAME=ar-tu-do
DEV_ENV="${CODE_ROOT}/${WORKING_DIR_NAME}"
REPO="https://github.com/arpg-sophisticated/ar-tu-do.git"

# test if the dev_env directory already exists
if [ -d "$DEV_ENV" ]; then
    echo "WARNING: Working directory ${DEV_ENV} already exists, skipping git clone"
else
    cd $CODE_ROOT
    git clone $REPO $WORKING_DIR_NAME
    d $DEV_ENV
    git submodule init
    git submodule update --recursive
    rosdep update
    cd ros_ws/
    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
fi

# after logging in, cd directly to the code directory
echo "cd $DEV_ENV" >> ~/.bashrc
