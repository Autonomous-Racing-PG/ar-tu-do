# Project Title

Autonomous Racing - Project Group - TU Dortmund

[![Build Status](https://travis-ci.com/Autonomous-Racing-PG/ros.package.svg?branch=master)](https://travis-ci.com/Autonomous-Racing-PG/ros.package)

## Getting Started

These instructions will get you a copy of the project up and running

### Install missing system dependencies
```bash
sudo apt install libsdl2-dev python-pip clang-format
pip install torch autopep8

# RangeLibc
sudo pip uninstall pip && sudo apt install python-pip
pip install cython
git clone http://github.com/kctess5/range_libc
cd range_libc/pywrapper
# Either:
./compile.sh            # on VM
# Or:
./compile_with_cuda.sh  # on car - compiles GPU ray casting methods
```

### Clone the Project

```bash
git clone --recurse-submodules https://github.com/Autonomous-Racing-PG/ros.package.git arpg
cd arpg
```

### Move to ROS Workspace

```bash
cd ros_ws
```

### Install missing ROS dependencies
```bash
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```

### Build ROS packages

```bash
catkin_make
```

### Run routines

```bash
source devel/setup.bash (or setup.zsh)
```

Now several routines can be started by executing the launch-files inside the **launch/** directory. E.g.

```bash
roslaunch launch/gazebo_car-teleop.launch
```

### Run tests

```bash
catkin_make run_tests
```

## Building a map with Cartographer

There are two bash scripts in the `scripts` folder which use [Cartographer](https://github.com/googlecartographer/cartographer_ros) to create a map of a racetrack. This map can then be used for different purposes, for example in the ROS navigation stack.

* To build a map while a roscore is running and providing sensor data, use the `cartographer_online` script.
* To build a map from a rosbag, use the `cartographer_offline` script. The rosbag must provide range data on the rostopic `/scan` and a transformation tree on `/tf`; depending on your configuration of cartographer in `car_cartographer/config` it may need to also have odometry data on `/odom` or IMU data on `/imu`.

```bash
# Either:
./scripts/cartographer_online.sh
# Or:
./scripts/cartographer_offline.sh /absolute/path/to/rosbag
```

## Documentation

* For general information and documentation checkout the [wiki page](https://github.com/Autonomous-Racing-PG/ros.package/wiki).
* For source code documentation checkout the auto-generated [Doxygen documentation](https://autonomous-racing-pg.github.io/ros.package/html/index.html).

## License

This project (exluded git submodules) is licensed under the MIT and GPLv3 dual licensed - see the [MIT.LICENSE](MIT.LICENSE) and [GPLv3.LICENSE](GPLv3.LICENSE) file for details

## Acknowledgments

* TU Dortmund


