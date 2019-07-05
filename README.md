# Autonomous Racing - Project Group - TU Dortmund

[![Build Status](https://travis-ci.com/Autonomous-Racing-PG/ros.package.svg?branch=master)](https://travis-ci.com/Autonomous-Racing-PG/ros.package)

This repository contains the results of the TU Dortmund Autononous Racing project group in chair XII of the computer science faculty. The set-up is based on the [F1/10](http://f1tenth.org/) competition which puts autonomous car-like robots in a race around a racetrack against the time or each other.

## Getting Started

These instructions will get you a copy of the project up and running.

### Install missing system dependencies

The project runs on the [Robot Operating System (ROS)](https://www.ros.org/), which needs to be installed first (unless already happened). The target version of ROS for this project is [ROS Kinetic](http://wiki.ros.org/kinetic/Installation), although it seems to also work on [ROS Melodic](http://wiki.ros.org/melodic/Installation) without any problems.

(Note: Gazebo 7.0.0, which is installed with ROS Kinetic by default, does not work on a virtual machine ([ref](https://bitbucket.org/osrf/gazebo/issues/1837/vmware-rendering-z-ordering-appears-random)). To solve this, Gazebo has to be updated to at least 7.4.0 by using [the step-by-step instructions in this tutorial](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0#Alternativeinstallation:step-by-step).)

Other dependencies:

```bash
sudo pip uninstall pip && sudo apt install python-pip
sudo apt install git libsdl2-dev python-pip clang-format python-pyqtgraph
pip install torch autopep8 cython

# RangeLibc
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
```

### Install missing ROS dependencies
```bash
cd arpg/ros_ws
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```

### Build ROS packages

```bash
catkin_make
```

### Run routines

```bash
source devel/setup.bash (or setup.zsh, depending on your shell)
```

Now several routines can be started by executing the launch-files inside the `launch` directory. E.g.

```bash
roslaunch launch/car.launch (Real car, Wallfollowing)
roslaunch launch/gazebo.launch (Simulation, Wallfollowing)
roslaunch launch/navigation_stack.launch (Simulation, SLAM & ROS navigation)
roslaunch launch/qlearning.launch (Simulation, Machine Learning/ Q-Learning)
```

Several arguments can be passed to the launch files above. The syntax for passing an argument is `argname:=value`; for example using the `gazebo.launch` file without emergency stop and with car highlighting can be done using `roslaunch launch/gazebo.launch emergency_stop:=false car_highlighting:=true`.  Note that not every of these launch files supports every argument below; refer to the table below for an exhaustive list of supported arguments.

<table>
  <tr>
    <th rowspan="2">Name</th>
    <th colspan="4">Supported by <code>launch/&lt;file&gt;.launch</code></th>
    <th rowspan="2">Description</th>
  </tr>
  <tr>
    <td>car</td>
    <td>gazebo</td>
    <td>navigation_stack</td>
    <td>qlearning</td>
  </tr>
  <tr>
    <td><code>car_highlighting</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✓</td>
    <td>Boolean value whether the car should glow green in Gazebo for better visibility. Usually defaults to false.</td>
  </tr>
  <tr>
    <td><code>debug</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✗</td>
    <td>Boolean value whether Gazebo should run in debug mode. Defaults to false.</td>
  </tr>
  <tr>
    <td><code>emergency_stop</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✗</td>
    <td>✗</td>
    <td>Boolean value whether the emergency stop should be active. Defaults to true.</td>
  </tr>
  <tr>
    <td><code>gui</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✓</td>
    <td>Boolean value whether Gazebo should show a user interface. Defaults to true.</td>
  </tr>
  <tr>
    <td><code>joystick_type</code></td>
    <td>✓</td>
    <td>✓</td>
    <td>✓</td>
    <td>✗</td>
    <td>The type of joystick controller. Possible values: <code>ps3</code>, <code>xbox360</code> and <code>xboxone</code></td>
  </tr>
  <tr>
    <td><code>map</code></td>
    <td>✗</td>
    <td>✗</td>
    <td>✓</td>
    <td>✗</td>
    <td>Name of the map to be used by the particle filter. Defaults to a prerecorded map of <code>racetrack_decorated_2</code>.</td>
  </tr>
  <tr>
    <td><code>paused</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✗</td>
    <td>Boolean value whether Gazebo should start paused. Defaults to false.</td>
  </tr>
  <tr>
    <td><code>pf_angle_step</code></td>
    <td>✗</td>
    <td>✗</td>
    <td>✓</td>
    <td>✗</td>
    <td>Angle step of the particle filter. Defaults to 18.</td>
  </tr>
  <tr>
    <td><code>pf_max_particles</code></td>
    <td>✗</td>
    <td>✗</td>
    <td>✓</td>
    <td>✗</td>
    <td>Maximum amount of particles to be used by the particle filter. Defaults to 500.</td>
  </tr>
  <tr>
    <td><code>pf_squash_factor</code></td>
    <td>✗</td>
    <td>✗</td>
    <td>✓</td>
    <td>✗</td>
    <td>Squash factor of the particle filter. Defaults to 2.2.</td>
  </tr>
  <tr>
    <td><code>plot_window</code></td>
    <td>✗</td>
    <td>✗</td>
    <td>✗</td>
    <td>✓</td>
    <td>Integer value indicating the amount of episodes that should be plotted. Defaults to 200.</td>
  </tr>
  <tr>
    <td><code>realtime_simulation</code></td>
    <td>✗</td>
    <td>✗</td>
    <td>✗</td>
    <td>✓</td>
    <td>Boolean value whether Gazebo should try to simulate with a real time factor of 1. If false, Gazebo tries to simulate as fast as possible. Defaults to true.</td>
  </tr>
  <tr>
    <td><code>use_gpu</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✓</td>
    <td>Boolean value whether Gazebo should use the GPU when simulating the lidar. Defaults to true.</td>
  </tr>
  <tr>
    <td><code>use_sim_time</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✗</td>
    <td>Boolean value whether all ros nodes should use simulated Gazebo time instead of wall clock time. Defaults to true.</td>
  </tr>
  <tr>
    <td><code>verbose</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✗</td>
    <td>Boolean value whether Gazebo should give verbose standard output. Defaults to true.</td>
  </tr>
  <tr>
    <td><code>visualize_lidar</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✗</td>
    <td>Boolean value whether Gazebo should show the simulated lidar rays. Defaults to false.</td>
  </tr>
  <tr>
    <td><code>world</code></td>
    <td>✗</td>
    <td>✓</td>
    <td>✓</td>
    <td>✓</td>
    <td>The name of the racetrack. Possible values: <code>racetrack_decorated</code>, <code>racetrack_decorated_2</code> (default) and <code>racetrack_decorated_2_big</code></td>
  </tr>
</table>

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

This project (excluding git submodules) is under MIT and GPLv3 dual licensed - see the [MIT.LICENSE](MIT.LICENSE) and [GPLv3.LICENSE](GPLv3.LICENSE) file for details.

## Acknowledgments

* TU Dortmund


