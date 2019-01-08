# Project Title

Autonomous Racing - Project Group - TU Dortmund

[![Build Status](https://travis-ci.com/Autonomous-Racing-PG/ros.package.svg?branch=master)](https://travis-ci.com/Autonomous-Racing-PG/ros.package)

## Getting Started

These instructions will get you a copy of the project up and running

### Move to ROS Workspace

```
cd ros_ws
```

### Build ROS packages

```
catkin_make
```

### Run routines

```
source devel/setup.bash (or setup.zsh)
```

Now several routines can be started by executing the launch-files inside the **launch/** directory. E.g.

```
roslaunch launch/gazebo_car-teleop.launch
```

### Run tests

```
catkin_make run_tests
```

## Documentation

* For general information and documentation checkout the [wiki page](https://github.com/Autonomous-Racing-PG/ros.package/wiki).
* For source code documentation checkout the auto-generated [Doxygen documentation](https://autonomous-racing-pg.github.io/ros.package/html/index.html).

## License

This project is licensed under the MIT and GPLv3 dual licensed - see the [MIT.LICENSE](MIT.LICENSE) and [GPLv3.LICENSE](GPLv3.LICENSE) file for details

## Acknowledgments

* TU Dortmund


