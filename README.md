# hero_chassis_controller

## Overview

This is a hero chassis controller which is used to control the Robomaster
hero chassis.

**Keywords:** RoboMaster, ROS, ros_control

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Youjian WU<br />
Affiliation: GDUT<br />
Maintainer: Youjian Wu, 1124895509@qq.com**

The hero_chassis_controller package has been tested under [ROS] Noetic on respectively 18.04 and 20.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- control_toolbox
- geometry_msgs
- control_msgs
- realtime_tools
- tf
- nav_msgs
- pluginlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/wuyoujian666/hero_chassis_controller.git
    # git clone https://github.com/wuyoujian666/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller run_simulation_and_controller.launch

## Config files

Config file config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.

## Launch files

* **run_simulation_and_controller.launch:** Hero chassis only simulation and simple chassis controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues)
.

[ROS]: http://www.ros.org
