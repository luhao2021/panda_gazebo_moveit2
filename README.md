

# panda_gazebo_moveit2

This repository includes software packages that enable the motion planning with Franka Emika Panda with MoveIt2 and the Gazebo simulator.

![scene](https://github.com/user-attachments/assets/d9f3fa4e-78b3-4dfd-8440-270fed293023)
![planning](https://github.com/user-attachments/assets/91b50f4e-37fd-4dd6-8df6-caca3e1d8c04)

## Overview

This package is tested with Ubuntu 22.04 Jammy, ROS 2 `humble` and Gazebo `fortress`. This follows the recommendataion of [Getting Started with Gazebo](https://gazebosim.org/docs/fortress/getstarted/).

The sturcture of this packages are described as below:

- [**panda_gazebo_demo**](./panda_gazebo_demo) – Lauch files, scripts of the demo
- [**panda_gazebo_description**](./panda_gazebo_description) – URDF (Gazebo supported) and Mesh description
- [**panda_gazebo_moveit_config**](./panda_gazebo_moveit_config) – MoveIt 2 configuration

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Humble](https://docs.ros.org/en/humble/Installation.html)
- Gazebo [Fortress](https://gazebosim.org/docs/fortress)
- Gazebo ROS 2 Control [Humble](https://github.com/ros-controls/gz_ros2_control/tree/humble)
- Gazebo ROS Integration [Humble](https://github.com/gazebosim/ros_gz/tree/humble)

This package assume you have ROS 2 Humble, MoveIt 2 Humble, and Gazebo Fortress installed. Additional dependencies can be pulled via [vcstool](https://wiki.ros.org/vcstool) ([dependencies.repos](./dependencies.repos)). Please see below for details of building process.

### Building

Clone this repository, import dependencies, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository and put it in ROS2 workspace
git clone https://github.com/luhao2021/panda_gazebo_moveit2.git
# Import dependencies
cd panda_gazebo_moveit2
vcs import < dependencies.repos
# Apply the patch
cd gz_ros2_control
git am ../0001-Fix-gz_ros2_control-bug.patch
# Install dependencies
export IGNITION_VERSION=fortress
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Run the demo

Please follow below steps to run the demo

```bash
# Terminal 1
# configure the gazebo plugin path and resource path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(CURRENT_WORKING_DIRECTORY)/install/lib
export GZ_SIM_RESOURCE_PATH=$(CURRENT_WORKING_DIRECTORY)/install/share:$(CURRENT_WORKING_DIRECTORY)/install/share/panda_gazebo_demo/models
source $(MOVEIT2_DIRECTORY)/install/setup.bash
source install/local_setup.bash
ros2 launch panda_gazebo_demo move_group_gazebo.launch.py


# Terminal 2:
# this script setup the bridge between gazebo and moveit2 which sync up the scene and pose information
source /opt/ros/humble/setup.bash
cd scripts
./gazebo_updater.py --config config.yaml
```
