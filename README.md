# Welcome to the Delta package!
This is a ROS package for the DRV90L robotic arm of Delta Electronics. This package contains:
* ROS robot model (URDF) for visualising robot in Rviz.
* Code for autonomously and remote controlling the robot without using the DRAStudio software.
* Code for using the Robotiq FT300 force torque sensor in combination with the robot.
![Alt Text](https://github.com/RemcoKuijpers/delta/blob/master/pics/Result.gif)

## Getting started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.
### Prerequisites
What things you need to install the software and how to install them.
* ROS Melodic
* pyModbusTCP
* pygame (only for remote control with PS4 controller)

Installing pyModbusTCP
```
pip install pyModbusTCP
```
Installing pygame
```
pip install pygame
```
### Installing
To install this package, navigate to your catkin_ws source folder ~/catkin_ws/src and run the following command:
```
git clone https://github.com/RemcoKuijpers/delta.git
```
## Running the tests
To test if the package is installed correctly, run the following commands
```
roslaunch arm_description real_with_EE.launch
```
This command will launch the Rviz visualisation of the real robot. In Rviz add the robot model and choose "base" as fixed frame. Now the robot model should be visable, and should be in the same position as the real robot.

With the following command the robot model gets displayed, without connecting to the real robot. It's also possible to add the gui:=true option. In this way the robot angles can be controlled with the following command the robot model is displayed which can be controlled. To see the displayed robot in Rviz run the following command
```
roslaunch arm_description display_with_EE.launch gui:=true
```
![Alt Text](https://github.com/RemcoKuijpers/delta/blob/master/pics/display_with_control_compressed.gif)

## Wiki
To learn more about how the Delta DRV90L package works check out the [wiki](https://github.com/RemcoKuijpers/delta/wiki) (Work in progress)
## Authors

* **Remco Kuijpers** - [RoboHub Eindhoven](https://github.com/RoboHubEindhoven)
