# Welcome to the Delta package!
This is a ROS package for the DRV90L/DRV70E robotic arm of Delta Electronics. This package contains:
* ROS package for controlling and visualizing the robot.
* Code for autonomously and remote controlling the robot without using the DRAStudio software.
* Code for using the Robotiq FT300 force torque sensor using ROS.
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
After git cloning the package a catkin_make is required in the ~/catkin_ws folder.
```
catkin_make
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

To run the code for the force torque sensor run the following command
```
rosrun ft300_sensor ft300.py
```
## Connecting to the real robot
To connect to the real robot make sure that your machine is on the same network as the robot. This can be with a ethernet cable, or wireless when the robot is connected to a router. To connect to the robot you'll need to setup a static ip adress, you can use anything from 192.168.1.2 to 192.168.1.99 depending on the ip addres of the other devices on the network. To chech which ip adresses are already used run the following command (the ip addres of the robot is 192.168.1.1
```
nmap -sP 192.168.1.*
```
Make sure to choose a ip addres that's not yet listed to a different device.

**Sometimes the connection to the robot won't work. This can be solved by disabling all other connections (turn of wifi).**

To test the connection to the robot run the following command
```
ping 192.168.1.1
```

## Wiki
To learn more about how the Delta DRV90L package works check out the [wiki](https://github.com/RemcoKuijpers/delta/wiki) (Work in progress)

## Report
To see the robot arm's part of the project's end report see the following [pdf file](https://github.com/RemcoKuijpers/delta/blob/master/docs/RobotArm_EndReport.pdf).

## Authors

* **Remco Kuijpers** - [RoboHub Eindhoven](https://github.com/RoboHubEindhoven)
