# vacuum_bot
Implementation of a Cleaning Robot on TurtleBot </br ></br >
[![Build Status](https://travis-ci.org/VBot2410/vacuum_bot.svg?branch=master)](https://travis-ci.org/VBot2410/vacuum_bot)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
This package uses Gazebo simulation of Turtlebot to implement a vacuum cleaning robot. It utilizes ROS navigation stack to drive the robot autonomously in a known map. The pattern it follows is given by a set of goals. </br >
This system is capable of following a predefined pattern to clean the room and avoid obstacles in its surroundings while completing its task. At the end of its task, it will return to its Home Location and be ready for next use.

## Description
### Navigation Stack
Navigation stack takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.</br >
For more information on Navigation Stack, see [navigation](http://wiki.ros.org/navigation)
### Node
This package contains a node named Clean which utilizes an implementation of ROS actionlib package. actionlib provides simple action specifications like goal, feedback, result in form of ROS actions.</br >
For more information on actionlib, see [actionlib](http://wiki.ros.org/actionlib)
### move_base
The move_base package provides an implementation of an [action](http://wiki.ros.org/actionlib) that, given a goal in the world, will attempt to reach it with a mobile base. We have used move_base_msgs for this implementation. move_base_msgs contains the messages used to communicate with move_base node.</br >
For more information on move_base, see [move_base](http://wiki.ros.org/move_base)
### Mapping
The world was built in Gazebo and ROS gmapping package was used for creating the map. For more information, See [Map Building](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM)

## Personnel
This package is developed and maintained by Robotics Grduate Student **Vaibhav Bhilare** as a part of Fall 2017 term Final Project for course ENPM 808X, Software Development for Robotics at University of Maryland, College Park.

## Disclaimer
MIT License

Copyright (c) 2017 Vaibhav Bhilare

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Dependencies
### ROS
ROS should be installed on the system. This package is tested on Ubuntu 16.04 LTS with [ROS Kinetic Desktop-Full Distribution](http://wiki.ros.org/kinetic).<br />
Installation Instructions can be found [here](http://wiki.ros.org/kinetic/Installation).
### catkin
catkin is a Low-level build system macros and infrastructure for ROS.<br />
catkin is included by default when ROS is installed. It can also be installed with apt-get
```
$ sudo apt-get install ros-kinetic-catkin
```
### Gazebo
Gazebo was used to build the world and is required to visualize the demo. To launch the demo, Gazebo should be installed. <br />
Installation instructions can be found [here](http://gazebosim.org/download)
### rviz
rviz is used for displaying sensor data and state information from ROS. rviz is included by default when ROS Kinetic Desktop-Full is installed.
### turtlebot simulation
To install Turtlebot simulation stack use following command:
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
### Package Dependency
- roscpp
- rospy
- actionlib
- actionlib_msgs
- geometry_msgs
- move_base_msgs
- std_msgs
- tf
- rostest
- turtlebot_gazebo

## Solo Iterative Process (SIP)
This package was developed using Solo Iterative Process.<br />
SIP Google sheet can be found [here](https://docs.google.com/spreadsheets/d/1xJOBrPESNhSnJeYWHMUa0bzHvRnpLpRPEiKgT72gR60/edit?usp=sharing)
<br />
Sprint Planning and Review notes can be found [here](https://docs.google.com/document/d/1MplUpR0tAjvwMJLPFIJioawKSmGYAsTr-fAWhTBEiD0/edit?usp=sharing)

## Known issues/bugs
~~Scaled Objects change shape when reloading a model in Gazebo.~~ <br />
~~Robot keeps rotating at one location rather than moving to the goal.~~ <br />
None

## Build Instructions
### Creating a catkin workspace
Create a catkin workspace using following instructions:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Running catkin_make command the first time in your workspace will create a CMakeLists.txt link in your 'src' folder. Before continuing source your new setup.*sh file:
```
$ source devel/setup.bash
```
### Building the Package inside catkin workspace
Clone the package in src folder of catkin workspace using following commands:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/VBot2410/vacuum_bot.git
```
Then build the package using following commands:
```
$ cd ~/catkin_ws/
$ catkin_make
```

## Running The Demo
Open a terminal and run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch vacuum_bot vacuum_bot.launch
```
### Rebuilding the map using gmapping
Though **not a part of final deliverable**, We have also added the script developed for gmapping using turtlebot navigation package.<br />
Run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch vacuum_bot vacuum_bot_gmapping.launch
```
After running above commands, open a new terminal and run following command:
```
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```
Drive the robot around to cover the full map. For more information, see [turtlebot_navigation tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM) <br />
After having a good map, open a new terminal and save the map to file using following command:
```
$ rosrun map_server map_saver -f /home/<username>/catkin_ws/src/vacuum_bot/map/hotel_room_map
```
**Note**: Do not close the gmapping launch until saving the map.

## Recording Bag files and how to Enable/Disable Recording:
Running the demo launch command sets the record argument in launch file to *false* by default. To enable rebag recording, simply add **record:=true** to the launch command.</br >
The new commands will be:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch vacuum_bot vacuum_bot.launch record:=true
```
This will generate a file named rosbag_recording.bag in results subdirectory.</br >
To disable the recording, use the default argument or specify **record:=false**.</br >
### Inspecting the bag file
For inspecting the recorded rosbag file, run following commands:
```
$ cd ~/catkin_ws/src/vacuum_bot/results
$ rosbag info rosbag_recording.bag
```
### Playing Back the bag file
To play the recorded bag file, use following instructions:
In a terminal, type following command:
```
$ roscore
```
Open a new terminal and run following commands:
```
$ cd ~/catkin_ws/src/vacuum_bot/results
$ rosbag play rosbag_recording.bag
```
Open a new terminal and run following command:
```
$ rqt_console
```
This will open a rqt_console which will play all the messages recorded in the bag file while recording.

## Testing
Tests for this package are written using rostest and gtest. To build the tests, run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make run_tests_vacuum_bot
```
To run the tests after building them in previous step, use following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rostest vacuum_bot vacuum_bot_test.launch
```
Output shown will be similar to the following:
```
... logging to /home/viki/.ros/log/rostest-ubuntu-3056.log
[ROSUNIT] Outputting test results to /home/viki/.ros/test_results/vacuum_bot/rostest-launch_vacuum_bot_test.xml
testvacuum_bot_test ... ok

[ROSTEST]-----------------------------------------------------------------------

[vacuum_bot.rosunit-vacuum_bot_test/Server_Existance_Test][passed]
[vacuum_bot.rosunit-vacuum_bot_test/X_Test][passed]
[vacuum_bot.rosunit-vacuum_bot_test/Y_Test][passed]
[vacuum_bot.rosunit-vacuum_bot_test/Goal_Test][passed]
[vacuum_bot.rosunit-vacuum_bot_test/Goal_X_Test][passed]
[vacuum_bot.rosunit-vacuum_bot_test/Goal_Y_Test][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 6
 * ERRORS: 0
 * FAILURES: 0
```
## Project Presentation And Demo
### Presentation Video with Gazebo demo can be found [here](https://youtu.be/L4NH78Ymsps)
### Gazebo Demo can be found [here](https://youtu.be/QoAIg1g_lWA)
### RViz Demo can be found [here](https://youtu.be/ap7zxn24Si8)
### PowerPoint Presentation file can be found [here](https://drive.google.com/file/d/1C4TkPUN_RbWLDyJ-lwqXit9o_GeN9osq/view?usp=sharing)
