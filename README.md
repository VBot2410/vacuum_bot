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
For more information on Navigation Stack, please see [navigation](http://wiki.ros.org/navigation)
### Node
This package contains a node named Clean which utilizes an implementation of ROS actionlib package. actionlib provides simple action specifications like goal, feedback, result in form of ROS actions.</br >
For more information on actionlib, please see [actionlib](http://wiki.ros.org/actionlib)
### move_base
The move_base package provides an implementation of an [action](http://wiki.ros.org/actionlib) that, given a goal in the world, will attempt to reach it with a mobile base. We have used move_base_msgs for this implementation. move_base_msgs contains the messages used to communicate with move_base node.</br >
For more information on move_base, please see [move_base](http://wiki.ros.org/move_base)
## Dependencies

## Build Instructions

## Running the Demo

## Running the Tests
