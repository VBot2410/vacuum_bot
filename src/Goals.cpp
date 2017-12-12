/**
 * @file Goals.cpp
 * @brief This file contains function definitions for Goals class.
 *        This class is dedicated to assigning x,y,angle values for
 *        move_base goals. 
 * 
 *
 * @author Vaibhav Bhilare
 * @copyright 2017, Vaibhav Bhilare
 *
 * MIT License
 * Copyright (c) 2017 Vaibhav Bhilare
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* --Includes-- */
#include <vector>
#include "../include/Goals.h"
#include "../include/Cleaner.h"

Goals::Goals() {
}
double Goals::Set_Current_X(std::vector<double>& Next_Goal,
                          move_base_msgs::MoveBaseGoal & goal) {
goal.target_pose.pose.position.x = Next_Goal.at(0);  // Set Goal X Coordinate
ROS_DEBUG("Goal X Point Successfully Set");
Get_Goal = Next_Goal;
return Next_Goal.at(0);  // Return goal point's X coordinate.
}

double Goals::Set_Current_Y(std::vector<double>& Next_Goal,
                          move_base_msgs::MoveBaseGoal & goal) {
goal.target_pose.pose.position.y = Next_Goal.at(1);  // Set Goal Y Coordinate
ROS_DEBUG("Goal Y Point Successfully Set");
return Next_Goal.at(1);  // Return goal point's Y coordinate.
}

void Goals::Set_Current_Orientation(std::vector<double>& Next_Goal,
                                   move_base_msgs::MoveBaseGoal & goal) {
auto Angle_Degrees = Next_Goal.at(2);  // Extract Orientation Value
auto Radians = Angle_Degrees*(3.14159/180);  // Convert Degrees to Radians
auto quaternion = tf::createQuaternionFromYaw(Radians);  // Create Quaternion
geometry_msgs::Quaternion qMsg;
tf::quaternionTFToMsg(quaternion, qMsg);  // Quaternion to Quaternion msg
goal.target_pose.pose.orientation = qMsg;  // Set Goal Orientation
ROS_DEBUG("Goal Orientation Successfully Set");
}

Goals::~Goals() {
}
