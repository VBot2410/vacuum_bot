/**
 * @file vacuum_bot.h
 * @brief This file contains function declarations for vacuum_bot class. 
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
#ifndef CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_VACUUM_BOT_H_
#define CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_VACUUM_BOT_H_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <vector>


class vacuum_bot{
 public:
vacuum_bot(ros::NodeHandle& n, std::vector<std::vector<double>>& Goals);
 private:
void Set_Current_X(std::vector<double>& Next_Goal,
                        move_base_msgs::MoveBaseGoal & goal);
void Set_Current_Y(std::vector<double>& Next_Goal,
                        move_base_msgs::MoveBaseGoal & goal);
void Set_Current_Orientation(std::vector<double>& Next_Goal,
                        move_base_msgs::MoveBaseGoal & goal);
};
#endif  // CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_VACUUM_BOT_H_
