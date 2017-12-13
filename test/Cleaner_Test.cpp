/**
 * @file Cleaner_Test.cpp
 * @brief This file contains all tests for Cleaner Class.
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
#include "../include/Cleaner.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "../include/Goals.h"


/**
 * @brief      Tests whether client comes online
 *             when map server is online.        
 *
 * @param[in]  TESTSuite                 gtest framework
 * @param[in]  Server_Existance_Test     Test Name
 */

TEST(TESTSuite, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                                          ac("move_base", true);
  // Check the Existence of Service
  bool exists(ac.waitForServer(ros::Duration(10)));
  EXPECT_FALSE(exists);
}

/**
 * @brief      Tests whether the Goal X Point is sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  X_Test         Test Name
 */
TEST(TESTSuite, X_Test) {
std::vector<std::vector<double>> Goal = {{0, 0, 0}};
ros::NodeHandle n;
Cleaner Bot(Goal);
Goals Clean;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(Clean.Set_Current_X(Point, goal) , 0);
}

/**
 * @brief      Tests whether the Goal Y Point is Sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Y_Test         Test Name
 */

TEST(TESTSuite, Y_Test) {
std::vector<std::vector<double>> Goal = {{0, 1, 0}};
ros::NodeHandle n;
Cleaner Bot(Goal);
Goals Clean;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(Clean.Set_Current_Y(Point, goal) , 1);
}
