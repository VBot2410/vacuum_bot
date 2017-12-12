/**
 * @file Goals_Test.cpp
 * @brief This file contains all tests for Goals Class.
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
#include "../include/Goals.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "../include/Cleaner.h"


/**
 * @brief      Tests whether the Goal Point is sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Goal_Test         Test Name
 */
TEST(TESTSuite, Goal_Test) {
std::vector<std::vector<double>> Goal = {{0, 0, 0}};
ros::NodeHandle n;
Cleaner Bot(Goal);
Goals Clean;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
Clean.Set_Current_X(Point, goal);
EXPECT_EQ(Clean.Get_Goal , Point);
}

/**
 * @brief      Tests whether the Goal X Point is sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Goal_X_Test         Test Name
 */
TEST(TESTSuite, Goal_X_Test) {
std::vector<std::vector<double>> Goal = {{0, 0, 0}};
ros::NodeHandle n;
Goals V;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(V.Set_Current_X(Point, goal) , 0);
}

/**
 * @brief      Tests whether the Goal Y Point is Sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Goal_Y_Test         Test Name
 */

TEST(TESTSuite, Goal_Y_Test) {
std::vector<std::vector<double>> Goal = {{0, 1, 0}};
ros::NodeHandle n;
Goals V;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(V.Set_Current_Y(Point, goal) , 1);
}
