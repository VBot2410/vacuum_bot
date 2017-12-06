/**
 * @file main.cpp
 * @brief A program that drives turtlebot vacuum in a known map by
 *        following a set of goals.
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
#include "../include/vacuum_bot.h"

/**
 * @brief      main  Execution starts here
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     Returns 0 upon successful execution
 */
int main(int argc, char **argv) {
std::vector<std::vector<double>> Goals{{4.5, 0.1, 90}, {4.6, 0.5, 180},
  {0, 0.5, 90}, {0, 1, 0}, {3.4, 1, 90}, {0, 1.2, 0}, {3.5, 2, 0},
  {0.9, 2, 90}, {0.9, 2.5, 0}, {4.6, 2.7, 90}, {4.6, 3, 180}, {0.9, 3, 90},
  {0.9, 3.5, 0}, {4.6, 3.5, 90}, {4.6, 4, 180}, {0, 3.8, 90}, {0, 4.5, 0},
  {4.6, 4.5, 90}, {0, 0, -90}};
  ros::init(argc, argv, "Clean");
  ros::NodeHandle n;
  vacuum_bot Bot(n, Goals);
  Bot.Clean_Room();
  return 0;
}
