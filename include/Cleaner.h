/**
 * @file Cleaner.h
 * @brief This file contains function declarations for Cleaner class. 
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
#ifndef CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_CLEANER_H_
#define CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_CLEANER_H_
#include "Goals.h"
#include <vector>

/**
 * @brief Cleaner class
 *        This class is a protected derived class of base class Goals.
 *        This class contains functions which initialize action client.
 *        This class also uses functions from Goals class to send goals
 *        sequentially to move_base.
 */
class Cleaner:protected Goals {
 public:  // Public access specifier
/**
 * @brief Constructor for Cleaner Class
 *        Takes Goal Points in _Goals and stores them in a vetor of vectors
 *        named Goal_Points.
 * @param _Goals vector of vectors of type double
 */
explicit Cleaner(const std::vector<std::vector<double>>& _Goals);
/**
 * @brief Clean_Room Initializes action client to communicate with move_base
 *                   This function sends goals to move_base.
 */
void Clean_Room();
/**
 * @brief Destructor for Cleaner Class
 */
virtual ~Cleaner();
 private:
/** Create move_base message to send goal*/
move_base_msgs::MoveBaseGoal goal;
};
#endif  // CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_CLEANER_H_
