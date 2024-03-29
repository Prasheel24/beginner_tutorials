/*****************************************************************************************
 Copyright (C) 2019 Prasheel Renkuntla

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
******************************************************************************************/

/**
 *  @copyright MIT License 2019 Prasheel Renkuntla
 *  @file    test/main.cpp
 *  @author  Prasheel Renkuntla
 *  @date    11/11/2019
 *  @version 1.0
 *
 *  @brief  Main cpp file to run all tests
 *
 *  @section DESCRIPTION
 *  
 *  Tests all the test cases in the testTalker.cpp file.
 *  Tests include - Service Presence, Service Output check, TF Broadcast check
 *  testTalkerRun.launch launch file is used to start all nodes at once
 */
#include <ros/ros.h>
#include <gtest/gtest.h>

/**
 *   @brief Main function to run all tests
 *
 *   @param int argc, argument count for the main function
 *   @param char argv, argument values for the main function
 *
 *   @return int 0 if node runs successfully.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "testTalker");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
