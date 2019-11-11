/*
 * Copyright (C) 2019 Prasheel Renkuntla
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 *  @copyright MIT License 2019 Prasheel Renkuntla
 *  @file    talker.cpp
 *  @author  Prasheel Renkuntla
 *  @date    11/04/2019
 *  @version 1.0
 *
 *  @brief Talker program with Service
 *
 *  @section DESCRIPTION
 *  
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/changeOutput.h"

/**
 * @brief Testing whether the ROS service provided by talker node is present.
 */
TEST(TestTalkerNode, testServicePresence) {
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::changeOutput>("changeOutput");

  EXPECT_TRUE(client.waitForExistence(ros::Duration(5.0)));
}

/**
 * @brief This test verifies whether the service by talker node, 
 * modifies the default text.
 */
TEST(TestTalkerNode, testOutputOfService) {
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::changeOutput>("changeOutput");
  beginner_tutorials::changeOutput::Request req;
  beginner_tutorials::changeOutput::Response res;

  req.incomingString = "This Test case to call service works!";
  std::string expectedString = req.incomingString;
  client.call(req, res);
  EXPECT_EQ(expectedString, res.outputString);
}

TEST(TestTalkerNode, testTFBroadcast) {
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  while (nh.ok()) {
    try {   
      listener.lookupTransform("/talk", "/world", ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException &exc) {
      ROS_ERROR_STREAM(exc.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  double xCoordinate = 5.5;
  double yCoordinate = -2.5;
  double zCoordinate = -7.5;
  double absErrorRange = 0.5;
  EXPECT_NEAR(xCoordinate, transform.getOrigin().x(), absErrorRange);
  EXPECT_NEAR(yCoordinate, transform.getOrigin().y(), absErrorRange);
  EXPECT_NEAR(zCoordinate, transform.getOrigin().z(), absErrorRange);
}
