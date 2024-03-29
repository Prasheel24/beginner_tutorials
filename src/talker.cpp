/****************************************************************************************
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
***************************************************************************************/

/**
 *  @copyright MIT License 2019 Prasheel Renkuntla
 *  @file    talker.cpp
 *  @author  Prasheel Renkuntla
 *  @date    11/11/2019
 *  @version 3.0
 *
 *  @brief Talker program with Service
 *
 *  @section DESCRIPTION
 *  
 *  Beginner tutorial for creating a ROS package to publish custom string message
 *  String message is modified with a service called by the listener node
 *  The frequency of the message can be passed through command line arguments.
 *  startAll launch file is used to start all nodes at once
 *  TF broadcaster is used to broadcast /talk and /world frame
 *  Rosbag recording is also used.
 */
#include <tf/transform_broadcaster.h>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "beginner_tutorials/changeOutput.h"

/**
 * @brief  This is a global stringstream type variable to be used by the service
 */
std::stringstream newString;

/**
 *   @brief Service callback function to change the output on screen
 *
 *   @param beginner_tutorials::changeOutput::incomingString req, Request to server
 *   @param beginner_tutorials::changeOutput::outputString resp, Response from server
 *
 *   @return bool successful response will return true.
 */
bool changeOutput(beginner_tutorials::changeOutput::Request& req,
                  beginner_tutorials::changeOutput::Response& res) {
  res.outputString = req.incomingString;

  //  Generate Debug level message for developers
  ROS_DEBUG_STREAM("The Output string will be: " << res.outputString);

  //  Take the incoming string into the variable.
  newString << res.outputString;
  return true;
}

/**
 *   @brief Main function to run the talker node
 *
 *   @param int argc, argument count for the main function
 *   @param char argv, argument values for the main function
 *
 *   @return int 0 if node runs successfully.
 */
int main(int argc, char **argv) {
  //  Variable to capture the frequency from command line
  double talkerParamFreq;

  //  The ros::init() function needs to see argc and argv so
  //  that it can perform any ROS arguments and name remapping
  //  that were provided at the command line.
  //  The third argument to init() is the name of the node.
  //  You must call one of the versions of ros::init() before using any other
  //  part of the ROS system.
  ros::init(argc, argv, "talker");

  //  NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle nh;


  //  Service Server object here calls the advertiseService function
  //  to check for service callbacks
  ros::ServiceServer service = nh.advertiseService("changeOutput",
                                                  changeOutput);


  //  The advertise(topic_name, queue size) returns a publisher object
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  //  check for variable input and assign to the variable above
  if (argc == 2) {
    talkerParamFreq = atoi(argv[1]);
    if (talkerParamFreq < 0) {
      // Fatal when there is a frequency less than 0
      ROS_FATAL_STREAM("Talker frequency must be greater than zero");
      exit(1);
    }
    if (talkerParamFreq == 0) {
      // Error when there is a frequency equal to 0. Can be recovered.
      ROS_ERROR_STREAM("Talker frequency should not be zero. Try Again!");
      exit(1);
    }
  }

  //  INFO level message on the screen about program start
  ROS_WARN_STREAM("Publishing messages at rate: " << talkerParamFreq);

  ros::Rate loop_rate(talkerParamFreq);

  //  Initailise the TF broadcaster
  static tf::TransformBroadcaster br;
  //  Create the transform object
  tf::Transform transform;
  //  set origin
  transform.setOrigin(tf::Vector3(0.0, 3.0, 9.0));
  //  initialise a Quaternion
  tf::Quaternion quat;
  quat.setRPY(0.5, 0.9, 1.90);
  //  set initial rotation
  transform.setRotation(quat);

  //  A count used to create a unique string for each message.
  int count = 0;

  //  Control given to ROS, runs the node till node has been shutdown.
  while (ros::ok()) {
    //  This is a message object. You stuff it with data, and then publish it.
    std_msgs::String msg;

    //  Stringstream variables to work with response from service.
    std::stringstream ss;
    std::stringstream tempString;

    ss << "Inside Talker" << " " << count;

    //  logic to check and display new string from service response.
    msg.data = newString.str();
    if (msg.data == "") {
      msg.data = ss.str();
      ROS_WARN_STREAM_ONCE("Service has not been called yet.");
    } else {
      tempString << newString.str() << " " << count;
      msg.data = tempString.str();
      ROS_WARN_STREAM_ONCE("Connection estabished with service.");
    }
    ROS_DEBUG_STREAM("Messages from talker with service called");
    ROS_INFO_STREAM("" << msg.data.c_str());

    //   The publish() function with parameter as the same message object.
    chatter_pub.publish(msg);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "world", "talk"));
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
