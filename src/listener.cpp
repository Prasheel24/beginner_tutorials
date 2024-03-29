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
 *  @file    listener.cpp
 *  @author  Prasheel Renkuntla
 *  @date    11/11/2019
 *  @version 3.0
 *
 *  @brief listener program with Service
 *
 *  @section DESCRIPTION
 *  
 *  Beginner tutorial for creating a ROS package to subscribe to a node
 *  String message is modified here and sent back to publisher
 *  Also, it works with Rosbag with the argument check in the code.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "beginner_tutorials/changeOutput.h"

/**
 *   @brief Simple reception of messages over the ROS system.
 *
 *   @param const std::msgs::String::ConstPtr& msg 
 *   @return none
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard [" << msg->data.c_str() << "]");
}

/**
 *   @brief Main function to run the listener node
 *
 *   @param int argc, argument count for the main function
 *   @param char argv, argument values for the main function
 *
 *   @return int 0 if node runs successfully.
 */
int main(int argc, char **argv) {
  //  The ros::init() function needs to see argc and argv so that it can perform
  //  any ROS arguments and name remapping that were provided at command line.
  //  The third argument to init() is the name of the node.
  //  You must call one of the versions of ros::init() before using any other
  //  part of the ROS system.
  ros::init(argc, argv, "listener");

  //  NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Service Client object calls serviceClient function to send service Response
  ros::ServiceClient strClient = nh.serviceClient
    <beginner_tutorials::changeOutput>("changeOutput");

  //  Variables types from service file
  beginner_tutorials::changeOutput::Request req;
  beginner_tutorials::changeOutput::Response resp;

  //  Input the incoming string to be sent as response
  req.incomingString = "From Listener";

  bool success = strClient.call(req, resp);

  //  Flag to check if recording is being played.
  bool recordedFilePlayed;
  if (argc == 2) {
    recordedFilePlayed = atoi(argv[2]);
    if (recordedFilePlayed == true) {
      //  call function to be checked for successful transmission of messages
      if (success) {
        ROS_INFO_STREAM_ONCE("\nOutput from talker: " << resp.outputString);
      } else {
        ROS_ERROR_STREAM("Service Call Failed");
        return -1;
      }
    }
  }


  //  The subscribe() invokes a call to the ROS
  //  master node, which keeps a registry of who is publishing and who
  //  is subscribing.  Messages are passed to a callback function, here
  //  called chatterCallback.  subscribe() returns a Subscriber object that you
  //  must hold on to until you want to unsubscribe.
  //  When all copies of the Subscriber object go out of scope,
  //  this callback will automatically be unsubscribed from this topic.
  //  The second parameter to the subscribe() function is the size of message
  //  queue. If messages are arriving faster than they are being processed, this
  //  is the no. of messages that will be buffered up before beginning to throw
  //  away the oldest ones.
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // ros::spin() will enter a loop, pumping callbacks.  With this version, all
  // callbacks will be called from within this thread (main one). ros::spin()
  // will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  ros::Rate loop_rate(10);

  //  Control given to ROS, runs the node till node has been shutdown.
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

