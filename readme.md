[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/Prasheel24/beginner_tutorials/blob/master/License)

## Authors

**Prasheel Renkuntla** - [GitHub](https://github.com/Prasheel24)

## Overview
Beginner Tutorials from the ROS Wiki Page that walks through the Publisher and Subscriber example.

## Description
A beginner tutorial that helps to create a custom ROS package that consists of a publisher and a service. Once the Master is setup, a talker(talker.cpp) will publish topic with a message. When the service is called from another terminal, the talker publishes the message accordingly.
<p></p>The message published will be "Inside Talker" followed by count
<p></p>The message subscribed will be "From Service" followed by count

## Dependencies	
1. ROS Kinetic - [Installation](http://wiki.ros.org/kinetic/Installation)
2. Catkin(installed by default with ROS) - a low level build system macros and infrastructure for ROS.
3. ROS libraries - std_msgs, roscpp

## Build
Build using the following commands-

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
cd src/

git clone --recursive https://github.com/prasheel24/beginner_tutorials
cd ..
catkin_make
```
This will make the workspace and package ready for execution

## Run the program to start Service
To run the code follow the steps below-

1. Open a terminal to setup the Master Node: 
```
cd ~/catkin_ws
source ./devel/setup.bash
roscore
```
&nbsp;&nbsp;&nbsp;Ensure if roscore is running in the terminal. For any issues check [ROS Troubleshoot](http://wiki.ros.org/ROS/Troubleshooting)

2. Open a new terminal to setup the Publisher Node: 
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```
&nbsp;&nbsp;&nbsp;Publisher node must transmit like this 
</br>"Service has not been called yet."
</br>"Inside Talker" with count increasing consecutively.

3. Open a new terminal to call the Service:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosservice call /changeOutput "From Service"
```

4. Upon successful execution of the commands, the talker node must show the following message
</br>"Service has been called."
</br>"From Service" with count increasing from the earlier point.

## References
* http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
