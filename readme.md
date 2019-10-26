[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/Prasheel24/beginner_tutorials/blob/master/License)

## Authors

* **Prasheel Renkuntla** - [GitHub](https://github.com/Prasheel24)

## Overview
Beginner Tutorials from the ROS Wiki Page that walks through the Publisher and Subscriber example.

## Description
A beginner tutorial that helps to create a custom ROS package that consists of a publisher and a listener. Once the Master is setup, a talker(talker.cpp) will publish topic with a message and a listener(listener.cpp) will subscribe the topic with that message. 
The message published will be "You Rock!!"
The message subscriber will be "I Heard : [You Rock!!]"


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

## Run the program
To run the code follow the steps below-

1. Open a terminal to setup the Master Node: 
```
cd ~/catkin_ws
source ./devel/setup.bash
roscore
```
Ensure if roscore is running in the terminal. For any issues check [ROS Troubleshoot](http://wiki.ros.org/ROS/Troubleshooting)

2. Open a new terminal to setup the Publisher Node: 
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```
Publisher node must transmit "You Rock!!"

3. Open a new terminal to setup the Subscriber Node:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

4. Upon successful execution of the commands, the subscriber node must show "I heard : [You Rock]" message

## Dependencies
1. std_msgs
2. roscpp
3. genmsg
4. rospy

## References
* http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
* http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber
