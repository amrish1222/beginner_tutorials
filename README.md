# ENPM808X ROS Beginner Tutorials
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/amrish1222/beginner_tutorials.git)

# Overview
 This ROS tutorial covers how to write a publisher and subscriber node in C++.
 
  Publisher node- Talker
  
  Subscriber node- Listener
  
  Basics concepts-
  - ROS master
  - nodes
  - topic
  - messages
  - package
  - catkin
  
   In this project the Talker node publishes to the topic "chatter" and the Listener node subscribes to the topic and prints
   it to the terminal
   
Publisher Terminal Output-
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/Talker.png)

Subscriber Terminal Output-
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/Listener.png)

# Dependencies
- ROS Kinetic

   Installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation)
   
- Catkin

   Information and installation instructions of Catkin can be found [here](http://wiki.ros.org/catkin)
   
   
# Build Instructions

**Catkin Workspace, cloning the repository and building**

Create a catkin workspace :
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Cloning the repository
```
git clone --recursive https://github.com/amrish1222/beginner_tutorials.git
```

Build
```
cd ..
catkin_make
```

Setting the environment variables
```
source devel/setup.bash
```

