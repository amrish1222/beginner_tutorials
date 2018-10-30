# ENPM808X ROS Beginner Tutorials
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/amrish1222/beginner_tutorials.git)

# Overview
 This ROS tutorial covers how to write a [publisher and subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) node in C++.
 
  Publisher node- Talker
  
  Subscriber node- Listener
  
  Basics concepts-
  - ROS master
  - nodes
  - topic
  - messages
  - packages
  - catkin
  - rqt_graph
  
   In this project the Talker node publishes to the topic "chatter" and the Listener node subscribes to the topic and prints
   it to the terminal
   
Publisher Terminal Output-
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/Talker.png)

Subscriber Terminal Output-
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/Listener.png)

ROS computation graph
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/rqt_graph.png)

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

# Running Instructions

Start ROS master- open a terminal and run
```
roscore
```

Run the talker node- open another terminal and run
```
rosrun beginner_tutorials talker
```

Run the listener node- open another terminal and run
```
rosrun beginner_tutorials listener
```

Note- To avoid using multiple terminal, a terminal Multiplexer like [tmux](https://linoxide.com/how-tos/install-tmux-manage-multiple-linux-terminals/) can be used. A beginner tutorial to tmux can be found [here](https://hackernoon.com/a-gentle-introduction-to-tmux-8d784c404340)

# Termination Instructions

In a terminal at a time with ROS master being the last press **Ctrl+C** 

# ROS Computation graph

To view the ROS computation graph, run the following command in a terminal while the ROS master, publisher and subscriber nodes are running
```
rqt_graph
```
