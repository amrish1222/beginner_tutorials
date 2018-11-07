# ENPM808X ROS Beginner Tutorials
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/amrish1222/beginner_tutorials.git)

# Overview
 This ROS tutorial covers how to 
 - Write a [publisher and subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) node in C++.
 
  Publisher node- Talker
  
  Subscriber node- Listener
  
 - Add a service to the talker node to change the base output string.
 
 - Use all five logging levels in the two nodes to output useful information.
 
 - Creating a launch file that modifies the talker node by changing the publish frequency.
 
  Basics concepts-
  - ROS master
  - nodes
  - topic
  - messages
  - packages
  - catkin
  - rqt_graph
  - launch file
  - services
  - launch arguments
  
   In this project the Talker node publishes to the topic "chatter" and the Listener node subscribes to the topic and prints
   it to the terminal.
   
   Also using the launch file both the talker and listener nodes are launched in two different windows. Using a service we change the base string published by the talker. A command line argument can be used to change the publish frequency of the talker node.
   
Publisher Terminal Output-
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/Talker.png)

Subscriber Terminal Output-
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/Listener.png)

ROS computation graph
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/master/images/rqt_graph.png)

Talker and Listener displayed using launch file
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/Week10_HW/images/TalkerAndListenerLaunchFile.png)

Output after using the service toggle_message to change the base string to "Winner Winner"
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/Week10_HW/images/ChangeStringWithService.png)

ROS computational graph when using the launch file- beginner_tutorials.launch
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/Week10_HW/images/NewRqt_Graph.png)

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
# Changing Branch to Week10_HW

Navigate to the beginner_tutorial folder and change the branch
```
cd ~/catkin_ws/src/beginner_tutorial
git checkout Week10_HW
```


# 1. Running Instructions (using rosrun)

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

# 2. Running Instructions (using launch file)

- Running with default Talker publish rate
```
roslaunch beginner_tutorials beginner_tutorial.launch
```

- Running with Talker publish rate = 20
```
roslaunch beginner_tutorials beginner_tutorial.launch talker_rate:=20
```

# Termination Instructions

In a terminal at a time with ROS master being the last press **Ctrl+C** 

# ROS Computation graph

To view the ROS computation graph, run the following command in a terminal while the ROS master, publisher and subscriber nodes are running
```
rqt_graph
```
