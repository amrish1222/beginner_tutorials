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
 
 - Modify Talker node to broadcast a tf frame called /talk with parent /world. The transform has a non zero translation and rotation. 
 
 - Using gtest and rostest to create a Level 2 integration test, that tests the Talker node 
 
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
  - tf frames
  - gtest
  - rostest
  
   In this project the Talker node publishes to the topic "chatter" and the Listener node subscribes to the topic and prints
   it to the terminal.
   
   It also broadcasts a tf frame called /talk with parent /world.
   
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
 
 - Gtest
 
   Information on usage and installation of gtest can be found [here](http://wiki.ros.org/gtest)
    
 - Rostest
 
   Information on usage of rostest can be found [here](http://wiki.ros.org/rostest)  
   
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
# Changing Branch to Week11_HW

Navigate to the beginner_tutorial folder and change the branch
```
cd ~/catkin_ws/src/beginner_tutorial
git checkout Week10_HW
```

Build
```
cd ..
cd ..
catkin_make
```

Setting the environment variables
```
source devel/setup.bash
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

Talker node can also be started with different publish rates by using a command line argument
```
rosrun beginner_tutorials talker 20
```
The above command will set the publish rate to 20.

# 2. Running Instructions (using launch file)

- Running with default Talker publish rate
```
roslaunch beginner_tutorials beginner_tutorial.launch
```

- Running with Talker publish rate = 20 using command-line argument
```
roslaunch beginner_tutorials beginner_tutorial.launch talker_rate:=20
```

# Using the toggle_message service to change the base string of Talker node

 After starting the talker node using the instructions above, the base string can be changed by executing the following command. For example the following command-line can be executed to change the base string to "Winner Winner".
```
rosservice call /toggle_message "stringReq: 'Winner Winner'"
```
- Warning

If an empty string is sent using
```
rosservice call /toggle_message "stringReq: ''"
```
Then a Fatal error is encountered and logged.

# Inspecting tf frames using tf tools

- view_frames creates a diagram of the frames being broadcast by tf over ROS.

```
rosrun tf view_frames
```
- Here a tf listener is listening to the frames that are being broadcast over ROS and drawing a tree of how the frames are connected. To view the tree:

```
evince frames.pdf
```

An example of this pdf can be viewed in the results folder

- rqt_tf_tree is a runtime tool for visualizing the tree of frames being broadcast over ROS.

```
rosrun rqt_tf_tree rqt_tf_tree
```

- tf_echo reports the transform between any two frames broadcast over ROS.

```
rosrun tf tf_echo world talk
```

# Running ROS Tests

- Using Catkin_make

```
cd ~/catkin_ws
catkin_make run_tests beginner_tutorials
```

The output will look like this

![alt text](https://github.com/amrish1222/beginner_tutorials/blob/Week11_HW/images/Catkin_make%20run_tests.png)

- Using Launch file

```
cd ~/catkin_ws
rostest beginner_tutorials talkerTest.launch
```

# Using rosbag

- Record all topics

```
rosbag record -a
```

- To display the summary of the example bag file 

```
cd ~/catkin_ws/src/beginner_tutorials/results
beginner_tutorials/results
```

- To play the example bag file, first run roscore and in another terminal use the command:

```
rosbag play rosbag_beginner_tutorials.bag
```

- To view the rosbag output to the chatter topic start ros master and in one terminal

```
rosrun beginner_tutorials listener
```

Now running the example file with the instructions above will produce the following output:
![alt text](https://github.com/amrish1222/beginner_tutorials/blob/Week11_HW/images/rosbagPlay_listener.png)

# Termination Instructions

In a terminal at a time with ROS master being the last press **Ctrl+C** 

If launch file is used then pressing **Ctrl+C** in the terminal used for launching can terminate the program.

# ROS Computation graph

To view the ROS computation graph, run the following command in a terminal while the ROS master, publisher and subscriber nodes are running
```
rqt_graph
```
