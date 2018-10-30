/** @file listener.cpp
 * @brief This demonstrates simple receipt of messages over the ROS system.
 * @author Amrish Baskaran
 * Copyright 2018 Amrish Baskaran
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief Callback function
 * @param Constant pointer of String
 * @return none.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @brief Main function
 * @param Number of arguments
 * @param Pointer to arguments
 * @return status.
 */
int main(int argc, char **argv) {
  // Initializing ROS system
  ros::init(argc, argv, "listener");
  // Creating NodeHanle
  ros::NodeHandle n;
  // Subscribe to the chatter topic
  // call chatterCallback when message arrives
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // Pumping Callback
  ros::spin();
  return 0;
}
