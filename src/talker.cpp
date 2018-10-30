/** @file talker.cpp
 * @brief This demonstrates simple sending of messages over the ROS system.
 * @author Amrish Baskaran
 * Copyright 2018 Amrish Baskaran
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief Main function
 * @param Number of arguments
 * @param Pointer to arguments
 * @return status.
 */
int main(int argc, char **argv) {
  // Initializing ROS system
  ros::init(argc, argv, "talker");
  // Creating NodeHanle
  ros::NodeHandle n;
  // Advertise publishing to topic chatter
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Setting loop rate
  ros::Rate loop_rate(10);
  // Message count
  int count = 0;
  while (ros::ok()) {
    // Message object
    std_msgs::String msg;
    // Storing data to stringstream
    std::stringstream ss;
    ss << "Go Terps! " << count;
    // Writing to msg
    msg.data = ss.str();
    // Sending the msg to ROS
    ROS_INFO("%s", msg.data.c_str());
    // Publishing msg to the topic
    chatter_pub.publish(msg);
    // Check for callback
    ros::spinOnce();
    // delay
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
