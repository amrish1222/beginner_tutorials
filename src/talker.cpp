/** MIT License
 *Copyright (c) 2018 Amrish Baskaran
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *
 * @file talker.cpp
 * @brief This demonstrates simple sending of messages over the ROS system.
 * @author Amrish Baskaran
 * @Copyright MIT License 
 * @Copyright 2018 Amrish Baskaran
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
