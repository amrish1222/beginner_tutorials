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
#include <string>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

// Include header file for the service type definition
#include "beginner_tutorials/toggle_string.h"

#include "std_msgs/String.h"

// setting default outputString
std::string outputString = "Go Terps!!";

/**
 * @brief Callback function to change base string
 * @param Request and response
 * @return none.
 */
bool toggleMessage(beginner_tutorials::toggle_string::Request &req,
                   beginner_tutorials::toggle_string::Response &resp) {
  // Changing the base string
  std::string line;
  if (!req.stringReq.empty()) {
    outputString = req.stringReq;
    ROS_DEBUG_STREAM("The base string has been changed");
    ROS_INFO_STREAM("The new  base string is: " << outputString);
    resp.stringResp = "The new  base string is change to: " + outputString;
    return true;
  } else {
    // No string received
    resp.stringResp = "Empty argument";
    ROS_FATAL_STREAM("Empty string received as argument");
    return false;
  }
}

/**
 * @brief Broadcasting the static tf frame
 * @brief translation 10,10,0 and yaw of 3.14 radians
 * @param none
 * @return none
 */
void broadcastTfStatic() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(10, 10, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 3.14);
  transform.setRotation(q);
  // Parent frame- world
  // Child frame- talk
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
}

/**
 * @brief Main function
 * @param Number of arguments
 * @param Pointer to arguments
 * @return status.
 */
int main(int argc, char **argv) {
  int loopRateInput = 10;
  if (argc > 1) {
    loopRateInput = atoi(argv[1]);
  }
  else {
    // setting to default loop rate
    loopRateInput = 10;
  }
  if (loopRateInput == 0) {
    ROS_ERROR_STREAM("Invalid Loop rate, use loop rate greater than 0");
    loopRateInput = 10;
    ROS_INFO_STREAM("Setting default loop rate = 10");
  }
  ROS_INFO_STREAM("Loop rate now set to " << loopRateInput);
  // Initializing ROS system
  ros::init(argc, argv, "talker");
  // Creating NodeHanle
  ros::NodeHandle n;
  // Register our service with the master.
  ros::ServiceServer server = n.advertiseService("toggle_message",
                                                 &toggleMessage);
  // Advertise publishing to topic chatter
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Setting loop rate
  ros::Rate loop_rate(loopRateInput);
  // Message count
  int count = 0;
  while (ros::ok()) {
    // Message object
    std_msgs::String msg;
    // Storing data to stringstream
    std::stringstream ss;
    ss << outputString << ": " << count;
    if (count >= 100) {
      ROS_WARN_STREAM_ONCE("Talker has run more than 100 iterations");
    }
    // Writing to msg
    msg.data = ss.str();
    // Sending the msg to ROS
    ROS_INFO("%s", msg.data.c_str());
    // Publishing msg to the topic
    chatter_pub.publish(msg);
    // Broadcast tf frame to talk
    broadcastTfStatic();
    // Check for callback
    ros::spinOnce();
    // delay
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
