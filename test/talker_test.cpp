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
 * @file talker_test.cpp
 * @brief contains tests for the talker node
 * @author Amrish Baskaran
 * @Copyright MIT License
 * @Copyright 2018 Amrish Baskaran
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/toggle_string.h"
#include "std_msgs/String.h"

/**
 * @brief Test for existence of service
 */
TEST(TestNodeTalker, existenceTest) {
  // Creating node handle
  ros::NodeHandle nh;
  // creating a service client
  ros::ServiceClient client = nh
      .serviceClient<beginner_tutorials::toggle_string>(
      "toggle_message");
  // checking whether the service exists
  bool exists(client.waitForExistence(ros::Duration(5)));
  // Testing
  ASSERT_TRUE(exists);
}

/**
 * @brief Test the service by changing the request
 */
TEST(TestNodeTalker, toggleMsgTest) {
  // Creating node handle
  ros::NodeHandle nh;
  // creating a service client
  ros::ServiceClient client = nh
      .serviceClient<beginner_tutorials::toggle_string>("toggle_message");
  // service object
  beginner_tutorials::toggle_string srv;
  // initiating request variable
  srv.request.stringReq = "abcd";
  // Testing if initiated correctly
  EXPECT_STREQ("abcd", srv.request.stringReq.c_str());
}
