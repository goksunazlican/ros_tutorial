/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt32.h>
#include <sstream>
uint32_t t_old=0;
uint32_t result;

class SubscribeAndPublish{
  public:
    SubscribeAndPublish(){
     result_pub = n_.advertise<std_msgs::String>("results",10);
     process_sub = n_.subscribe("values", 10, &SubscribeAndPublish::callback, this);
    }
 void callback(const std_msgs::UInt32 msg){

  result = (msg.data*msg.data)+t_old;
  t_old = result;

  std_msgs::String pub_msg;
  pub_msg.data = std::to_string(result);

  result_pub.publish(pub_msg);

  ROS_INFO("Published result is %s",pub_msg.data.c_str());
  ROS_INFO("Subscriber: I heard: [%i]  result is: %i and t_old is: %i", msg, result, t_old);
 }
private:
 ros::NodeHandle n_;
 ros::Publisher result_pub;
 ros::Subscriber process_sub;
};


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");
  SubscribeAndPublish SAPObject;
  ros::spin();


  return 0;
}
