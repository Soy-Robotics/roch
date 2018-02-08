/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * Copyright (c) 2017, Soy Robotics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class RochXboxTeleop
{
public:
  RochXboxTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_;
  int forward_, left_, right_, back_, increase_, reduce_;
  double l_scale_, a_scale_, increase_scale_, reduce_scale_;
  double linear_speed_, angular_speed_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

RochXboxTeleop::RochXboxTeleop():
  ph_("~"),
  linear_(1),
  angular_(0),
  forward_(3),
  back_(0),
  left_(2),
  right_(1),
  increase_(13),
  reduce_(14),
  increase_scale_(1.1),
  reduce_scale_(.9),
  deadman_axis_(4),
  l_scale_(0.3),
  a_scale_(0.9),
  linear_speed_(0.5),
  angular_speed_(2.5)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_forward", forward_, forward_);
  ph_.param("axis_back", back_, back_);
  ph_.param("axis_left", left_, left_);
  ph_.param("axis_right", right_, right_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("linear_speed", linear_speed_, linear_speed_);
  ph_.param("angular_speed", angular_speed_, angular_speed_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RochXboxTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&RochXboxTeleop::publish, this));
}

void RochXboxTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  if( (a_scale_*joy->axes[angular_] != 430) || (l_scale_*joy->axes[linear_] != 255) ){
    vel.angular.z = a_scale_*joy->axes[angular_];
    vel.linear.x = l_scale_*joy->axes[linear_];
  }
  if( joy->buttons[increase_]){
    linear_speed_ *= increase_scale_;
    angular_speed_ *= increase_scale_;
  }
  if( joy->buttons[reduce_]){
    linear_speed_ *= reduce_scale_;
    angular_speed_ *= reduce_scale_;
  }
  if( joy->buttons[forward_]){
    vel.angular.z = 0;
    vel.linear.x = linear_speed_;
  }
  if( joy->buttons[back_]){
    vel.angular.z = 0;
    vel.linear.x = -linear_speed_;
  }
  if( joy->buttons[left_]){
    vel.angular.z = angular_speed_;
    vel.linear.x = 0;
  }
  if( joy->buttons[right_]){
    vel.angular.z = -angular_speed_;
    vel.linear.x = 0;
  }
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  ROS_INFO_STREAM("current linear speed:"<<linear_speed_<<"\t angular speed:"<<angular_speed_);
}

void RochXboxTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roch_teleop");
  RochXboxTeleop husky_xbox_teleop;

  ros::spin();
}
