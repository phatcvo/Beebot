/**
 * @file base_controller.cpp
 * @author phatcvo
 * @brief base_controller for mobile robot
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <string>
#include <utility>

#include "base_controller/base_controller.h"

BaseController::BaseController(void) : private_nh_("~"), mode_(std::make_pair("stop", "manual"))
{
  private_nh_.param<int>("hz", param_.hz, 20);
  private_nh_.param<double>("max_linear_vel", param_.max_linear_vel, 0.25);
  private_nh_.param<double>("max_angular_vel", param_.max_angular_vel, 1.0);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = nh_.subscribe("/joy", 1, &BaseController::joy_callback, this);
  planner_cmd_vel_sub_ = nh_.subscribe("/planner/cmd_vel", 1, &BaseController::planner_cmd_vel_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("hz: " << param_.hz);
  ROS_INFO_STREAM("max_linear_vel: " << param_.max_linear_vel);
  ROS_INFO_STREAM("max_angular_vel: " << param_.max_angular_vel);
}

void BaseController::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
  mode_ = select_mode(msg, mode_);
  if (mode_.first == "move" && mode_.second == "manual")
    cmd_vel_ = joy_to_cmd_vel(msg);
  else if (mode_.first == "stop")
    cmd_vel_ = geometry_msgs::Twist();
}

void BaseController::planner_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  if (mode_.first == "move" && mode_.second == "auto")
  {
    cmd_vel_ = *msg;
    cmd_vel_.linear.x = std::min(cmd_vel_.linear.x, param_.max_linear_vel);
    if (cmd_vel_.angular.z > 0)
      cmd_vel_.angular.z = std::min(cmd_vel_.angular.z, param_.max_angular_vel);
    else
      cmd_vel_.angular.z = std::max(cmd_vel_.angular.z, -param_.max_angular_vel);
  }
}

void BaseController::process(void)
{
  ros::Rate loop_rate(param_.hz);
  while (ros::ok())
  {
    ros::spinOnce();
    cmd_vel_pub_.publish(cmd_vel_);
    print_status(mode_, cmd_vel_);
    loop_rate.sleep();
  }
}

std::pair<std::string, std::string>
BaseController::select_mode(const sensor_msgs::Joy::ConstPtr &joy, const std::pair<std::string, std::string> &prev_mode)
{
  std::string first_mode = prev_mode.first;
  std::string second_mode = prev_mode.second;

  if (joy->buttons[0] == 1)
    first_mode = "stop";
  if (joy->buttons[1] == 1)
    first_mode = "move";
  if (joy->buttons[2] == 1)
    second_mode = "auto";
  if (joy->buttons[3] == 1)
    second_mode = "manual";

  return std::make_pair(first_mode, second_mode);
}

geometry_msgs::Twist BaseController::joy_to_cmd_vel(const sensor_msgs::Joy::ConstPtr &joy)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = joy->axes[1] * param_.max_linear_vel;
  cmd_vel.angular.z = joy->axes[0] * param_.max_angular_vel;

  cmd_vel.linear.x = std::abs(cmd_vel.linear.x) < FLT_EPSILON ? 0.0 : cmd_vel.linear.x;
  cmd_vel.angular.z = std::abs(cmd_vel.angular.z) < FLT_EPSILON ? 0.0 : cmd_vel.angular.z;

  return cmd_vel;
}

void BaseController::print_status(const std::pair<std::string, std::string> &mode, const geometry_msgs::Twist &cmd_vel)
{
  ROS_INFO_STREAM("[" << mode.second << " - " << mode.first << "]");
  ROS_INFO_STREAM("linear.x: " << cmd_vel.linear.x);
  ROS_INFO_STREAM("angular.z: " << cmd_vel.angular.z);
  ROS_INFO_STREAM("----------");
  ROS_INFO_STREAM("");
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "base_controller");
  BaseController base_controller;
  base_controller.process();

  return 0;
}
