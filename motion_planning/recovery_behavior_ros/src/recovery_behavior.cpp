/**
 * @file recovery_behavior.cpp
 * @author beebot
 * @brief Recovery behavior class
 * @copyright Copyright (c) 2024
 */

#include <limits>
#include <utility>

#include "recovery_behavior/recovery_behavior.h"

RecoveryBehavior::RecoveryBehavior(void) : private_nh_("~")
{
  private_nh_.param<int>("hz", param_.hz, 10);
  private_nh_.param<int>("sim_time_samples", param_.sim_time_samples, 10);
  private_nh_.param<int>("vel_samples", param_.vel_samples, 3);
  private_nh_.param<float>("move_time", param_.move_time, 2.0);
  private_nh_.param<float>("min_vel_x", param_.min_vel_x, -0.2);
  private_nh_.param<float>("max_vel_theta", param_.max_vel_theta, 0.4);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/recovery/cmd_vel", 1);
  footprint_pub_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
  cloud_sub_ = nh_.subscribe("/cloud", 1, &RecoveryBehavior::cloud_callback, this, ros::TransportHints().tcpNoDelay());
  trigger_service_ = private_nh_.advertiseService("trigger", &RecoveryBehavior::trigger_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("Recovery behavior parameters");
  ROS_INFO_STREAM("  hz: " << param_.hz);
  ROS_INFO_STREAM("  sim_time_samples: " << param_.sim_time_samples);
  ROS_INFO_STREAM("  vel_samples: " << param_.vel_samples);
  ROS_INFO_STREAM("  move_time: " << param_.move_time);
  ROS_INFO_STREAM("  min_vel_x: " << param_.min_vel_x);
  ROS_INFO_STREAM("  max_vel_theta: " << param_.max_vel_theta);
  Footprint().print_params();
}

void RecoveryBehavior::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, pcl_cloud_);
}

bool RecoveryBehavior::trigger_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (pcl_cloud_.empty())
  {
    res.success = false;
    res.message = "Cloud is empty. Failed to start recovery behavior..";
  }
  else
  {
    recovery_move();
    res.success = true;
    res.message = "Recovery behavior has finished..";
  }
  return true;
}

void RecoveryBehavior::recovery_move(void)
{
  ros::Rate loop_rate(param_.hz);
  const ros::Time start_time = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start_time).toSec() < param_.move_time)
  {
    cmd_vel_pub_.publish(select_best_cmd_vel());
    footprint_pub_.publish(Footprint().get_footprint());
    ros::spinOnce();
    loop_rate.sleep();
  }

  cmd_vel_pub_.publish(geometry_msgs::Twist());
}

geometry_msgs::Twist RecoveryBehavior::select_best_cmd_vel(void)
{
  const float vel_x_resolution = param_.min_vel_x / (param_.vel_samples - 1);
  float rot_vel_resolution = (param_.max_vel_theta) / (param_.vel_samples - 1);
  const PointT first_nearest_point = find_nearest_point();
  if (0.0 < atan2(first_nearest_point.y, first_nearest_point.x))
    rot_vel_resolution *= -1.0;

  geometry_msgs::Twist cmd_vel;
  float max_distance = 0.0;

  for (int i = 1; i < param_.vel_samples; i++)
  {
    for (int j = 1; j < param_.vel_samples; j++)
    {
      geometry_msgs::Twist cmd_vel_tmp;
      cmd_vel_tmp.linear.x = i * vel_x_resolution;
      cmd_vel_tmp.angular.z = j * rot_vel_resolution;
      const std::pair<bool, float> result = simulate_with_check_collision_and_calc_dist_to_nearest_point(cmd_vel_tmp);
      if (result.first && max_distance < result.second)
      {
        max_distance = result.second;
        cmd_vel = cmd_vel_tmp;
      }
    }
  }

  return cmd_vel;
}

RecoveryBehavior::PointT RecoveryBehavior::find_nearest_point(void)
{
  float min_distance = std::numeric_limits<float>::max();
  std::optional<PointT> nearest_point;
  Footprint footprint;

  for (const auto &point : pcl_cloud_)
  {
    const float distance = footprint.calc_dist_to_obs(point.x, point.y);
    const float angle = atan2(point.y, point.x);
    if (distance < min_distance && -M_PI / 2.0 < angle && angle < M_PI / 2.0)
    {
      min_distance = distance;
      nearest_point = point;
    }
  }

  return nearest_point.has_value() ? nearest_point.value() : pcl_cloud_.points.front();
}

std::pair<bool, float>
RecoveryBehavior::simulate_with_check_collision_and_calc_dist_to_nearest_point(const geometry_msgs::Twist &cmd_vel)
{
  Footprint footprint;
  geometry_msgs::Point32 robot_state;
  const float sim_time_resolution = param_.move_time / param_.sim_time_samples;

  for (int i = 0; i < param_.sim_time_samples; i++)
  {
    robot_state.z += cmd_vel.angular.z * sim_time_resolution;
    robot_state.x += cmd_vel.linear.x * cos(robot_state.z) * sim_time_resolution;
    robot_state.y += cmd_vel.linear.x * sin(robot_state.z) * sim_time_resolution;
    for (const auto &point : pcl_cloud_)
    {
      if (footprint.move_footprint(robot_state.x, robot_state.y, robot_state.z).check_collision(point.x, point.y))
        return std::make_pair(false, 0.0);
    }
  }

  float min_dist = std::numeric_limits<float>::max();
  for (const auto &point : pcl_cloud_)
    min_dist = std::min(
        footprint.move_footprint(robot_state.x, robot_state.y, robot_state.z).calc_dist_to_obs(point.x, point.y),
        min_dist);

  return std::make_pair(true, min_dist);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "recovery_behavior");
  RecoveryBehavior recovery_behavior;
  ros::spin();

  return 0;
}
