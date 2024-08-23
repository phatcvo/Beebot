/**
 * @file local_goal_creator.cpp
 * @author phatcvo
 * @brief C++ implementation of local goal creator
 * @copyright Copyright (c) 2024
 */

#include <algorithm>

#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator(void) : private_nh_("~")
{
  private_nh_.param<int>("hz", hz_, 10);
  private_nh_.param<float>("target_dist_to_goal", target_dist_to_goal_, 0.5);
  private_nh_.param<bool>("use_direction_in_path", use_direction_in_path_, false);

  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
  path_sub_ = nh_.subscribe("/path", 1, &LocalGoalCreator::path_callback, this);
  pose_sub_ = nh_.subscribe("/robot_pose", 1, &LocalGoalCreator::pose_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("target_dist_to_goal: " << target_dist_to_goal_);
  ROS_INFO_STREAM("use_direction_in_path: " << use_direction_in_path_);
}

void LocalGoalCreator::path_callback(const nav_msgs::Path::ConstPtr &msg) { path_ = *msg; }

void LocalGoalCreator::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  robot_pose_ = *msg;
}

void LocalGoalCreator::process(void)
{
  ros::Rate loop_rate(hz_);

  std::optional<nav_msgs::Path> prev_path;
  geometry_msgs::PoseStamped goal_pose;

  while (ros::ok())
  {
    ros::spinOnce();

    if (robot_pose_.has_value() && path_.has_value())
    {
      if (prev_path.has_value())
      {
        goal_pose = update_goal(
            robot_pose_.value(), path_.value(), target_dist_to_goal_, use_direction_in_path_, prev_path.value(),
            goal_pose);
      }
      else
      {
        goal_pose = create_goal(robot_pose_.value(), path_.value(), target_dist_to_goal_, use_direction_in_path_, 0);
      }
      goal_pose.header.stamp = ros::Time::now();
      goal_pose.header.frame_id = path_.value().header.frame_id;
      goal_pub_.publish(goal_pose);
      prev_path = path_.value();
    }

    loop_rate.sleep();
  }
}

geometry_msgs::PoseStamped LocalGoalCreator::update_goal(
    const geometry_msgs::PoseWithCovarianceStamped &robot_pose, const nav_msgs::Path &path,
    const float target_dist_to_goal, const bool use_direction_in_path, const nav_msgs::Path prev_path,
    const geometry_msgs::PoseStamped &prev_goal_pose)
{
  int start_index = 0;
  if (!is_changed_path(path, prev_path))
  {
    float min_dist = FLT_MAX;
    for (int i = 0; i < path.poses.size(); i++)
    {
      const float dist = calc_dist_between_points(prev_goal_pose.pose.position, path.poses[i].pose.position);
      if (dist < min_dist)
      {
        min_dist = dist;
        start_index = i;
      }
    }
  }

  return create_goal(robot_pose, path, target_dist_to_goal, use_direction_in_path, start_index);
}

bool LocalGoalCreator::is_changed_path(const nav_msgs::Path &path, const nav_msgs::Path &prev_path)
{
  if (path.poses.size() == 0)
  {
    ROS_WARN("Path is empty");
    return false;
  }
  else if (prev_path.poses.size() == 0)
  {
    ROS_WARN("Previous path is empty");
    return true;
  }
  const bool is_changed_start = path.poses.front().pose.position.x != prev_path.poses.front().pose.position.x ||
                                path.poses.front().pose.position.y != prev_path.poses.front().pose.position.y;
  const bool is_changed_goal = path.poses.back().pose.position.x != prev_path.poses.back().pose.position.x ||
                               path.poses.back().pose.position.y != prev_path.poses.back().pose.position.y;

  return is_changed_start || is_changed_goal;
}

geometry_msgs::PoseStamped LocalGoalCreator::create_goal(
    const geometry_msgs::PoseWithCovarianceStamped &robot_pose, const nav_msgs::Path &path,
    const float target_dist_to_goal, const bool use_direction_in_path, int start_index)
{
  if (path.poses.size() == 0)
  {
    ROS_WARN("Path is empty");
    return geometry_msgs::PoseStamped();
  }
  else if (path.poses.size() <= start_index)
  {
    ROS_WARN("Start index is out of range");
    return geometry_msgs::PoseStamped();
  }

  std::optional<geometry_msgs::PoseStamped> goal_pose;
  int i = std::max(start_index, 0);
  while (i < path.poses.size())
  {
    if (calc_dist_between_points(robot_pose.pose.pose.position, path.poses[i].pose.position) >= target_dist_to_goal)
    {
      goal_pose = path.poses[i];
      break;
    }
    i++;
  }

  if (!goal_pose.has_value())
    goal_pose = path.poses.back();
  if (!use_direction_in_path)
  {
    if (path.poses.size() == 1)
      goal_pose.value().pose.orientation = robot_pose.pose.pose.orientation;
    else if (i != path.poses.size() - 1)
      goal_pose.value().pose.orientation = calc_direction(path.poses[i].pose.position, path.poses[i + 1].pose.position);
    else
      goal_pose.value().pose.orientation = calc_direction(path.poses[i - 1].pose.position, path.poses[i].pose.position);
  }

  return goal_pose.value();
}

float LocalGoalCreator::calc_dist_between_points(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
  return hypot(point2.x - point1.x, point2.y - point1.y);
}

geometry_msgs::Quaternion
LocalGoalCreator::calc_direction(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(point2.y - point1.y, point2.x - point1.x));
  geometry_msgs::Quaternion q_msg;
  tf2::convert(q, q_msg);
  return q_msg;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "local_goal_creator");
  LocalGoalCreator local_goal_creator;
  local_goal_creator.process();

  return 0;
}
