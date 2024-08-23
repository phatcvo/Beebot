/**
 * @file recovery_behavior.h
 * @author phatcvo
 * @brief Recovery behavior class
 * @copyright Copyright (c) 2024
 */

#ifndef RECOVERY_BEHAVIOR_RECOVERY_BEHAVIOR_H
#define RECOVERY_BEHAVIOR_RECOVERY_BEHAVIOR_H

#include <geometry_msgs/Twist.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <utility>

#include "amr_navigation_utils_ros/footprint.h"

/**
 * @struct RecoveryBehaviorParam
 * @brief Recovery behavior parameter
 */
struct RecoveryBehaviorParam
{
  int hz;
  int sim_time_samples;
  int vel_samples;
  float move_time;
  float min_vel_x;
  float max_vel_theta;
};

/**
 * @class RecoveryBehavior
 * @brief Recovery behavior class
 */
class RecoveryBehavior
{
public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  /**
   * @brief Construct a new Recovery Behavior object
   */
  RecoveryBehavior(void);

private:
  /**
   * @brief Callback function for point cloud
   */
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief Callback function for trigger service to start recovery behavior
   *
   * @param req Request
   * @param res Response
   * @return true Recovery behavior has finished
   * @return false Failed to start recovery behavior
   */
  bool trigger_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  /**
   * @brief Move the robot for a certain time
   */
  void recovery_move(void);

  /**
   * @brief Select the best command velocity
   *
   * @return geometry_msgs::Twist Best command velocity
   */
  geometry_msgs::Twist select_best_cmd_vel(void);

  /**
   * @brief Find the nearest point from the robot
   *
   * @return PointT Nearest point
   */
  PointT find_nearest_point(void);

  /**
   * @brief Simulate the robot with the given command velocity, and check collision
   *
   * @param cmd_vel Command velocity
   * @return std::pair<bool, float> Result of simulation and distance to the nearest point
   */
  std::pair<bool, float>
  simulate_with_check_collision_and_calc_dist_to_nearest_point(const geometry_msgs::Twist &cmd_vel);

  RecoveryBehaviorParam param_;
  PointCloudT pcl_cloud_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher footprint_pub_;
  ros::Subscriber cloud_sub_;
  ros::ServiceServer trigger_service_;
};

#endif  // RECOVERY_BEHAVIOR_RECOVERY_BEHAVIOR_H
