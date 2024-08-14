/**
 * @file base_controller.h
 * @author beebot
 * @brief base_controller for mobile robot
 * @copyright Copyright (c) 2024
 */

#ifndef BASE_CONTROLLER_BASE_CONTROLLER_H
#define BASE_CONTROLLER_BASE_CONTROLLER_H

#include <string>
#include <utility>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/**
 * @struct ControllerParam
 * @brief Controller parameters
 */
struct ControllerParam
{
  int hz = 0;
  double max_linear_vel = 0.0;
  double max_angular_vel = 0.0;
};

/**
 * @class BaseController
 * @brief Base controller class
 */
class BaseController
{
public:
  /**
   * @brief Construct a new Base Controller object
   */
  BaseController(void);

  /**
   * @brief Process
   */
  void process(void);

private:
  /**
   * @brief Joy callback
   *
   * @param msg Joy message
   */
  void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);

  /**
   * @brief Planner cmd vel callback
   *
   * @param msg Planner cmd vel message
   */
  void planner_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);

  /**
   * @brief Select mode
   *
   * @param joy Joy message
   * @param prev_mode Previous mode
   * @return std::pair<std::string, std::string> Current mode
   */
  std::pair<std::string, std::string>
  select_mode(const sensor_msgs::Joy::ConstPtr &joy, const std::pair<std::string, std::string> &prev_mode);

  /**
   * @brief Joy to cmd vel
   *
   * @param joy Joy message
   * @return geometry_msgs::Twist Cmd vel message
   */
  geometry_msgs::Twist joy_to_cmd_vel(const sensor_msgs::Joy::ConstPtr &joy);

  /**
   * @brief Print status
   *
   * @param mode Current mode
   * @param cmd_vel Cmd vel message
   */
  void print_status(const std::pair<std::string, std::string> &mode, const geometry_msgs::Twist &cmd_vel);

  ControllerParam param_;
  std::pair<std::string, std::string> mode_;  // <stop, move>, <auto, manual>

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber planner_cmd_vel_sub_;

  geometry_msgs::Twist cmd_vel_;
};

#endif  // BASE_CONTROLLER_BASE_CONTROLLER_H
