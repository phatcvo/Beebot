/**
 * @file footprint.h
 * @author beebot
 * @brief Footprint class
 * @copyright Copyright (c) 2024
 */

#ifndef AMR_NAVIGATION_UTILS_ROS_FOOTPRINT_H
#define AMR_NAVIGATION_UTILS_ROS_FOOTPRINT_H

#include <Eigen/Dense>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <string>

/**
 * @struct FootprintParams
 * @brief Footprint parameters
 */
struct FootprintParams
{
  std::string frame_id;
  float front_side_length;
  float rear_side_length;
  float left_side_length;
  float right_side_length;
};

/**
 * @class Footprint
 * @brief Footprint class
 */
class Footprint
{
public:
  /**
   * @brief Construct a new Footprint object
   */
  Footprint(void);

  /**
   * @brief Construct a new Footprint object
   *
   * @param footprint footprint
   * @param origin origin of the footprint
   */
  Footprint(const geometry_msgs::PolygonStamped &footprint, const geometry_msgs::Point32 &origin);

  /**
   * @brief Print parameters
   */
  void print_params(void);

  /**
   * @brief Get the footprint object
   *
   * @return geometry_msgs::PolygonStamped footprint
   */
  geometry_msgs::PolygonStamped get_footprint(void) { return footprint_; }

  /**
   * @brief Check if the point is inside of the footprint
   *
   * @param x x
   * @param y y
   * @return true if the point is inside of the footprint
   * @return false if the point is outside of the footprint
   */
  bool check_collision(const float x, const float y);

  /**
   * @brief Check if the point is inside of the triangle
   *
   * @param x x
   * @param y y
   * @param triangle triangle
   * @return true if the point is inside of the triangle
   * @return false if the point is outside of the triangle
   */
  bool is_inside_of_triangle(const float x, const float y, const geometry_msgs::Polygon &triangle);

  /**
   * @brief Calcualte the distance to the obstacle
   *
   * @param x x
   * @param y y
   * @return float distance to the obstacle
   */
  float calc_dist_to_obs(const float x, const float y);

  /**
   * @brief Move the footprint
   *
   * @param x x
   * @param y y
   * @param yaw yaw
   * @return Footprint footprint
   */
  Footprint move_footprint(const float x, const float y, const float yaw);

private:
  /**
   * @brief Create the footprint
   */
  void create_footprint(void);

  /**
   * @brief Calculate the intersection point
   *
   * @param x x
   * @param y y
   * @return geometry_msgs::Point32 intersection point
   */
  geometry_msgs::Point32 calc_intersection(const float x, const float y);

  FootprintParams params_;
  geometry_msgs::PolygonStamped footprint_;
  geometry_msgs::Point32 origin_;
  ros::NodeHandle private_nh_;
};

#endif  // AMR_NAVIGATION_UTILS_ROS_FOOTPRINT_H
