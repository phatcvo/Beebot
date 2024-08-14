/**
 * @file footprint.cpp
 * @author beebot
 * @brief Footprint class
 * @copyright Copyright (c) 2024
 */

#include <string>

#include "amr_navigation_utils_ros/footprint.h"

Footprint::Footprint(void) : private_nh_("~")
{
  private_nh_.param<std::string>("frame_id", params_.frame_id, std::string("base_footprint"));
  private_nh_.param<float>("front_side_length", params_.front_side_length, 0.5);
  private_nh_.param<float>("rear_side_length", params_.rear_side_length, 0.5);
  private_nh_.param<float>("left_side_length", params_.left_side_length, 0.5);
  private_nh_.param<float>("right_side_length", params_.right_side_length, 0.5);

  create_footprint();
}

Footprint::Footprint(const geometry_msgs::PolygonStamped &footprint, const geometry_msgs::Point32 &origin)
    : footprint_(footprint), origin_(origin)
{
}

void Footprint::create_footprint(void)
{
  geometry_msgs::Point32 point;

  // right front
  point.x = params_.front_side_length;
  point.y = -params_.right_side_length;
  footprint_.polygon.points.push_back(point);

  // right rear
  point.x = -params_.rear_side_length;
  footprint_.polygon.points.push_back(point);

  // left rear
  point.y = params_.left_side_length;
  footprint_.polygon.points.push_back(point);

  // left front
  point.x = params_.front_side_length;
  footprint_.polygon.points.push_back(point);

  // frame_id
  footprint_.header.frame_id = params_.frame_id;
}

void Footprint::print_params(void)
{
  ROS_INFO("Footprint parameters:");
  ROS_INFO_STREAM("  frame_id: " << params_.frame_id);
  ROS_INFO_STREAM("  front_side_length: " << params_.front_side_length);
  ROS_INFO_STREAM("  rear_side_length: " << params_.rear_side_length);
  ROS_INFO_STREAM("  left_side_length: " << params_.left_side_length);
  ROS_INFO_STREAM("  right_side_length: " << params_.right_side_length);
}

bool Footprint::check_collision(const float x, const float y)
{
  for (int i = 0; i < footprint_.polygon.points.size(); i++)
  {
    geometry_msgs::Polygon triangle;
    triangle.points.push_back(origin_);
    triangle.points.push_back(footprint_.polygon.points[i]);
    triangle.points.push_back(footprint_.polygon.points[(i + 1) == footprint_.polygon.points.size() ? 0 : i + 1]);
    if (is_inside_of_triangle(x, y, triangle))
      return true;
  }

  return false;
}

bool Footprint::is_inside_of_triangle(const float x, const float y, const geometry_msgs::Polygon &triangle)
{
  if (triangle.points.size() != 3)
  {
    ROS_ERROR("Not triangle");
    return false;
  }

  const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
  const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
  const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
  const Eigen::Vector3d vector_P(x, y, 0.0);

  const Eigen::Vector3d vector_AB = vector_B - vector_A;
  const Eigen::Vector3d vector_BP = vector_P - vector_B;
  const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

  const Eigen::Vector3d vector_BC = vector_C - vector_B;
  const Eigen::Vector3d vector_CP = vector_P - vector_C;
  const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

  const Eigen::Vector3d vector_CA = vector_A - vector_C;
  const Eigen::Vector3d vector_AP = vector_P - vector_A;
  const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

  if ((0 < cross1.z() && 0 < cross2.z() && 0 < cross3.z()) || (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0))
    return true;
  else
    return false;
}

float Footprint::calc_dist_to_obs(const float x, const float y)
{
  if (check_collision(x, y))
  {
    return 0.0;
  }
  else
  {
    geometry_msgs::Point32 intersection = calc_intersection(x, y);
    return hypot((x - intersection.x), (y - intersection.y));
  }
}

geometry_msgs::Point32 Footprint::calc_intersection(const float x, const float y)
{
  geometry_msgs::Point32 intersection;

  for (int i = 0; i < footprint_.polygon.points.size(); i++)
  {
    const Eigen::Vector3d vector_A(x, y, 0.0);
    const Eigen::Vector3d vector_B(origin_.x, origin_.y, 0.0);
    const Eigen::Vector3d vector_C(footprint_.polygon.points[i].x, footprint_.polygon.points[i].y, 0.0);
    const Eigen::Vector3d vector_D(
        footprint_.polygon.points[(i + 1) == footprint_.polygon.points.size() ? 0 : i + 1].x,
        footprint_.polygon.points[(i + 1) == footprint_.polygon.points.size() ? 0 : i + 1].y, 0.0);

    const float deno = (vector_B - vector_A).cross(vector_D - vector_C).z();
    const float s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
    const float t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

    intersection.x = vector_A.x() + s * (vector_B - vector_A).x();
    intersection.y = vector_A.y() + s * (vector_B - vector_A).y();

    if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t))
      return intersection;  // intersection
  }

  intersection.x = 1.0e+10;
  intersection.y = 1.0e+10;

  return intersection;  // no intersection
}

Footprint Footprint::move_footprint(const float x, const float y, const float yaw)
{
  Eigen::Translation2f translation(x, y);
  Eigen::Rotation2Df rotation(yaw);
  Eigen::Transform<float, 2, Eigen::Isometry> transform = translation * rotation;

  geometry_msgs::PolygonStamped footprint_tmp = footprint_;
  for (auto &point : footprint_tmp.polygon.points)
  {
    Eigen::Vector2f point_in(point.x, point.y);
    Eigen::Vector2f point_out = transform * point_in;
    point.x = point_out.x();
    point.y = point_out.y();
  }

  Eigen::Vector2f origin_in(origin_.x, origin_.y);
  Eigen::Vector2f origin_out = transform * origin_in;
  geometry_msgs::Point32 origin_tmp;
  origin_tmp.x = origin_out.x();
  origin_tmp.y = origin_out.y();

  Footprint footprint(footprint_tmp, origin_tmp);
  return footprint;
}
