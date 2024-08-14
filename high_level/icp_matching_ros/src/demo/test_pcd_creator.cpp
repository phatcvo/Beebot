/**
 * @file test_pcd_creator.cpp
 * @author beebot
 * @brief Create a PCD file for icp_matching test
 * @copyright Copyright (c) 2024
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/transforms.h>
#include <random>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>

int main(int argc, char *argv[])
{
  // initialize ROS
  ros::init(argc, argv, "test_pcd_creator");
  ros::NodeHandle pnh("~");

  // read parameters
  std::string src_pcd_path, target_pcd_path;
  int point_num;
  float rotation_yaw, translation_x, translation_y;
  pnh.param<std::string>("src_pcd_path", src_pcd_path, std::string("src_pcd.pcd"));
  pnh.param<std::string>("target_pcd_path", target_pcd_path, std::string("target_pcd.pcd"));
  pnh.param<int>("point_num", point_num, 20);
  pnh.param<float>("rotation_yaw", rotation_yaw, 0.2);
  pnh.param<float>("translation_x", translation_x, 5.0);
  pnh.param<float>("translation_y", translation_y, -2.0);

  // print information
  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("  src_pcd_path: " << src_pcd_path);
  ROS_INFO_STREAM("  target_pcd_path: " << target_pcd_path);
  ROS_INFO_STREAM("  point_num: " << point_num);
  ROS_INFO_STREAM("  rotation_yaw: " << rotation_yaw);
  ROS_INFO_STREAM("  translation_x: " << translation_x);
  ROS_INFO_STREAM("  translation_y: " << translation_y);

  // create point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_src;
  cloud_src.is_dense = false;
  cloud_src.resize(point_num);
  std::random_device rd;
  std::mt19937 gen{rd()};
  std::uniform_int_distribution<> distrib_x(0.0, 20.0);
  std::uniform_int_distribution<> distrib_y(0.0, 10.0);
  for (auto &point : cloud_src.points)
  {
    point.x = distrib_x(gen);
    point.y = distrib_y(gen);
    point.z = 0.0;
  }

  // transform point cloud
  tf::Quaternion q;
  q.setRPY(0, 0, rotation_yaw);
  tf::Transform tf(q, tf::Vector3(translation_x, translation_y, 0.0));
  pcl::PointCloud<pcl::PointXYZ> cloud_target;
  pcl_ros::transformPointCloud(cloud_src, cloud_target, tf);

  // save point cloud
  pcl::io::savePCDFileASCII(src_pcd_path, cloud_src);
  ROS_INFO_STREAM("Saved PCD to " << src_pcd_path);
  pcl::io::savePCDFileASCII(target_pcd_path, cloud_target);
  ROS_INFO_STREAM("Saved PCD to " << target_pcd_path);

  return 0;
}
