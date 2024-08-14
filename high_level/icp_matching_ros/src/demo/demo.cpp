/**
 * @file demo.cpp
 * @author beebot
 * @brief Demo for ICP matching
 * @copyright Copyright (c) 2024
 */

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include "icp_matching/icp_matching.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[])
{
  // initialize ROS
  ros::init(argc, argv, "icp_matching_demo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher cloud_src_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_src", 1);
  ros::Publisher cloud_target_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_target", 1);
  ros::Publisher cloud_src_registered_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_src_registered", 1);

  // read parameters
  bool enable_downsampling;
  float leaf_size, sleep_time;
  std::string src_pcd_path, target_pcd_path, frame_id;
  pnh.param<std::string>("src_pcd_path", src_pcd_path, std::string("src_pcd.pcd"));
  pnh.param<std::string>("target_pcd_path", target_pcd_path, std::string("target_pcd.pcd"));
  pnh.param<std::string>("frame_id", frame_id, std::string("map"));
  pnh.param<bool>("enable_downsampling", enable_downsampling, false);
  pnh.param<float>("leaf_size", leaf_size, 0.003);
  pnh.param<float>("sleep_time", sleep_time, 0.3);

  // print information
  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("  src_pcd_path: " << src_pcd_path);
  ROS_INFO_STREAM("  target_pcd_path: " << target_pcd_path);
  ROS_INFO_STREAM("  frame_id: " << frame_id);
  ROS_INFO_STREAM("  enable_downsampling: " << enable_downsampling);
  ROS_INFO_STREAM("  leaf_size: " << leaf_size);
  ROS_INFO_STREAM("  sleep_time: " << sleep_time);

  // initialize
  ICPMatching icp_matching;

  // read pcd files
  PointCloudT::Ptr cloud_src(new PointCloudT), cloud_target(new PointCloudT);
  icp_matching.read_pcd(src_pcd_path, cloud_src);
  icp_matching.read_pcd(target_pcd_path, cloud_target);

  // voxel grid filter
  if (enable_downsampling)
  {
    icp_matching.voxel_grid_filter(cloud_src, leaf_size, cloud_src);
    icp_matching.voxel_grid_filter(cloud_target, leaf_size, cloud_target);
  }

  // convert to ROS message
  sensor_msgs::PointCloud2 cloud_src_msg, cloud_target_msg, cloud_src_registered_msg;
  pcl::toROSMsg(*cloud_src, cloud_src_msg);
  pcl::toROSMsg(*cloud_target, cloud_target_msg);
  cloud_src_msg.header.frame_id = frame_id;
  cloud_target_msg.header.frame_id = frame_id;

  // initial publish
  ros::Duration(1.0).sleep();
  cloud_src_pub.publish(cloud_src_msg);
  cloud_target_pub.publish(cloud_target_msg);
  cloud_src_registered_pub.publish(cloud_src_msg);
  cloud_src_pub.publish(cloud_src_msg);

  // ICP matching and publish registered point cloud
  double prev_fitness_score = 0.0;
  for (int i = 0; ros::ok(); i++)
  {
    // sleep
    ros::Duration(sleep_time).sleep();
    // ICP matching
    PointCloudT cloud_src_registered;
    icp_matching.align(cloud_src, cloud_target, cloud_src_registered, i);
    ROS_INFO_STREAM("ICP iteration: " << i << ", fitness score: " << icp_matching.get_fitness_score());
    pcl::toROSMsg(cloud_src_registered, cloud_src_registered_msg);
    cloud_src_registered_msg.header.frame_id = frame_id;
    // publish
    cloud_src_pub.publish(cloud_src_msg);
    cloud_target_pub.publish(cloud_target_msg);
    cloud_src_registered_pub.publish(cloud_src_registered_msg);
    // check convergence
    if (abs(icp_matching.get_fitness_score() - prev_fitness_score) < DBL_EPSILON &&
        icp_matching.get_fitness_score() < 1e-4)
      break;
    else
      prev_fitness_score = icp_matching.get_fitness_score();
  }

  // print final transformation matrix
  ROS_INFO_STREAM(std::endl << "Transform matrix:" << std::endl << icp_matching.get_transformation());

  return 0;
}
