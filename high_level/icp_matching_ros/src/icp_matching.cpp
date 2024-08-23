/**
 * @file icp_matching.cpp
 * @author phatcvo
 * @brief ICP matching class
 * @copyright Copyright (c) 2024
 */

#include <string>

#include "icp_matching/icp_matching.h"

ICPMatching::ICPMatching(void)
    : has_converged_(false), fitness_score_(0.0), transformation_(Eigen::Matrix4f::Identity())
{
}

void ICPMatching::read_pcd(const std::string &file_name, PointCloudT::Ptr cloud)
{
  while (ros::ok() && pcl::io::loadPCDFile<PointT>(file_name, *cloud) == -1)
  {
    ROS_WARN_STREAM("Couldn't read file " << file_name);
    ros::Duration(1.0).sleep();
  }
  ROS_INFO_STREAM("Loaded " << cloud->size() << " data points from " << file_name);
}

void ICPMatching::voxel_grid_filter(
    const PointCloudT::Ptr cloud, const float leaf_size, PointCloudT::Ptr cloud_filtered)
{
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloud_filtered);
}

void ICPMatching::align(
    const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_target, PointCloudT &cloud_src_registered,
    const int iterations, const double max_correspondence_distance)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  icp.setInputSource(cloud_src);
  icp.setInputTarget(cloud_target);
  icp.setMaximumIterations(iterations);
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-8);
  icp.align(cloud_src_registered);

  has_converged_ = icp.hasConverged();
  fitness_score_ = icp.getFitnessScore();
  transformation_ = icp.getFinalTransformation();
}
