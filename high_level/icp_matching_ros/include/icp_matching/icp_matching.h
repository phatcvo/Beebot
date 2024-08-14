/**
 * @file icp_matching.h
 * @author beebot
 * @brief ICP matching class
 * @copyright Copyright (c) 2024
 */

#ifndef ICP_MATCHING_ICP_MATCHING_H
#define ICP_MATCHING_ICP_MATCHING_H

#include <limits>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <string>

/**
 * @class ICPMatching
 * @brief ICP matching class
 */
class ICPMatching
{
public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  /**
   * @brief Construct a new ICPMatching object
   */
  ICPMatching(void);

  /**
   * @brief Read PCD file
   *
   * @param file_name PCD file name
   * @param cloud Point cloud
   */
  void read_pcd(const std::string &file_name, PointCloudT::Ptr cloud);

  /**
   * @brief Downsample point cloud using voxel grid filter
   *
   * @param cloud Point cloud
   * @param leaf_size Leaf size
   * @param cloud_filtered Filtered point cloud
   */
  void voxel_grid_filter(const PointCloudT::Ptr cloud, const float leaf_size, PointCloudT::Ptr cloud_filtered);

  /**
   * @brief Align two point clouds
   *
   * @param cloud_src Source point cloud
   * @param cloud_target Target point cloud
   * @param cloud_src_registered Registered point cloud
   * @param iterations Maximum iterations
   */
  void align(
      const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_target, PointCloudT &cloud_src_registered,
      const int iterations = 10,
      const double max_correspondence_distance = std::sqrt(std::numeric_limits<double>::max()));

  /**
   * @brief Check if ICP has converged
   *
   * @return true if ICP has converged
   * @return false if ICP has not converged
   */
  bool has_converged(void) { return has_converged_; }

  /**
   * @brief Get fitness score
   *
   * @return float Fitness score
   */
  float get_fitness_score(void) { return fitness_score_; }

  /**
   * @brief Get transformation matrix
   *
   * @return Eigen::Matrix4f Transformation matrix
   */
  Eigen::Matrix4f get_transformation(void) { return transformation_; }

private:
  bool has_converged_;
  float fitness_score_;
  Eigen::Matrix4f transformation_;
};

#endif  // ICP_MATCHING_ICP_MATCHING_H
