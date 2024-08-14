/**
 * @file pointcloud_angle_filter.cpp
 * @author beebot
 * @brief Filter the pointcloud by the angle
 * @copyright Copyright (c) 2024
 */

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * @class PointCloudAngleFilter
 * @brief Filter the pointcloud by the angle
 */
class PointCloudAngleFilter
{
public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  /**
   * @brief Construct a new Point Cloud Angle Filter object
   */
  PointCloudAngleFilter(void) : private_nh_("~")
  {
    private_nh_.param<std::vector<float>>("valid_angle_range_list", valid_angle_range_list_, {-M_PI / 2, M_PI / 2});
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
    cloud_sub_ =
        nh_.subscribe("/cloud", 1, &PointCloudAngleFilter::cloud_callback, this, ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO("valid_angle_range_list:");
    for (int i = 0; i < valid_angle_range_list_.size(); i += 2)
    {
      ROS_INFO_STREAM("  " << valid_angle_range_list_[i] << ", " << valid_angle_range_list_[i + 1]);
    }
  }

private:
  /**
   * @brief Callback function for the pointcloud
   *
   * @param msg PointCloud message
   */
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    sensor_msgs::PointCloud2 cloud_filtered = filter_cloud(*msg);
    cloud_filtered.header = msg->header;
    cloud_pub_.publish(cloud_filtered);
  }

  /**
   * @brief Filter the pointcloud
   *
   * @param cloud
   * @return sensor_msgs::PointCloud2 Filtered pointcloud
   */
  sensor_msgs::PointCloud2 filter_cloud(const sensor_msgs::PointCloud2 &cloud)
  {
    PointCloudT pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    for (auto it = pcl_cloud.begin(); it != pcl_cloud.end();)
    {
      if (!is_valid_point(*it))
        it = pcl_cloud.erase(it);
      else
        ++it;
    }
    sensor_msgs::PointCloud2 cloud_filtered;
    pcl::toROSMsg(pcl_cloud, cloud_filtered);
    return cloud_filtered;
  }

  /**
   * @brief Check if the point is valid
   *
   * @param point Point
   * @return true If the point is valid
   * @return false If the point is invalid
   */
  bool is_valid_point(const PointT &point)
  {
    const float angle = atan2(point.y, point.x);
    for (int i = 0; i < valid_angle_range_list_.size(); i += 2)
    {
      if (valid_angle_range_list_[i] <= angle && angle <= valid_angle_range_list_[i + 1])
        return true;
    }
    return false;
  }

  std::vector<float> valid_angle_range_list_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_angle_filter");
  PointCloudAngleFilter pointcloud_angle_filter;
  ros::spin();

  return 0;
}
