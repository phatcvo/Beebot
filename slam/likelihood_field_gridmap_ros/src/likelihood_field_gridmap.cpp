/**
 * @file likelihood_field_gridmap.cpp
 * @author phatcvo
 * @brief C++ implementation of likelihood field grid map
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

/**
 * @class LikelihoodFieldGridMap
 * @brief Class for likelihood field grid map
 */
class LikelihoodFieldGridMap
{
public:
  /**
   * @brief Construct a new Likelihood Field Grid Map object
   */
  LikelihoodFieldGridMap(void) : private_nh_("~")
  {
    private_nh_.param<int>("hz", hz_, 1);
    private_nh_.param<float>("likelihood_range", likelihood_range_, 1.0);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map/likelihood_field", 1);
    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO_STREAM("hz: " << hz_);
    ROS_INFO_STREAM("likelihood_range: " << likelihood_range_);
  }

  /**
   * @brief Process function
   */
  void process(void)
  {
    create_likelihood_field_map(get_map(), likelihood_range_, likelihood_field_map_);
    ros::Rate rate(hz_);
    while (ros::ok())
    {
      likelihood_field_map_.header.stamp = ros::Time::now();
      map_pub_.publish(likelihood_field_map_);
      rate.sleep();
    }
  }

private:
  /**
   * @brief Client function to get map
   */
  nav_msgs::OccupancyGrid get_map(void)
  {
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response resp;
    while (ros::ok() && !ros::service::call("/static_map", req, resp))
    {
      ROS_WARN_THROTTLE(2.0, "Waiting for a map");
      ros::Duration(0.5).sleep();
    }
    ROS_WARN("Received a map");

    return resp.map;
  }

  /**
   * @brief Create likelihood field map
   *
   * @param gridmap Original gridmap
   * @param likelihood_range Range of likelihood field
   * @param likelihood_field_map Likelihood field map
   */
  void create_likelihood_field_map(
      const nav_msgs::OccupancyGrid &gridmap, const float likelihood_range,
      nav_msgs::OccupancyGrid &likelihood_field_map)
  {
    std::vector<std::vector<float>> likelihood_fields(gridmap.info.width, std::vector<float>(gridmap.info.height, 0.0));
    const int likelihood_range_index = static_cast<int>(likelihood_range / gridmap.info.resolution);

    for (int i = 0; i < gridmap.data.size(); i++)
    {
      if (gridmap.data[i] != 100)
        continue;

      const int center_index_x = i % gridmap.info.width;
      const int center_index_y = static_cast<int>(i / gridmap.info.width);
      const int start_index_x = center_index_x - likelihood_range_index;
      const int start_index_y = center_index_y - likelihood_range_index;

      for (int j = 0; j < likelihood_range_index * 2; j++)
      {
        for (int k = 0; k < likelihood_range_index * 2; k++)
        {
          const int index_x = start_index_x + j;
          const int index_y = start_index_y + k;

          if (index_x < 0 || index_x >= gridmap.info.width || index_y < 0 || index_y >= gridmap.info.height)
            continue;

          const float distance =
              sqrt(pow(center_index_x - index_x, 2) + pow(center_index_y - index_y, 2)) * gridmap.info.resolution;
          likelihood_fields[index_x][index_y] =
              std::max(likelihood_fields[index_x][index_y], norm_pdf(distance, 0.0, likelihood_range / 2.0));
        }
      }
    }

    normalize(likelihood_fields);

    likelihood_field_map = gridmap;
    for (int i = 0; i < gridmap.info.width; i++)
      for (int j = 0; j < gridmap.info.height; j++)
        likelihood_field_map.data[i + j * gridmap.info.width] = 100 - static_cast<int>(likelihood_fields[i][j] * 100.0);
  }

  /**
   * @brief Normal distribution probability density function
   *
   * @param x Value
   * @param mean Mean
   * @param stddev Standard deviation
   * @return float Probability density
   */
  float norm_pdf(const float x, const float mean, const float stddev)
  {
    return 1.0 / sqrt(2.0 * M_PI * pow(stddev, 2.0)) * exp(-pow((x - mean), 2.0) / (2.0 * pow(stddev, 2.0)));
  }

  /**
   * @brief Normalize likelihood field
   *
   * @param likelihood_fields Likelihood fields
   */
  void normalize(std::vector<std::vector<float>> &likelihood_fields)
  {
    float max_likelihood = 0.0;
    for (const auto &likelihood_field : likelihood_fields)
      for (const auto &likelihood : likelihood_field)
        max_likelihood = std::max(max_likelihood, likelihood);

    for (auto &likelihood_field : likelihood_fields)
      for (auto &likelihood : likelihood_field)
        likelihood /= max_likelihood;
  }

  int hz_;
  float likelihood_range_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher map_pub_;
  nav_msgs::OccupancyGrid likelihood_field_map_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "likelihood_field_gridmap");
  LikelihoodFieldGridMap likelihood_field_gridmap;
  likelihood_field_gridmap.process();

  return 0;
}
