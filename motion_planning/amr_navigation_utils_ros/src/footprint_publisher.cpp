/**
 * @file footprint_publisher.cpp
 * @author beebot
 * @brief Footprint publisher
 * @copyright Copyright (c) 2024
 */

#include "amr_navigation_utils_ros/footprint.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "footprint_publisher");
  ros::NodeHandle nh;
  ros::Publisher footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
  Footprint footprint;
  geometry_msgs::PolygonStamped footprint_msg = footprint.get_footprint();

  ros::Rate rate(1);
  while (ros::ok())
  {
    footprint_msg.header.stamp = ros::Time::now();
    footprint_pub.publish(footprint_msg);
    rate.sleep();
  }

  return 0;
}
