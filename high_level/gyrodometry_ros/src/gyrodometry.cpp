/**
 * @file gyrodometry.cpp
 * @author beebot
 * @brief C++ implementation of Gyrodometry for 2D navigation
 * @copyright Copyright (c) 2024
 */

#include <nav_msgs/Odometry.h>
#include <optional>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * @class Gyrodometry
 * @brief Class for estimating odometry using gyroscope
 */
class Gyrodometry
{
public:
  /**
   * @brief Construct a new Gyrodometry object
   */
  Gyrodometry(void) : private_nh_("~")
  {
    private_nh_.param<std::string>("child_frame_id", gyrodom_.child_frame_id, std::string("gyrodom"));

    gyrodom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gyrodom", 1);
    imu_sub_ =
        nh_.subscribe("/imu", 1, &Gyrodometry::imu_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    odom_sub_ =
        nh_.subscribe("/odom", 1, &Gyrodometry::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO_STREAM("child_frame_id: " << gyrodom_.child_frame_id);
  }

private:
  /**
   * @brief Callback function for imu
   * @param msg imu message
   */
  void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    if (last_imu_.has_value())
    {
      const double dt = (msg->header.stamp - last_imu_->header.stamp).toSec();
      yaw_ += (msg->angular_velocity.z + last_imu_->angular_velocity.z) / 2.0 * dt;
      if (last_odom_.has_value())
        publish_gyrodom(msg->header.stamp);
    }
    last_imu_ = *msg;
  }

  /**
   * @brief Callback function for odometry
   * @param msg odom message
   */
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (last_odom_.has_value())
    {
      const double dt = (msg->header.stamp - last_odom_->header.stamp).toSec();
      gyrodom_.pose.pose.position.x += msg->twist.twist.linear.x * dt * cos(yaw_);
      gyrodom_.pose.pose.position.y += msg->twist.twist.linear.x * dt * sin(yaw_);
    }
    else
    {
      gyrodom_.header.frame_id = msg->header.frame_id;
      gyrodom_.pose.pose.position = msg->pose.pose.position;
      yaw_ += tf2::getYaw(msg->pose.pose.orientation);
    }
    last_odom_ = *msg;
  }

  /**
   * @brief Publish gyrodometry
   * @param stamp timestamp
   */
  void publish_gyrodom(const ros::Time &stamp)
  {
    // Publish gyrodom
    gyrodom_.header.stamp = stamp;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    tf2::convert(q, gyrodom_.pose.pose.orientation);
    gyrodom_pub_.publish(gyrodom_);
    // broadcast tf
    geometry_msgs::TransformStamped transform;
    transform.header = gyrodom_.header;
    transform.child_frame_id = gyrodom_.child_frame_id;
    transform.transform.translation.x = gyrodom_.pose.pose.position.x;
    transform.transform.translation.y = gyrodom_.pose.pose.position.y;
    transform.transform.rotation = gyrodom_.pose.pose.orientation;
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transform);
  }

  double yaw_ = 0.0;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher gyrodom_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;

  nav_msgs::Odometry gyrodom_;
  std::optional<sensor_msgs::Imu> last_imu_;
  std::optional<nav_msgs::Odometry> last_odom_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gyrodometry");
  Gyrodometry gyrodometry;
  ros::spin();

  return 0;
}
