#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    # Inject some reasonable default covariances
    msg.orientation_covariance = [0.01, 0, 0,
                                  0, 0.01, 0,
                                  0, 0, 0.01]

    msg.angular_velocity_covariance = [0.01, 0, 0,
                                       0, 0.01, 0,
                                       0, 0, 0.01]

    msg.linear_acceleration_covariance = [0.1, 0, 0,
                                          0, 0.1, 0,
                                          0, 0, 0.1]

    pub.publish(msg)

rospy.init_node('imu_covariance_injector')

sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
pub = rospy.Publisher('/imu/data_with_cov', Imu, queue_size=10)

rospy.spin()