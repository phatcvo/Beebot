#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_publisher():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    
    # Create a publisher for cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        twist = Twist()
        
        # Set linear and angular velocities
        twist.linear.x = 0.5  # Forward speed
        twist.angular.z = 0.1  # Turning speed

        # Publish the message
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
