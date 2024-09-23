#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
    rospy.init_node('simple_publisher', anonymous=True)
    pub = rospy.Publisher('/test_topic', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 message per second

    while not rospy.is_shutdown():
        message = "Hello, ROS! " + str(rospy.get_time())
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
