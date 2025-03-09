#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
import tkinter as tk
import threading

class RosTkinterApp:
    def __init__(self, master):
        self.master = master
        master.title("ROS Publisher and Subscriber")

        self.label = tk.Label(master, text="Waiting for messages...")
        self.label.pack()

        self.yaw_label = tk.Label(master, text="Yaw: 0.0")
        self.yaw_label.pack()

        self.imu_label = tk.Label(master, text="IMU Data:\nAcc: (0.0, 0.0, 0.0)\nGyro: (0.0, 0.0, 0.0)")
        self.imu_label.pack()

        self.data = None
        self.yaw_data = 0.0
        self.imu_data = {'acc': (0.0, 0.0, 0.0), 'gyro': (0.0, 0.0, 0.0)}

        # Start the ROS node
        rospy.init_node('tkinter_ros_node', anonymous=True)

        # Publisher setup
        self.publisher = rospy.Publisher('/test_topic', String, queue_size=10)

        # Subscribe to the topics
        self.subscriber = rospy.Subscriber('/test_topic', String, self.callback)
        self.yaw_subscriber = rospy.Subscriber('/yaw', Float32, self.yaw_callback)
        self.imu_subscriber = rospy.Subscriber('/imu_data', Imu, self.imu_callback)

        # Create buttons
        self.publish_hello_button = tk.Button(master, text="Publish Hello", command=self.publish_hello)
        self.publish_hello_button.pack()

        self.publish_goodbye_button = tk.Button(master, text="Publish Goodbye", command=self.publish_goodbye)
        self.publish_goodbye_button.pack()

        # Start the GUI update loop
        self.update_gui()

    def publish_hello(self):
        msg = String()
        msg.data = "Hello, ROS! " + str(rospy.get_time())
        rospy.loginfo("Publishing: " + msg.data)
        self.publisher.publish(msg)

    def publish_goodbye(self):
        msg = String()
        msg.data = "Goodbye, ROS! " + str(rospy.get_time())
        rospy.loginfo("Publishing: " + msg.data)
        self.publisher.publish(msg)

    def callback(self, msg):
        self.data = msg.data  # Update the data when a new message arrives

    def yaw_callback(self, msg):
        self.yaw_data = msg.data  # Update yaw_data when a new message arrives

    def imu_callback(self, msg):
        # Update IMU data
        self.imu_data['acc'] = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.imu_data['gyro'] = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

    def update_gui(self):
        if self.data is not None:
            self.label.config(text=self.data)  # Update the label text
        
        # Update yaw label
        self.yaw_label.config(text="Yaw: {:.2f}".format(self.yaw_data))

        # Update IMU label
        acc = self.imu_data['acc']
        gyro = self.imu_data['gyro']
        self.imu_label.config(text="IMU Data:\nAcc: ({:.2f}, {:.2f}, {:.2f})\nGyro: ({:.2f}, {:.2f}, {:.2f})".format(
            acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]))

        self.master.after(100, self.update_gui)  # Call this method again after 100 ms

def ros_thread():
    rospy.spin()

if __name__ == "__main__":
    root = tk.Tk()
    app = RosTkinterApp(root)

    # Start a separate thread for ROS
    ros_thread_instance = threading.Thread(target=ros_thread)
    ros_thread_instance.start()

    root.mainloop()
