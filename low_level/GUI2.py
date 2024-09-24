#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
import tkinter as tk
import threading
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class RosTkinterApp:
    def __init__(self, master):
        self.master = master
        master.title("ROS Publisher and Subscriber")

        self.label = tk.Label(master, text="Waiting for messages...")
        self.label.pack()

        self.yaw_label = tk.Label(master, text="Yaw: 0.0")
        self.yaw_label.pack()

        self.roll_label = tk.Label(master, text="Roll: 0.0")
        self.roll_label.pack()

        self.pitch_label = tk.Label(master, text="Pitch: 0.0")
        self.pitch_label.pack()

        self.imu_label = tk.Label(master, text="IMU Data:\nAcc: (0.0, 0.0, 0.0)\nGyro: (0.0, 0.0, 0.0)")
        self.imu_label.pack()

        self.data = None
        self.yaw_data = 0.0
        self.roll_data = 0.0
        self.pitch_data = 0.0
        self.imu_data = {'acc': (0.0, 0.0, 0.0), 'gyro': (0.0, 0.0, 0.0)}

        # Start the ROS node
        rospy.init_node('tkinter_ros_node', anonymous=True)

        # Publisher setup
        self.publisher = rospy.Publisher('/test_topic', String, queue_size=10)

        # Subscribe to the topics
        self.subscriber = rospy.Subscriber('/test_topic', String, self.callback)
        self.roll_subscriber = rospy.Subscriber('/roll', Float32, self.roll_callback)
        self.pitch_subscriber = rospy.Subscriber('/pitch', Float32, self.pitch_callback)
        self.yaw_subscriber = rospy.Subscriber('/yaw', Float32, self.yaw_callback)
        self.imu_subscriber = rospy.Subscriber('/imu_data', Imu, self.imu_callback)

        # Create buttons
        self.publish_hello_button = tk.Button(master, text="Publish Hello", command=self.publish_hello)
        self.publish_hello_button.pack()

        self.publish_goodbye_button = tk.Button(master, text="Publish Goodbye", command=self.publish_goodbye)
        self.publish_goodbye_button.pack()

        # Initialize 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master)
        self.canvas.get_tk_widget().pack()

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

    def roll_callback(self, msg):
        self.roll_data = msg.data  # Update roll_data when a new message arrives

    def pitch_callback(self, msg):
        self.pitch_data = msg.data  # Update pitch_data when a new message arrives

    def imu_callback(self, msg):
        # Update IMU data
        self.imu_data['acc'] = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.imu_data['gyro'] = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

    def update_gui(self):
        if self.data is not None:
            self.label.config(text=self.data)  # Update the label text
        
        # Update yaw, roll, and pitch labels
        self.yaw_label.config(text="Yaw: {:.2f}".format(self.yaw_data))
        self.roll_label.config(text="Roll: {:.2f}".format(self.roll_data))
        self.pitch_label.config(text="Pitch: {:.2f}".format(self.pitch_data))

        # Update IMU label
        acc = self.imu_data['acc']
        gyro = self.imu_data['gyro']
        self.imu_label.config(text="IMU Data:\nAcc: ({:.2f}, {:.2f}, {:.2f})\nGyro: ({:.2f}, {:.2f}, {:.2f})".format(
            acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]))

        # Update the 3D plot
        self.ax.clear()

        # Draw axes
        # self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X-axis')
        # self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y-axis')
        # self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z-axis')

        # Calculate the direction based on roll, pitch, and yaw
        R = self.rotation_matrix(self.roll_data, self.pitch_data, self.yaw_data)
        # Yaw vector
        yaw_vector = np.dot(R, np.array([1, 0, 0]))
        # Pitch vector
        pitch_vector = np.dot(R, np.array([0, 1, 0]))
        # Roll vector
        roll_vector = np.dot(R, np.array([0, 0, 1]))

        # Draw orientation vectors
        self.ax.quiver(0, 0, 0, yaw_vector[0], yaw_vector[1], yaw_vector[2], color='orange', label='Yaw Orientation')
        self.ax.quiver(0, 0, 0, pitch_vector[0], pitch_vector[1], pitch_vector[2], color='cyan', label='Pitch Orientation')
        self.ax.quiver(0, 0, 0, roll_vector[0], roll_vector[1], roll_vector[2], color='purple', label='Roll Orientation')

        # Set limits and labels
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_title('3D IMU Orientation Visualization')
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.set_zlabel('Z-axis')
        self.ax.legend()

        self.canvas.draw()

        self.master.after(100, self.update_gui)  # Call this method again after 100 ms

    def rotation_matrix(self, roll, pitch, yaw):
        # Convert angles from degrees to radians
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        # Calculate rotation matrix
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        # Combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

def ros_thread():
    rospy.spin()

if __name__ == "__main__":
    root = tk.Tk()
    app = RosTkinterApp(root)

    # Start a separate thread for ROS
    ros_thread_instance = threading.Thread(target=ros_thread)
    ros_thread_instance.start()

    root.mainloop()
