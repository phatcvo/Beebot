#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>  // Serial library for communication

serial::Serial ser;  // Declare serial object
ros::Publisher vel_pub;

void readSerialAndPublish() {
    if (ser.available()) {
        std::string result = ser.readline();  // Read the serial input
        std::istringstream iss(result);
        float linear_x, angular_z;
        
        if (iss >> linear_x) {
            iss.ignore();  // Ignore comma
            if (iss >> angular_z) {
                geometry_msgs::Twist msg;
                msg.linear.x = linear_x;
                msg.angular.z = angular_z;
                vel_pub.publish(msg);  // Publish to ROS topic
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh;

    // Initialize serial communication
    ser.setPort("/dev/ttyACM0");  // Specify the correct serial port
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);

    try {
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(10);  // Set rate for publishing messages (10 Hz)
    while (ros::ok()) {
        readSerialAndPublish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}