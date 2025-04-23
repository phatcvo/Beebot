#ifndef ARDUINO_H
#define ARDUINO_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include <chrono>
#include <memory>
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
class ArduinoSerial
{
public:
    ArduinoSerial();
    // ~ArduinoSerial();
    void run();

private:
    ros::NodeHandle private_nh_;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher  arduino_pub;
    ros::Publisher oled_msg_pub;
    // Create a serial object
    serial::Serial serialPort;
    
    std::string serial_port = "/dev/ttyACM0";
    int serial_baudrate = 115200;

    float roll, pitch, yaw;
    uint8_t bat_percent, go_btn;

    std::string cmd_vel_str = "";

    void send_arduino();
    void pub_arduino_feedback(); 
    void initForROS();
    // subscribe
    void callbackFromPC(const std_msgs::StringConstPtr &msg);
    void callbackFromCmdVel(const geometry_msgs::TwistConstPtr &msg);
};


#endif // ARDUINO_H
