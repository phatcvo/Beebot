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

using namespace std;
class ArduinoSerial
{
public:
    ArduinoSerial();
    // ~ArduinoSerial();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber robot_state_sub;
    ros::Publisher  arduino_pub;
    // Create a serial object
    serial::Serial serialPort;
    std::string user_pwd_ = "9999,8,9,9,        ";
    
    std::string serial_port = "/dev/ttyACM0";
    int serial_baudrate = 115200;

    int door_status=1;
    int sys_btn = 0;
    int go_btn = 0;

    std::string start_marker = "<";
    std::string end_marker = ">";
    std::string delimiter = ",";
   
    void send_arduino();
    void pub_arduino_feedback(); 
    void initForROS();
    // subscribe
    void callbackFromPC(const std_msgs::StringConstPtr &msg);
};


#endif // ARDUINO_H
