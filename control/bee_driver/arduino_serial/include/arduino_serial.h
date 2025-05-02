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
    bool sendTargetVel(float valA = 0.0, float valB = 0.0)
    {
        return send("/tag", valA, valB);
    }

    bool sendPwm(int valA = 0, int valB = 0)
    {
        return send("/pwm", valA, valB);
    }

    bool setCmdTimeout(int timeout_ms)
    {
        return send("/timeout", timeout_ms, 0);
    }

    void getCmdTimeout(int &timeout_ms)
    {
        get("/timeout");

        timeout_ms = val[0];

        val[0] = 0.0;
        val[1] = 0.0;
    }

    void getMotorsPos(float &angPosA, float &angPosB)
    {
        get("/pos");

        angPosA = val[0];
        angPosB = val[1];

        val[0] = 0.0;
        val[1] = 0.0;
    }

    void getMotorsVel(float &angVelA, float &angVelB)
    {
        get("/vel");

        angVelA = val[0];
        angVelB = val[1];

        val[0] = 0.0;
        val[1] = 0.0;
    }

    void getMotorAData(float &angPos, float &angVel)
    {
        get("/dataA");

        angPos = val[0];
        angVel = val[1];

        val[0] = 0.0;
        val[1] = 0.0;
    }

    void getMotorBData(float &angPos, float &angVel)
    {
        get("/dataB");

        angPos = val[0];
        angVel = val[1];

        val[0] = 0.0;
        val[1] = 0.0;
    }

    void getMotorAMaxVel(float &maxVel)
    {
        get("/maxVelA");

        maxVel = val[0];

        val[0] = 0.0;
        val[1] = 0.0;
    }

    void getMotorBMaxVel(float &maxVel)
    {
        get("/maxVelB");
        
        maxVel = val[0];

        val[0] = 0.0;
        val[1] = 0.0;
    }

private:
    ros::NodeHandle private_nh_;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher arduino_pub;
    ros::Publisher oled_msg_pub;
    // Create a serial object
    serial::Serial serialPort;
    
    std::string serial_port = "/dev/ttyACM0";
    int serial_baudrate = 115200;
    int timeout_ms_;
    float val[2];
    float roll, pitch, yaw;
    uint8_t bat_percent, go_btn;

    float angPosA, angPosB;
    float angVelA, angVelB;

    std::chrono::duration<double> duration;
    float sampleTime = 0.02;
    std::string cmd_vel_str = "";

    void send_arduino();
    void pub_arduino_feedback(); 
    void initForROS();
    // subscribe
    void callbackFromPC(const std_msgs::StringConstPtr &msg);
    void callbackFromCmdVel(const geometry_msgs::TwistConstPtr &msg);

    std::string send_msg(const std::string &msg_to_send)
    {
        auto prev_time = std::chrono::system_clock::now();
        std::chrono::duration<double> duration;

        std::string response = "";

        while (response == "")
        {
            try
            {
                try
                {
                    serialPort.write(msg_to_send);
                    response = serialPort.readline(1024, "\n");
                    duration = (std::chrono::system_clock::now() - prev_time);
                }
                catch (...)
                {
                    continue;
                }

                duration = (std::chrono::system_clock::now() - prev_time);
                if (duration.count() > 2.0)
                {
                    throw duration.count();
                }
            }
            catch (double x)
            {
                std::cerr << "Error getting response from arduino nano, wasted much time \n";
            }
        }

        return response;
    }

    bool send(std::string cmd_route, float valA, float valB)
    {
        std::stringstream msg_stream;
        msg_stream << cmd_route << "," << valA << "," << valB;

        std::string res = send_msg(msg_stream.str());
        if(!res.empty()){
            int data = std::stoi(res);
            if (data)
                return true;
            else
                return false;
        }
    }

    void get(std::string cmd_route)
    {
        std::string res = send_msg(cmd_route);

        std::stringstream ss(res);
        std::vector<std::string> v;

        while (ss.good())
        {
            std::string substr;
            getline(ss, substr, ',');
            v.push_back(substr);
        }

        for (size_t i = 0; i < v.size(); i++)
        {
            val[i] = std::atof(v[i].c_str());
        }
    }
};


#endif // ARDUINO_H
