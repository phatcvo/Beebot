#include "serial_comm_cpp.hpp"

// SerialComm motorControl;
SerialComm::SerialComm() //: private_nh_("~")
{
    initForROS();
}

void SerialComm::initForROS()
{
    ROS_INFO("=== READY!!! ===");

    private_nh_.getParam("port", serial_port);
    private_nh_.getParam("baud", serial_baudrate);
    printf("Serial port: %s\n", serial_port.c_str());
    printf("Serial baudrate: %d\n", serial_baudrate);

    // oled_msg_pub = private_nh_.advertise<std_msgs::String>("/oled_msg", 10);
    cmd_vel_sub = private_nh_.subscribe("/cmd_vel", 10, &SerialComm::callbackFromCmdVel, this);

}


void SerialComm::callbackFromCmdVel(const geometry_msgs::TwistConstPtr &msg)
{
    // std::stringstream ss;
    // ss << msg->linear.x << "," << -msg->angular.z << "," << msg->linear.z << "\n";
    // std::string cmd_vel_str = ss.str();
    // std_msgs::String oled_msg;
    // oled_msg.data = cmd_vel_str;
    // oled_msg_pub.publish(oled_msg);

    float cmdVel = msg->linear.x; // in rad/sec
    float cmdAgu = msg->angular.z; // in rad/sec
    printf("cmdVel: %f, cmdAgu: %f \n", cmdVel, cmdAgu);
    sendTargetVel(cmdVel+cmdAgu, cmdVel-cmdAgu); // targetA,
}

void SerialComm::run()
{
    int rate = 10;
    ros::Rate loop_rate(rate);

    initForROS();
    float angPosA, angPosB;
    float angVelA, angVelB;

    auto prevTime = std::chrono::system_clock::now();
    std::chrono::duration<double> duration;
    float sampleTime = 0.02;

    // auto ctrlPrevTime = std::chrono::system_clock::now();
    // std::chrono::duration<double> ctrlDuration;
    // float ctrlSampleTime = 5.0;

    // std::string port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0";
    std::string port = "/dev/ttyACM0";
    connect(port);

    // // wait for the motorControl to fully setup
    // for (int i = 1; i <= 5; i += 1)
    // {
    //     delay_ms(1000);
    //     std::cout << "configuring controller: " << i << " sec" << std::endl;
    // }
    // sendTargetVel(0.0, 0.0); // targetA, targetB
    // std::cout << "configuration complete" << std::endl;

    // int motor_cmd_timeout_ms = 3000;
    // setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
    // getCmdTimeout(motor_cmd_timeout_ms);
    // std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

    // sendTargetVel(lowTargetVel, lowTargetVel); // targetA, targetB
    // sendHigh = true;

    prevTime = std::chrono::system_clock::now();
    // ctrlPrevTime = std::chrono::system_clock::now();

    while (ros::ok())
    {

        // ctrlDuration = (std::chrono::system_clock::now() - ctrlPrevTime);
        // if (ctrlDuration.count() > ctrlSampleTime)
        // {
        //     if (sendHigh)
        //     {
        //     sendTargetVel(highTargetVel, highTargetVel); // targetA, targetB
        //     sendHigh = false;
        //     }
        //     else
        //     {
        //     sendTargetVel(lowTargetVel, lowTargetVel); // targetA, targetB
        //     sendHigh = true;
        //     }

        //     ctrlPrevTime = std::chrono::system_clock::now();
        // }

        duration = (std::chrono::system_clock::now() - prevTime);
        if (duration.count() > sampleTime)
        {
            try
            {
                getMotorsPos(angPosA, angPosB); // gets angPosA, angPosB
                getMotorsVel(angVelA, angVelB); // gets angVelA, angVelB
            }
            catch (...)
            {
            // std::cout << "motorA_readings: [" << angPosA << std::fixed << std::setprecision(4) << "," << angVelA << std::fixed << std::setprecision(4) << "]" << std::endl;
            // std::cout << "motorB_readings: [" << angPosB << std::fixed << std::setprecision(4) << "," << angVelB << std::fixed << std::setprecision(4) << "]" << '\n' << std::endl;
            }

            std::cout << "motorA_readings: [" << angPosA << std::fixed << std::setprecision(4) << "," << angVelA << std::fixed << std::setprecision(4) << "]" << std::endl;
            std::cout << "motorB_readings: [" << angPosB << std::fixed << std::setprecision(4) << "," << angVelB << std::fixed << std::setprecision(4) << "]" << '\n'
                    << std::endl;

            prevTime = std::chrono::system_clock::now();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_comm");
    SerialComm rc;
    rc.run();

    return 0;
}