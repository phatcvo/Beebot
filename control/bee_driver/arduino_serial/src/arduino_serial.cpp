#include <arduino_serial.h>

ArduinoSerial::ArduinoSerial()
    :private_nh_("~")
{
    initForROS();
}

void ArduinoSerial::initForROS()
{
    ROS_INFO("=== READY!!! ===");

    private_nh_.getParam("port", serial_port);
    private_nh_.getParam("baud", serial_baudrate);
    printf("Serial port: %s\n", serial_port.c_str());
    printf("Serial baudrate: %d\n", serial_baudrate);

    arduino_pub = private_nh_.advertise<std_msgs::Int16MultiArray>("/arduino_feedback", 10);
    oled_msg_pub = private_nh_.advertise<std_msgs::String>("/oled_msg", 10);
    cmd_vel_sub = private_nh_.subscribe("/cmd_vel", 10, &ArduinoSerial::callbackFromCmdVel, this);
}

void ArduinoSerial::callbackFromCmdVel(const geometry_msgs::TwistConstPtr &msg)
{
    std::stringstream ss;
    ss << msg->linear.x << "," << -msg->angular.z << "," << msg->linear.z << "\n";
    cmd_vel_str = ss.str();
    std_msgs::String oled_msg;
    oled_msg.data = cmd_vel_str;
    oled_msg_pub.publish(oled_msg);
    float cmdVel = 5*msg->linear.x; // in rad/sec
    float cmdAgu = 5*msg->angular.z; // in rad/sec
    // printf("cmdVel: %f, cmdAgu: %f \n", cmdVel, cmdAgu);
    sendTargetVel(-(cmdVel + cmdAgu), cmdVel - cmdAgu); 
}

void ArduinoSerial::pub_arduino_feedback()
{
    std_msgs::Int16MultiArray m_msg;
    m_msg.data.resize(3);
    m_msg.data[0] = go_btn;
    m_msg.data[1] = bat_percent;
    m_msg.data[2] = (uint8_t)yaw;
    printf("IMU -> Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    printf("Battery: %d%, go_btn: %d\n", bat_percent, go_btn);
    arduino_pub.publish(m_msg);
}

void ArduinoSerial::send_arduino()
{
    printf("cmd_vel_str: %s \n", cmd_vel_str.c_str());
    serialPort.write(cmd_vel_str);
}

void ArduinoSerial::run()
{
    int rate = 10;
    ros::Rate loop_rate(rate);
    try
    {
        // Open the serial port
        serialPort.setPort(serial_port);
        serialPort.setBaudrate(serial_baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);
        serialPort.close(); 
        serialPort.open();
    }
    catch (const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open the serial port: " << e.what());
        return;
    }

    usleep(10000);
    auto prevTime = std::chrono::system_clock::now();
    while (ros::ok() && serialPort.isOpen())
    {
        // printf("========================= \n");
        ros::spinOnce();

        // if (serialPort.available() > 0) 
        // {
        //     size_t available_bytes = serialPort.available();
        //     const size_t PACKET_SIZE = 15;  // Header (1) + 3 floats (12) + 2 bytes (2)
        //     uint8_t buffer[PACKET_SIZE];

        //     if (available_bytes >= PACKET_SIZE) 
        //     {
        //         size_t bytes_read = serialPort.read(buffer, PACKET_SIZE);

        //         if (bytes_read == PACKET_SIZE && buffer[0] == 0xEE) 
        //         {
        //             memcpy(&roll, &buffer[1], sizeof(float));
        //             memcpy(&pitch, &buffer[5], sizeof(float));
        //             memcpy(&yaw, &buffer[9], sizeof(float));

        //             bat_percent = buffer[13];
        //             go_btn = buffer[14];
        //         }
        //     }
        // }

        // pub_arduino_feedback();
        // send_arduino();

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

            std::cout << "motorA_readings: [" << angPosA << std::fixed << std::setprecision(2) << "," << angVelA << std::fixed << std::setprecision(2) << "]" << std::endl;
            std::cout << "motorB_readings: [" << angPosB << std::fixed << std::setprecision(2) << "," << angVelB << std::fixed << std::setprecision(2) << "]" << '\n' << std::endl;

            prevTime = std::chrono::system_clock::now();
        }
        loop_rate.sleep();
    }

    if(!serialPort.isOpen())
    {
        ROS_ERROR("Serial port stopped working");
        serialPort.close();
        return;
    }
    
    serialPort.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduino_serial");
    ArduinoSerial rc;
    rc.run();

    return 0;
}


