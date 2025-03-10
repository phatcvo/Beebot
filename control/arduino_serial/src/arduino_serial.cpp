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
    cmd_vel_sub = private_nh_.subscribe("/cmd_vel", 10, &ArduinoSerial::callbackFromCmdVel, this);
}

void ArduinoSerial::callbackFromCmdVel(const geometry_msgs::TwistConstPtr &msg)
{
    std::stringstream ss;
    ss << msg->linear.x << "," << msg->angular.z << "," << msg->linear.z << "\n";
    cmd_vel_str = ss.str();
}

void ArduinoSerial::pub_arduino_feedback()
{
    std_msgs::Int16MultiArray m_msg;
    m_msg.data.resize(3);
    m_msg.data[0] = roll;
    m_msg.data[1] = pitch;
    m_msg.data[2] = yaw;
    printf("IMU -> Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    printf("Buttons -> sys_btn: %d, go_btn: %d, door_status: %d\n", sys_btn, go_btn, door_status);
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

    usleep(200000);
    while (ros::ok() && serialPort.isOpen())
    {
        // printf("========================= \n");
        ros::spinOnce();

        if (serialPort.available() > 0) 
        {
            size_t available_bytes = serialPort.available();
            const size_t PACKET_SIZE = 16;  // Header (1) + 3 floats (12) + 3 bytes (3)
            uint8_t buffer[PACKET_SIZE];

            if (available_bytes >= PACKET_SIZE) 
            {
                size_t bytes_read = serialPort.read(buffer, PACKET_SIZE);

                if (bytes_read == PACKET_SIZE && buffer[0] == 0xEE) 
                {
                    memcpy(&roll, &buffer[1], sizeof(float));
                    memcpy(&pitch, &buffer[5], sizeof(float));
                    memcpy(&yaw, &buffer[9], sizeof(float));

                    sys_btn = buffer[13];
                    go_btn = buffer[14];
                    door_status = buffer[15];
                }
            }
        }

        pub_arduino_feedback();
        send_arduino();
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


