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

    robot_state_sub = private_nh_.subscribe("/robot_state", 10, &ArduinoSerial::callbackFromPC, this);
    arduino_pub = private_nh_.advertise<std_msgs::Int16MultiArray>("/arduno_feedback", 10);
}
void ArduinoSerial::callbackFromPC(const std_msgs::StringConstPtr &msg)
{
	if(msg->data.length() != 19)
		return;
#if 0
	std::string pw_data = msg->data.substr(0,4);
	if(pw_data != user_pwd_)
	{
		user_pwd_ = pw_data;
		printf("user_pwd_: %s\n", user_pwd_.c_str());
	}
#else
	if(msg->data != user_pwd_)
	{
		user_pwd_ = msg->data;
		printf("user_pwd_: %s\n", user_pwd_.c_str());
	}
#endif
}
void ArduinoSerial::pub_arduino_feedback()
{
    std_msgs::Int16MultiArray m_msg;
    m_msg.data.resize(3);
    m_msg.data[0] = sys_btn;
    m_msg.data[1] = go_btn;
    m_msg.data[2] = door_status;

    arduino_pub.publish(m_msg);
}

void ArduinoSerial::send_arduino()
{
    string message_send = "";
    message_send += start_marker;
    message_send += user_pwd_;
    // message_send += delimiter;
    // message_send += "robot_state";
    message_send += end_marker;

    printf("message_send: %s \n", message_send.c_str());
    serialPort.write(message_send);
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

        if (serialPort.available())
        {

            const size_t buffer_size = serialPort.available();
            uint8_t buffer[buffer_size];
            size_t bytes_read = serialPort.read(buffer, buffer_size);

            int start_idx = -1;
            for (size_t i = 0; i < bytes_read; ++i)
            {
                // ROS_INFO("Received byte: 0x%.2X", buffer[i]);
                if(buffer[i]==0xEE)
                {
                    start_idx = i;
                }
            }

            if(start_idx != -1 && bytes_read>=start_idx+4 && buffer[start_idx]==0xEE)
            {
                sys_btn = buffer[start_idx+1];
                go_btn = buffer[start_idx+2];
                door_status = buffer[start_idx+3];
                printf("sys_btn: %d, go_btn: %d, door_status: %d \n", sys_btn, go_btn, door_status);
                
            }
            else
            {
                // printf("dmmmmmmmmm fault, start_idx: %d \n", start_idx);
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


