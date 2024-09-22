#include <ros.h>
#include <geometry_msgs/Twist.h>

// Create a ROS node handle
ros::NodeHandle nh;

// Define a callback function for the cmd_vel subscriber
void cmdVelCallback(const geometry_msgs::Twist& msg) {
    float linear_x = msg.linear.x;  // Get linear velocity
    float angular_z = msg.angular.z; // Get angular velocity

    // Implement your motor control logic here
    Serial.print("Linear X: ");
    Serial.println(linear_x);
    Serial.print("Angular Z: ");
    Serial.println(angular_z);
    Serial3.print("Linear X: ");
    Serial3.println(linear_x);
    Serial3.print("Angular Z: ");
    Serial3.println(angular_z);

    // Example: Control motors based on received velocities
    // Replace these with your actual motor control functions
    controlMotors(linear_x, angular_z);
}

// Function to control motors (implement your logic here)
void controlMotors(float linear, float angular) {
    // Example control logic
    int speed = linear * 255; // Scale linear speed to PWM range
    int turn = angular * 255; // Scale angular speed to PWM range

    // Replace with your motor control functions
    // e.g., setMotorSpeed(speed, turn);
}

// Create a subscriber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial3.begin(115200);
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce(); // Process incoming messages
    delay(100);    // Adjust as necessary
}
