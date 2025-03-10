// #include <Wire.h>
// #include <FreeSixIMU.h>
// #include <FIMU_ADXL345.h>
// #include <FIMU_ITG3200.h>

// #include "ros.h"
// #include <std_msgs/String.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Float32.h>
// #include <geometry_msgs/Twist.h>
// #include <ros/time.h>
#include <string.h>
// Set the FreeSixIMU object
// FreeSixIMU sixDOF = FreeSixIMU();

// sensor_msgs::Imu imu_msg;
// std_msgs::Float32 roll_msg, pitch_msg, yaw_msg;

// ros::NodeHandle nh;
// ros::Publisher imu_publisher("imu_data", &imu_msg);
// ros::Publisher imu_roll_pub("roll", &roll_msg);
// ros::Publisher imu_pitch_pub("pitch", &pitch_msg);
// ros::Publisher imu_yaw_pub("yaw", &yaw_msg); 

// yaw pitch roll
float angles[3]; 
float roll, pitch, yaw;
int sys_btn, go_btn, door_status;
// Define motor control pins
const int IN1 = 7;
const int IN2 = 8;
const int PWM_A = 5;
const int IN3 = 11;
const int IN4 = 12;
const int PWM_B = 6;

// wheel encoder interrupts
// pin 2,3,21,20,19,18
// int 0,1,2, 3, 4, 5
#define encoder0PinA 2 // interrupt 2      // encoder 1
#define encoder0PinB 9 // interrupt 3

#define encoder1PinA 3 // interrupt 4     // encoder 2
#define encoder1PinB 10 // interrupt 5

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

int cmd_speed;
int cmd_direction;
int mode;

unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;
// ** ROS callback & subscriber **

// void velCallback(const geometry_msgs::Twist& vel)
// {
//     float vel_linear = vel.linear.x;
//     float vel_angular = vel.angular.z;

    
//     // Keep values within -1 to 1 range
//     vel_linear = constrain(vel_linear, -1.0, 1.0);
//     vel_angular = constrain(vel_angular, -1.0, 1.0);
    
//     // Map to motor control range
//     cmd_speed = (int)map(vel_linear * 100, -100, 100, -255, 255);
//     cmd_direction = (int)map(vel_angular * 100, -100, 100, -255, 255);
//     Serial3.print("ROS vel_linear: "); Serial3.print(vel_linear);
//     Serial3.print(", vel_linear: "); Serial3.println(vel_angular);
// }

// ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel" , velCallback);

void setup() {
  // Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  // Initial direction the motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins 0
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(3, doEncoderB, CHANGE);

  pinMode(encoder1PinA, INPUT_PULLUP);    // encoder pins 1
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE);

  // Initialize serial communication
  Serial.begin(115200);
  Serial3.begin(115200);

//   nh.initNode();
// //  nh.subscribe(sub);
//   nh.advertise(imu_publisher);
//   nh.advertise(imu_roll_pub);
//   nh.advertise(imu_pitch_pub);
//   nh.advertise(imu_yaw_pub);
  
//   Wire.begin();
//   delay(5);
//   sixDOF.init(); //begin the IMU
//   delay(5);
}

void loop() {
  if (Serial3.available()) {
    String input = Serial3.readStringUntil('\n');  // Read until newline
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1); // Find second comma

    if (firstComma > 0 && secondComma > firstComma) {
        float vel_linear = input.substring(0, firstComma).toFloat();
        float vel_angular = input.substring(firstComma + 1, secondComma).toFloat();
        mode = input.substring(secondComma + 1).toFloat();

        // Keep values within -1 to 1 range
        vel_linear = constrain(vel_linear, -1.0, 1.0);
        vel_angular = constrain(vel_angular, -1.0, 1.0);
        mode = constrain(mode, 0, 1);

        // Map to motor control range
        cmd_speed = (int)map(vel_linear * 100, -100, 100, -255, 255);
        cmd_direction = (int)map(vel_angular * 100, -100, 100, -255, 255);
    }
  }
  // nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one
  currentMillis = millis(); 
  if (currentMillis - previousMillis >= loopTime) {  // start timed loop for everything else
    previousMillis = currentMillis;

    //update IMU data
    // sixDOF.getEuler(angles);
    
    // imu_msg.header.stamp = nh.now();  // Timestamp
    // imu_msg.header.frame_id = "imu_link";  // Frame ID

//    // Set orientation to quaternion
//    imu_msg.orientation.x = mpu.getQuaternion(0);
//    imu_msg.orientation.y = mpu.getQuaternion(1);
//    imu_msg.orientation.z = mpu.getQuaternion(2);
//    imu_msg.orientation.w = mpu.getQuaternion(3);
//    // Set angular velocity from gyroscope data
//    imu_msg.angular_velocity.x = mpu.getGyro(0);
//    imu_msg.angular_velocity.y = mpu.getGyro(1);
//    imu_msg.angular_velocity.z = mpu.getGyro(2);
//    // Set linear acceleration from accelerometer data
//    imu_msg.linear_acceleration.x = mpu.getAcc(0);
//    imu_msg.linear_acceleration.y = mpu.getAcc(1);
//    imu_msg.linear_acceleration.z = mpu.getAcc(2);

    // roll_msg.data = angles[0];
    // pitch_msg.data = angles[1];
    // yaw_msg.data = angles[2];
    
    // Publish the IMU data
//    imu_publisher.publish(&imu_msg);
//    imu_roll_pub.publish(&roll_msg);
//    imu_pitch_pub.publish(&pitch_msg);
//    imu_yaw_pub.publish(&yaw_msg);

//    Serial3.print("Yaw: "); Serial3.print(angles[2]);
    Serial.print("Speed: "); Serial.print(cmd_speed);
    Serial.print(", Direction: "); Serial.print(cmd_direction);
    Serial.print(", mode: "); Serial.print(mode);
    // Apply the calculated speed and direction to the motors
    controlMotors(cmd_speed, cmd_direction, mode);

    // Simulated IMU updates
    roll += 0.1; pitch += 0.2; yaw += 0.3;
    if (roll > 360) roll = 0;
    if (pitch > 360) pitch = 0;
    if (yaw > 360) yaw = 0;

    // Simulated button state updates (randomly changing)
    sys_btn = random(0, 2);
    go_btn = random(0, 2);
    door_status = random(0, 2);

    // Send data to PC
    sendIMUData(roll, pitch, yaw, sys_btn, go_btn, door_status);
  }
  delay(10);
}
// Function to send IMU & button data over Serial
void sendIMUData(float roll, float pitch, float yaw, uint8_t sys, uint8_t go, uint8_t door) {
    uint8_t buffer[15]; // Header (1) + 3 floats (12) + 3 bytes (3)

    buffer[0] = 0xEE;  // Header for synchronization

    // Copy floats (roll, pitch, yaw) into buffer
    memcpy(&buffer[1], &roll, sizeof(float));
    memcpy(&buffer[5], &pitch, sizeof(float));
    memcpy(&buffer[9], &yaw, sizeof(float));

    // Copy button states
    buffer[13] = sys;
    buffer[14] = go;
    buffer[15] = door;

    // Send binary data
    Serial.write(buffer, sizeof(buffer));
}
void controlMotors(int speed, int direction, int mode) {
  int leftMotorSpeed = speed + direction;
  int rightMotorSpeed = speed - direction;

  // Constrain motor speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Set motor speeds and directions
  setMotorSpeed(IN1, IN2, PWM_A, leftMotorSpeed, mode);
  setMotorSpeed(IN4, IN3, PWM_B, rightMotorSpeed, mode);
  Serial.print(", encoder0Pos: "); Serial.print(encoder0Pos);
  Serial.print(", encoder1Pos: "); Serial.println(encoder1Pos);
}

void setMotorSpeed(int inPin1, int inPin2, int pwmPin, int speed, int mode) {
  if (speed > 0 && mode != 0) {
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
    analogWrite(pwmPin, abs(speed));
  } else if (speed < 0 && mode != 0) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
    Serial.print(" => Stopped");
  }
}

// ****** encoder 0 ******

void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ****** encoder 1 ******

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
  

}
