#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include "ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

sensor_msgs::Imu imu_msg;
std_msgs::Float32 roll_msg, pitch_msg, yaw_msg;

ros::NodeHandle nh;
ros::Publisher imu_publisher("imu_data", &imu_msg);
ros::Publisher imu_roll_pub("roll", &roll_msg);
ros::Publisher imu_pitch_pub("pitch", &pitch_msg);
ros::Publisher imu_yaw_pub("yaw", &yaw_msg); 

// yaw pitch roll
float angles[3]; 

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

unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;
// ** ROS callback & subscriber **

void velCallback(const geometry_msgs::Twist& vel)
{
    float vel_linear = vel.linear.x;
    float vel_angular = vel.angular.z;

    
    // Keep values within -1 to 1 range
    vel_linear = constrain(vel_linear, -1.0, 1.0);
    vel_angular = constrain(vel_angular, -1.0, 1.0);
    
    // Map to motor control range
    cmd_speed = (int)map(vel_linear * 100, -100, 100, -255, 255);
    cmd_direction = (int)map(vel_angular * 100, -100, 100, -255, 255);
    Serial3.print("ROS vel_linear: "); Serial3.print(vel_linear);
    Serial3.print(", vel_linear: "); Serial3.println(vel_angular);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel" , velCallback);

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

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(imu_publisher);
  nh.advertise(imu_roll_pub);
  nh.advertise(imu_pitch_pub);
  nh.advertise(imu_yaw_pub);
  
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() {
  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one
  currentMillis = millis(); 
  if (currentMillis - previousMillis >= loopTime) {  // start timed loop for everything else
    previousMillis = currentMillis;

    //update IMU data
    sixDOF.getEuler(angles);
    
    imu_msg.header.stamp = nh.now();  // Timestamp
    imu_msg.header.frame_id = "imu_link";  // Frame ID

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

    roll_msg.data = angles[0];
    pitch_msg.data = angles[1];
    yaw_msg.data = angles[2];
    
    // Publish the IMU data
//    imu_publisher.publish(&imu_msg);
//    imu_roll_pub.publish(&roll_msg);
//    imu_pitch_pub.publish(&pitch_msg);
//    imu_yaw_pub.publish(&yaw_msg);

//    Serial3.print("Yaw: "); Serial3.print(angles[2]);
    Serial3.print("Speed: "); Serial3.print(cmd_speed);
    Serial3.print(", Direction: "); Serial3.print(cmd_direction);
    // Apply the calculated speed and direction to the motors
    controlMotors(cmd_speed, cmd_direction);
  }else{
      cmd_speed = 0;
      cmd_direction = 0;
  }
}

void controlMotors(int speed, int direction) {
  int leftMotorSpeed = speed + direction;
  int rightMotorSpeed = speed - direction;

  // Constrain motor speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Set motor speeds and directions
  setMotorSpeed(IN1, IN2, PWM_A, leftMotorSpeed);
  setMotorSpeed(IN4, IN3, PWM_B, rightMotorSpeed);
  Serial3.print(", encoder0Pos: "); Serial3.print(encoder0Pos);
  Serial3.print(", encoder1Pos: "); Serial3.println(encoder1Pos);
}

void setMotorSpeed(int inPin1, int inPin2, int pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
    analogWrite(pwmPin, abs(speed));
  } else if (speed < 0) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
    Serial3.print(" => Stopped");
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
