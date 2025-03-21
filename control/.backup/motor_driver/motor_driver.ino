//#include "MPU9250.h"
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


float angles[3]; // yaw pitch roll

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

//MPU9250 mpu;
sensor_msgs::Imu imu_msg;
std_msgs::Float32 roll_msg, pitch_msg, yaw_msg;

ros::NodeHandle nh;
ros::Publisher imu_publisher("imu_data", &imu_msg);
ros::Publisher imu_roll_pub("roll", &roll_msg);
ros::Publisher imu_pitch_pub("pitch", &pitch_msg);
ros::Publisher imu_yaw_pub("yaw", &yaw_msg);

// Define motor control pins
const int EN_A = 7;
const int DIR_A = 8;
const int PWM_A = 9;
const int EN_B = 12;
const int DIR_B = 11;
const int PWM_B = 10;

// Define AR620 receiver pins
const int CH1_PIN = A2; // Throttle
const int CH2_PIN = A3; // Direction
const int CH5_PIN = A6; // Gear

// wheel encoder interrupts
// pin 2,3,21,20,19,18
// int 0,1,2, 3, 4, 5
#define encoder0PinA 21 // interrupt 2      // encoder 1
#define encoder0PinB 20 // interrupt 3

#define encoder1PinA 19 // interrupt 4     // encoder 2
#define encoder1PinB 18 // interrupt 5

#define AUTO 0
#define MANUAL 1

int ch1Value = 0; // To store throttle value
int ch2Value = 0; // To store direction value
int ch5Value = 0; // To store gear value

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

int act_speed;
int act_direction;
int ros_speed;
int ros_direction;

unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;
// ** ROS callback & subscriber **

void velCallback(const geometry_msgs::Twist& vel)
{
    float vel_linear = vel.linear.x;
    float vel_angular = vel.angular.z;

    vel_linear = constrain(vel_linear,-1,1);     // try to keep it under control
    vel_angular = constrain(vel_angular,-1,1);
    ros_speed = map(vel_linear, -1, 1, -255, 255);
    ros_direction = map(vel_angular, -1, 1, -255, 255);
    Serial3.print("\t vel_linear: "); Serial3.print(vel_linear);
    Serial3.print(", vel_linear: "); Serial3.println(vel_angular);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);     //create a subscriber for ROS cmd_vel topic


void setup() {
  // Set all the motor control pins to outputs
  pinMode(EN_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(EN_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  // Enable the motors
  digitalWrite(EN_A, LOW);
  digitalWrite(EN_B, LOW);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins 0
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(2, doEncoderA, CHANGE);
  attachInterrupt(3, doEncoderB, CHANGE);

  pinMode(encoder1PinA, INPUT_PULLUP);    // encoder pins 1
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(4, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE);

  // Initialize serial communication
  Serial.begin(115200);
  Serial3.begin(115200);
  // Set receiver pins as inputs
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(imu_publisher);
  nh.advertise(imu_roll_pub);
  nh.advertise(imu_pitch_pub);
  nh.advertise(imu_yaw_pub);
  
  Wire.begin();
  //  mpu.setup();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() {
  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one
  currentMillis = millis(); 
  if (currentMillis - previousMillis >= loopTime) {  // start timed loop for everything else
    previousMillis = currentMillis;

    sixDOF.getEuler(angles);

    Serial3.print(angles[0]);
    Serial3.print(" | ");  
    Serial3.print(angles[1]);
    Serial3.print(" | ");
    Serial3.println(angles[2]);
    roll_msg.data = angles[0];
    pitch_msg.data = angles[1];
    yaw_msg.data = angles[2];
    
//    mpu.update();
//    Serial3.print("roll, pitch, yaw (x, y, z): ");
//    Serial3.print(mpu.getRoll());
//    Serial3.print(", ");
//    Serial3.print(mpu.getPitch());
//    Serial3.print(", ");
//    Serial3.println(mpu.getYaw());
    
    // Fill the IMU message
    imu_msg.header.stamp = nh.now();  // Timestamp
    imu_msg.header.frame_id = "imu_link";  // Frame ID

//    // Set orientation to quaternion (replace with actual values if available)
//    imu_msg.orientation.x = mpu.getQuaternion(0);
//    imu_msg.orientation.y = mpu.getQuaternion(1);
//    imu_msg.orientation.z = mpu.getQuaternion(2);
//    imu_msg.orientation.w = mpu.getQuaternion(3);
//
//    // Set angular velocity from gyroscope data
//    imu_msg.angular_velocity.x = mpu.getGyro(0);
//    imu_msg.angular_velocity.y = mpu.getGyro(1);
//    imu_msg.angular_velocity.z = mpu.getGyro(2);
//
//    // Set linear acceleration from accelerometer data
//    imu_msg.linear_acceleration.x = mpu.getAcc(0);
//    imu_msg.linear_acceleration.y = mpu.getAcc(1);
//    imu_msg.linear_acceleration.z = mpu.getAcc(2);
//
//    roll_msg.data = mpu.getRoll();
//    pitch_msg.data = mpu.getPitch();
//    yaw_msg.data = mpu.getYaw();
    // Publish the IMU data
    imu_publisher.publish(&imu_msg);
    imu_roll_pub.publish(&roll_msg);
    imu_pitch_pub.publish(&pitch_msg);
    imu_yaw_pub.publish(&yaw_msg);
    
    // Read the pulse width from the receiver
    ch1Value = pulseIn(CH1_PIN, HIGH, 50000); // Throttle
    ch2Value = pulseIn(CH2_PIN, HIGH, 50000); // Direction
    ch5Value = pulseIn(CH5_PIN, HIGH, 50000); // Direction

    // Convert pulse width to motor speed and direction
    int rc_speed = map(ch1Value, 1100, 1900, -255, 255);
    int rc_direction = map(ch2Value, 1100, 1900, -255, 255);
    bool rc_gear = (ch5Value < 1600) ? true : false;

    // Check if the receiver is disconnected
    if (ch1Value < 1100 || ch2Value < 1100) {
        // Handle disconnection for CH1 (Throttle)
        Serial3.print("RC Disconnected! =>");
        stopMotors();
    }
    else{
      if(rc_gear == AUTO){
        Serial3.print(">>>>> Auto mode >>>>>>> \t");
        act_speed = ros_speed;
        act_direction = ros_direction;
      }
      else {
        Serial3.print(">>>>> Manual mode >>>>>>> \t");
        act_speed = rc_speed;
        act_direction = rc_direction;
        Serial3.print("Throttle: "); Serial3.print(ch1Value);
        Serial3.print(", Direction: "); Serial3.print(ch2Value);
      }
      Serial3.print("\t Speed: "); Serial3.print(act_speed);
      Serial3.print(", Direction: "); Serial3.print(act_direction);
      // Apply the calculated speed and direction to the motors
      controlMotors(act_speed, act_direction);
    }
  }
  // delay(50); // Delay to avoid excessive serial printing
}

void controlMotors(int speed, int direction) {
  int leftMotorSpeed = speed + direction;
  int rightMotorSpeed = speed - direction;

  // Constrain motor speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Set motor speeds and directions
  setMotorSpeed(DIR_A, PWM_A, leftMotorSpeed);
  setMotorSpeed(DIR_B, PWM_B, rightMotorSpeed);
  Serial3.print("\t encoder0Pos: "); Serial3.print(encoder0Pos);
  Serial3.print(", encoder1Pos: "); Serial3.println(encoder1Pos);
}

void setMotorSpeed(int dirPin, int pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -speed);
  }
}

void stopMotors() {
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  Serial3.println("Stopped");
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
