#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <string.h>
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

// yaw pitch roll
float angles[3]; 
float roll, pitch, yaw;
int sys_btn, go_btn, door_status;
// Define motor control pins
const int PWM_A = 5;
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 10;
const int IN4 = 11;
const int PWM_B = 12;

const int LED1 = A0;
const int LED2 = A1;
const int sysPin = A2;
const int goPin = A3;
// wheel encoder interrupts
// pin 2,3,21,20,19,18
// int 0,1,2, 3, 4, 5
#define encoder0PinA 2 // encoder 1
#define encoder0PinB 8 

#define encoder1PinA 3 // encoder 2
#define encoder1PinB 9

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

int cmd_speed;
int cmd_direction;
int mode;

unsigned long currentMillis;
unsigned long previousMillis;
int loopTime = 10;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  // LED
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Initial direction the motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Button
  pinMode(sysPin, INPUT);
  pinMode(goPin, INPUT);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins 0
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(3, doEncoderB, CHANGE);

  pinMode(encoder1PinA, INPUT_PULLUP);    // encoder pins 1
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE);

  // Initialize serial communication
  Serial3.begin(115200);
  Serial.begin(115200);
  
   Wire.begin();
   delay(5);
   sixDOF.init(); //begin the IMU
   delay(5);
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
  currentMillis = millis(); 
  if (currentMillis - previousMillis >= loopTime) {  // start timed loop for everything else
    previousMillis = currentMillis;

    //update IMU data
    sixDOF.getEuler(angles);

    
    Serial.print("Speed: "); Serial.print(cmd_speed);
    Serial.print(", Direction: "); Serial.print(cmd_direction);
    Serial.print(", mode: "); Serial.print(mode);  
    Serial.print("\tRoll: "); Serial.print(angles[0]);
    Serial.print(", pitch: "); Serial.print(angles[1]);
    Serial.print(", yaw: "); Serial.print(angles[2]);
    Serial.print(", sys_btn: "); Serial.print(sys_btn);
    Serial.print(", go_btn: "); Serial.print(go_btn);

    // Apply the calculated speed and direction to the motors
    controlMotors(cmd_speed, cmd_direction, mode);

    sys_btn = digitalRead(sysPin);
    go_btn = digitalRead(goPin);

    // Send data to PC
    sendIMUData(angles[0], angles[1], angles[2], sys_btn, go_btn);
  }
  delay(10);
}
// Function to send IMU & button data over Serial
void sendIMUData(float roll, float pitch, float yaw, uint8_t sys, uint8_t go) {
    uint8_t buffer[14]; // Header (1) + 3 floats (12) + 3 bytes (2)

    buffer[0] = 0xEE;  // Header for synchronization

    // Copy floats (roll, pitch, yaw) into buffer
    memcpy(&buffer[1], &roll, sizeof(float));
    memcpy(&buffer[5], &pitch, sizeof(float));
    memcpy(&buffer[9], &yaw, sizeof(float));

    // Copy button states
    buffer[13] = sys;
    buffer[14] = go;

    // Send binary data
    Serial3.write(buffer, sizeof(buffer));
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
  Serial.print(", enA: "); Serial.print(encoder0Pos);
  Serial.print(", enB: "); Serial.println(encoder1Pos);
}

void setMotorSpeed(int inPin1, int inPin2, int pwmPin, int speed, int mode) {
  digitalWrite(LED1, HIGH);
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
    analogWrite(pwmPin, 0);
    digitalWrite(LED2, HIGH);
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
