#include <stdio.h>
#include <string.h>

// Define motor control pins
const int DR1 = 5;//pwmA
const int PWM1 = 6;//in1
const int EN1 = 7;//in2
const int DR2 = 10;//in3
const int PWM2 = 11;//in4
const int EN2 = 12;//pwmB
 
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
const int loopTime = 10;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(PWM1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DR2, OUTPUT);

  // Initial direction the motors
  digitalWrite(EN1, LOW);
  digitalWrite(DR1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(DR2, LOW);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins 0
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  
  pinMode(encoder1PinA, INPUT_PULLUP);    // encoder pins 1
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderC, CHANGE);

  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read until newline
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
  if (currentMillis - previousMillis >= loopTime) {
    previousMillis = currentMillis;
    
    // Serial.print("Speed: "); Serial.print(cmd_speed);
    // Serial.print(", Direction: "); Serial.print(cmd_direction);
    // Serial.print(", mode: "); Serial.println(mode);  

    // Apply the calculated speed and direction to the motors
    controlMotors(cmd_speed, cmd_direction, mode);
  }
  
  
}

void controlMotors(int speed, int direction, int mode) {
  int leftMotorSpeed = speed - direction;
  int rightMotorSpeed = speed + direction;

  // Constrain motor speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Set motor speeds and directions
  setMotorSpeed(DR1, PWM1, EN1, leftMotorSpeed, mode);
  setMotorSpeed(DR2, PWM2, EN2, rightMotorSpeed, mode);
  // Serial.print(", enA: "); Serial.print(encoder0Pos);
  // Serial.print(", enB: "); Serial.println(encoder1Pos);
}

void setMotorSpeed(int drPin, int pwmPin, int enPin, int speed, int mode) {
  if (speed > 0 && mode != 0) {
    digitalWrite(drPin, HIGH);
    digitalWrite(enPin, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else if (speed < 0 && mode != 0) {
    digitalWrite(drPin, LOW);
    digitalWrite(enPin, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else {
    digitalWrite(enPin, LOW);
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
// ****** encoder 1 ******
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
