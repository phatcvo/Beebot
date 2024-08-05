#include <Encoder.h>

// Define motor control pins
const int motorA_EN = 10;
const int motorA_IN1 = 8;
const int motorA_IN2 = 9;
const int motorB_EN = 13;
const int motorB_IN1 = 11;
const int motorB_IN2 = 12;

// Define encoder pins
const int encA1 = 2;
const int encA2 = 3;
const int encB1 = 4;
const int encB2 = 5;

// Define receiver input pins
const int ch1Pin = 6;  // Channel 1 (Throttle)
const int ch2Pin = 7;  // Channel 2 (Steering)

// Create encoder objects
Encoder motorAEncoder(encA1, encA2);
Encoder motorBEncoder(encB1, encB2);

// PID constants
const double Kp = 1.0;
const double Ki = 0.1;
const double Kd = 0.01;

// Target speed in encoder counts per loop
const double targetSpeed = 100.0;

double previousError = 0;
double integral = 0;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(motorA_EN, OUTPUT);
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);

  // Set receiver pins as input
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);

  Serial.begin(115200); // Initialize serial communication at 115200 bps
}

void loop() {
  // Read receiver inputs
  int ch1 = pulseIn(ch1Pin, HIGH, 50000); // Throttle
  int ch2 = pulseIn(ch2Pin, HIGH, 50000); // Steering

  // Map receiver input to motor speed and direction
  int throttle = map(ch1, 1000, 2000, -255, 255);  // Map to range -255 to 255
  int steering = map(ch2, 1000, 2000, -255, 255);  // Map to range -255 to 255

  // Calculate target speed for each motor
  int targetSpeedA = 100;//throttle + steering;
  int targetSpeedB = 100;//throttle - steering;

  // PID control to keep the tank moving straight
  static long lastMotorAPos = 0;
  static long lastMotorBPos = 0;

  long motorAPos = motorAEncoder.read();
  long motorBPos = motorBEncoder.read();

  long deltaMotorA = motorAPos - lastMotorAPos;
  long deltaMotorB = motorBPos - lastMotorBPos;

  double speedA = deltaMotorA / (double)targetSpeed;
  double speedB = deltaMotorB / (double)targetSpeed;

  double error = speedA - speedB;

  integral += error;
  double derivative = error - previousError;
  double correction = Kp * error + Ki * integral + Kd * derivative;

  int speedAOutput = constrain(targetSpeedA - correction, -255, 255);
  int speedBOutput = constrain(targetSpeedB + correction, -255, 255);

  moveMotors(speedAOutput, speedBOutput);

  previousError = error;
  lastMotorAPos = motorAPos;
  lastMotorBPos = motorBPos;

  // Print the encoder values for debugging
  Serial.print("Motor A Position: ");
  Serial.print(motorAPos);
  Serial.print(" Motor B Position: ");
  Serial.println(motorBPos);

  delay(100); // Adjust delay as necessary
}

void moveMotors(int speedA, int speedB) {
  // Control Motor A
  if (speedA > 0) {
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    analogWrite(motorA_EN, speedA);
  } else {
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, HIGH);
    analogWrite(motorA_EN, -speedA);
  }

  // Control Motor B
  if (speedB > 0) {
    digitalWrite(motorB_IN1, HIGH);
    digitalWrite(motorB_IN2, LOW);
    analogWrite(motorB_EN, speedB);
  } else {
    digitalWrite(motorB_IN1, LOW);
    digitalWrite(motorB_IN2, HIGH);
    analogWrite(motorB_EN, -speedB);
  }
}

void stop() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, 0);

  digitalWrite(motorB_IN1, LOW);
  digitalWrite(motorB_IN2, LOW);
  analogWrite(motorB_EN, 0);
}
