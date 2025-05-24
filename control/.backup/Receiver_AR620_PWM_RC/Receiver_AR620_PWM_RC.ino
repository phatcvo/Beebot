#include <Servo.h>

// RC receiver input pins
const int THROTTLE_PIN = A2; // CH1
const int STEERING_PIN = A3; // CH2

// Servo output pins
const int SERVO_THROTTLE_PIN = 2;
const int SERVO_STEERING_PIN = 3;

// Create servo objects
Servo servoThrottle;
Servo servoSteering;

void setup() {
  // Attach servos to output pins
  servoThrottle.attach(SERVO_THROTTLE_PIN);
  servoSteering.attach(SERVO_STEERING_PIN);

  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // Read PWM pulse width from receiver
  int throttlePWM = pulseIn(THROTTLE_PIN, HIGH, 50000); // CH1
  int steeringPWM = pulseIn(STEERING_PIN, HIGH, 50000); // CH2

  // Only send values if valid pulse was read
  if (throttlePWM >= 1000 && throttlePWM <= 2000) {
    servoThrottle.writeMicroseconds(throttlePWM);
  }

  if (steeringPWM >= 1000 && steeringPWM <= 2000) {
    servoSteering.writeMicroseconds(steeringPWM);
  }

  // Debug print
  Serial.print("Throttle: ");
  Serial.print(throttlePWM);
  Serial.print(" | Steering: ");
  Serial.println(steeringPWM);

  delay(100); // 20ms is typical servo update rate
}
