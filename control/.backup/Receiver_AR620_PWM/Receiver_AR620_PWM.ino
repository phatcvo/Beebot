#include <Servo.h>
// Define RC channel input pins
const int CH_PINS[6] = {A0, A1, A2, A3, A4, A5};
// Servo output pins
const int SERVO_THROTTLE_PIN = 2;
const int SERVO_STEERING_PIN = 3;
// Create servo objects
Servo servoThrottle;
Servo servoSteering;
// Variables to store pulse widths and timing
unsigned long pulseWidths[6] = {0};
unsigned long lastPulseTimes[6] = {0};
float frequencies[6] = {0};

void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(CH_PINS[i], INPUT);
  }
  Serial.begin(115200);
  // Attach servos to output pins
  servoThrottle.attach(SERVO_THROTTLE_PIN);
  servoSteering.attach(SERVO_STEERING_PIN);
}

void loop() {
  for (int i = 0; i < 6; i++) {
    // Measure pulse width (us)
    unsigned long width = pulseIn(CH_PINS[i], HIGH, 60000); // timeout 60ms

    // Measure time since last valid pulse
    unsigned long now = micros();
    unsigned long interval = now - lastPulseTimes[i];
    lastPulseTimes[i] = now;

    pulseWidths[i] = width;

    // Calculate frequency
    if (interval > 0) {
      frequencies[i] = 1000000.0 / interval;
    } else {
      frequencies[i] = 0;
    }
  }

  // Print all channel readings
  for (int i = 0; i < 6; i++) {
    Serial.print("CH"); Serial.print(i + 1); Serial.print(": ");
    Serial.print(pulseWidths[i]); Serial.print(" us (");
    Serial.print(frequencies[i], 2); Serial.print(" Hz) | ");
  }
  // Only send values if valid pulse was read
  if (pulseWidths[2] >= 1000 && pulseWidths[2] <= 2000) {
    servoThrottle.writeMicroseconds(pulseWidths[2]);
  }

  if (pulseWidths[3] >= 1000 && pulseWidths[3] <= 2000) {
    servoSteering.writeMicroseconds(pulseWidths[3]);
  }
  Serial.println();

  delay(10);
}
