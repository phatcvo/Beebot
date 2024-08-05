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

int ch1Value = 0; // To store throttle value
int ch2Value = 0; // To store direction value

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

  // Initialize serial communication
  Serial.begin(115200);

  // Set receiver pins as inputs
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
}

void loop() {
  // Read the pulse width from the receiver
  ch1Value = pulseIn(CH1_PIN, HIGH, 50000); // Throttle
  ch2Value = pulseIn(CH2_PIN, HIGH, 50000); // Direction

  // Convert pulse width to motor speed and direction
  int speed = map(ch1Value, 1100, 1900, -255, 255);
  int direction = map(ch2Value, 1100, 1900, -255, 255);

  // Apply the calculated speed and direction to the motors
  controlMotors(speed, direction);

  // Print values for debugging
  Serial.print("Throttle: "); Serial.print(ch1Value);
  Serial.print(" Direction: "); Serial.println(ch2Value);

  delay(50); // Delay to avoid excessive serial printing
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
  Serial.println("Stopped");
}
