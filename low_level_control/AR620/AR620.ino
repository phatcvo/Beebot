int ch1; // Variable to store pulse width for channel 1
int ch2; // Variable to store pulse width for channel 2
int ch3; // Variable to store pulse width for channel 3
int ch4; // Variable to store pulse width for channel 4
int ch5; // Variable to store pulse width for channel 5
int ch6; // Variable to store pulse width for channel 6

void setup() {
  pinMode(5, INPUT); // Set pin 5 as input
  pinMode(6, INPUT); // Set pin 6 as input
  pinMode(7, INPUT); // Set pin 7 as input
  pinMode(8, INPUT); // Set pin 8 as input
  pinMode(9, INPUT); // Set pin 9 as input
  pinMode(10, INPUT); // Set pin 10 as input

  Serial.begin(115200); // Initialize serial communication at 115200 bps
}

void loop() {
  // Read the pulse width of each channel
  ch1 = pulseIn(5, HIGH, 50000); // Read pulse width on pin 5, timeout after 50ms
  ch2 = pulseIn(6, HIGH, 50000); // Read pulse width on pin 6, timeout after 50ms
  ch3 = pulseIn(7, HIGH, 50000); // Read pulse width on pin 7, timeout after 50ms
  ch4 = pulseIn(8, HIGH, 50000); // Read pulse width on pin 8, timeout after 50ms
  ch5 = pulseIn(9, HIGH, 50000); // Read pulse width on pin 9, timeout after 50ms
  ch6 = pulseIn(10, HIGH, 50000); // Read pulse width on pin 10, timeout after 50ms

  // Check if pulses are detected
  if (ch1 == 0 || ch2 == 0 || ch3 == 0 || ch4 == 0 || ch5 == 0 || ch6 == 0) {
    Serial.println("Pulse not detected on one or more channels.");
  } else {
    // Print the pulse widths to the serial monitor
    Serial.print("ch1: "); Serial.print(ch1);
    Serial.print(", ch2: "); Serial.print(ch2);
    Serial.print(", ch3: "); Serial.print(ch3);
    Serial.print(", ch4: "); Serial.print(ch4);
    Serial.print(", ch5: "); Serial.print(ch5);
    Serial.print(", ch6: "); Serial.println(ch6);
  }

  delay(100); // Small delay to make the serial monitor more readable
}
