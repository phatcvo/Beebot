#include <HCSR04.h>

UltraSonicDistanceSensor distanceSensor(11, 12);  // Initialize sensor that uses digital pins 13 and 12.
const int led = 13; 
const int relay = 10; 
void setup () {
    Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
    pinMode(relay, OUTPUT);
    pinMode(led, OUTPUT);
}

void loop () {
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    double distance = distanceSensor.measureDistanceCm();
    Serial.println(distance);
    if(distance < 50){
      digitalWrite(relay, HIGH);
      digitalWrite(led, HIGH);
      delay(100);
    }
    else{
      delay(5000);
      digitalWrite(relay, LOW);
      digitalWrite(led, LOW);
    }
    
}
