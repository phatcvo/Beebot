In part 1 of the project, you understood the **basic motion control** of your wheeled mobile robot and you have **calibrated** it for **precise distance, turn angle, and variable linear and rotational speeds**. In labsson 9, you learned about **ultrasonic sensors** and you **calibrated** and **denoised** your own ultrasonic sensor. 

In this part, you will use your **knowledge of ultrasonics sensors** and the **basic motion control of the robot** to give the **obstacle detection** and **avoidance** ability to your mobile robot. We will use a **servo motor** to turn/sweep the ultrasonics sensor, so let's start by learning about servo motors first. 

## Introduction to servo motors

So far, we have used **brushed DC motors**, but there's a class of **commercial DC motors called servo motors** that have their own **control circuit built-in**. We are going to introduce that part now.

### Theory behind Servo Motors

A **servo motor** is a rotary or linear actuator that allows for **precise control** of angular or linear position, velocity, and acceleration. Servo motors are widely used in robotics, automation, and model-making due to their ability to provide accurate and repeatable motion.

**Servo motors** typically consist of a **DC motor**, a **gear train**, and a **feedback control system**. The feedback system includes a **potentiometer** or an **encoder** that provides **position feedback** to the **motor controller**, allowing it to adjust the motor's rotation to achieve the desired position. The **potentiometer** inside the servo motor is **connected to the output shaft of the servo**. As the **servo gear rotates, so does the shaft of the potentiometer**. This rotation **changes the resistance of the potentiometer**, which in turn alters the voltage at the potentiometer's wiper (output). **The voltage at the wiper of the potentiometer provides a direct representation of the shaft's position. This voltage is read by the servo's control circuitry.**

<figure>
<p align="center">
<img width="522" alt="small_servo_motor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/aae38a48-c35c-4e76-8de9-8cf6ea4183e6">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The **servo motor's control circuit** compares the **actual position of the servo** (as indicated by the **potentiometer's voltage**) to the **desired position** (**as indicated by the PWM signal sent to the servo**). **The desired position is encoded in the width of the PWM pulse that is applied to the servo motor.**

If there is a **discrepancy between the actual position (from the potentiometer) and the desired position (from the PWM signal)**, the control circuit adjusts the **power supplied to the motor** to **move the shaft to the correct position**. 

<figure>
<p align="center">
<img width="522" alt="servo_motor_control_circuit" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/56c4d216-c141-41bd-9fe9-4d6b7603e817">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### How Servo Motors Interpret the Signal

Servo motors interpret control signals in the form of **PWM (Pulse Width Modulation) signals**. The PWM signal consists of a repeating pulse (where we have studied before) where the **width of the pulse determines** the position of the servo motor's shaft:

- A **pulse width of 1.5 milliseconds** typically corresponds to the **center position of the servo motor** (90 deg).
- A pulse width **less than 1.5 milliseconds** rotates the shaft in one direction (e.g., **counterclockwise**).
- A pulse width **greater than 1.5 milliseconds** rotates the shaft in the **opposite direction** (e.g., **clockwise**).

<figure>
<p align="center">
<img width="522" alt="PWM_and_servo_rotation" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/fcac74dd-b9c8-4f84-add2-b3534e88cefb">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### Using Servo Motors with Arduino

Using **small servo motors** with Arduino is pretty straightforward (Arduino can provide the current requirement of a small servo motor), all you need to do is **include the IDE's built-in servo library**, attach a specific **PWM pin to the servo's signal wire** and **write the desired angle to the motor**. This can be done as follows:

- First **wire** your servo motor to Arduino. Make sure to connect the **signal wire to a PWM pin**, and two others to GND and 5v pins of the Arduino. 
- Then upload the following code to your Arduino board and explain what happens:

```cpp
#include <Servo.h>

// Create a servo object
Servo servoMotor;

// Define the servo pin
// Note that the pin should be one of Arduino's PWM pins
const int servoPin = 9;

void setup() {
  // Attach the servo to the pin
  servoMotor.attach(servoPin);
}

void loop() {
  // Move the servo to the 0-degree position
  servoMotor.write(0);
  delay(1000); // Wait for 1 second
  
  // Move the servo to the 90-degree position
  servoMotor.write(90);
  delay(1000); // Wait for 1 second
  
  // Move the servo to the 180-degree position
  servoMotor.write(180);
  delay(1000); // Wait for 1 second
}
```
- Now, choose the `Sweep` code from the **Examples in the Arduino IDE** and upload it to your Arduino board to see how the servo motor can be programmed to move back and forth across its range automatically. 

## Obstacle Avoidance Using a Rotating Ultrasonic Sensor

We need to create a code for wandering around with the mobile robot without hitting obstacles. This requires creating separate modular functions: **car motion module** (that you already implemented in part 1), **ultrasonic module** (which is largely done in labsson 9), **servo scan module**, **setup module** and **loop module**.

### Definitions and Global Variables

Before starting the obstacle avoidance project, first include pin declarations and initiations (you can do this as you go along and when need arises as well). This is how my definition of variables look like (**note that I write quick codes just to quickly test that things are working and they are neither optimized for performance nor intended for use; they are simply meant to provide a functional demonstration or a proof of concept. So do not copy and just get inspiration on where to start**). 

```cpp
#include <Servo.h> 
 
// Driving settings 
int AIN1 = 7; 
int BIN1 = 8;  
int PWMA = 5; 
int PWMB = 6; 
int STBY = 3;  
 
// Scanner settings 
Servo myservo; // servo object 
const int servoPin = 10; // servo pin (look at where the wires from servo go on the shield)
int rotation_delay = 2; // Servo scanning delay in milliseconds 
const int echoPin = 12; // Echo pin connected (verify this by checking the pins on the shield)
const int trigPin = 13; // Trig pin connected (verify this by checking the pins on the shield)
const float m = [include your own from calibration]; // Linear fit slope (following calibration of ultrasonic sensor) 
const float b = [include your own from calibration]; // Linear fit offset (following calibration of ultrasonic sensor) 
const float max_dist = 60; // Maximum detectable distance by ultrasonic sensor (in cm for my robot). Make sure to adjust this. 
```

### Setup Module

The setup module sets the pin modes, initializes the car state to be stationary and brings the servo motor to the default position at 90 degrees (facing forward direction), as follows (**again this is a quick code to just get inspiration from**):

```cpp
void setup() { 
  // Driving setup 
  pinMode(AIN1,OUTPUT);  
  pinMode(BIN1,OUTPUT);  
  pinMode(PWMA,OUTPUT);  
  pinMode(PWMB,OUTPUT);  
  pinMode(STBY,OUTPUT); 
  digitalWrite(STBY,HIGH); 
  stopCar(); 
 
  // Scanner setup 
  myservo.attach(servoPin); // Attach the servo to the pin 
  myservo.write(90);  // Default servo position which is the forward direction
  pinMode(trigPin, OUTPUT); // Attach ultrasonic trigger pin 
  pinMode(echoPin, INPUT); // Attach ultrasonic echo pin  
} 
```

### Car Motion Module

You have done this module in part 1 so make sure that you have **functions for forward motion**, **backward motion**, **left motion**, **right motion** and **stopping**. Check the "some important tips" at the end of this guide to make necessary changes to these functions based on possible challenges. 

### Ultrasonic Module

We have learnt how to use the ultrasonic sensor in labsson 9, starting with calibration and ending with smoothening. **Now turn the code that you wrote for that labsson into a function**, where the function should `return` the sensor reading to the main loop function. The **calibrated sensor reading will be **good enough** for turning around obstacles**. The skeleton of this function is something like following:

```cpp
float Ultrasonic() {

//include your calculations for calibrated distance here 

  return calibrated_distance;
}
```

### Servo Scanning Module

The car is equipped with an **ultrasonic sensor attached to a servo motor**. We need to use these utilities for **decision making** (**in part 1, you made decisions and now it's time for the robot to be able to make decisions**). **The car should keep moving forward till an obstacle appears within the sensor's range (the sensor is initially looking forward)**. If an **obstacle is detected**, the **car should stop** and the **servo motor performs a complete 180 degrees scan of the area**. The direction with the **furthest obstacle** should be the **optimal direction** to turn to. Each direction is marked with an angle, and therefore the function should return the optimal angle associated with the optimal direction. One thing to keep in mind is that, if multiple directions have close readings, the function should consider the direction closest to the current forward direction to avoid wasting energy and time on turning around.

**I again wrote a quick code below for this part but this code is not optimal**. **Get inspiration from this code and make it more optimal**: 

```cpp
int ServoScan() { 
  // Return the servo to zero position 
  myservo.write(0); delay(10); 
 
  // Map the area till you find a direction with the furthest obstacle ahead 
  // the step size here is 10. play around with it to find an optimal step size
  int angle; int optimal_angle = 0; float optimal_dist = 0; 
  for (angle = 0; angle <= 180; angle += 10) { 
    // Rotate 
    myservo.write(angle); // Send the new angular command 
    delay(rotation_delay); // Adjust the delay for the desired speed 
 
    // Scan the direction and declare new distance as optimal if met with least obstacles 
    float dist = Ultrasonic(); // Capture a new distance reading 
    if (dist > optimal_dist || (dist==optimal_dist && abs(angle-90)<abs(optimal_angle-90))){ 
        optimal_dist = dist; // The new discovered distance is optimal distance 
        optimal_angle = angle;  
    } 
  } 
  myservo.write(90);  // Back to default servo position 
  return optimal_angle; 
}
```

### Main Module

The main module is the `void loop` function. As explained before, the car would continuously monitor the ultrasonic sensor's output. If the output indicates an obstacle, the car would stop, **call** the `ServoScan` function and turn around to achieve the optimal angle. Following the turning around period, the car would move forward again. The following is an skeleton code to start with. Note that you should change this code and make it more optimal. The **numbers for distance, angle, and speed should also change based on your calibration (they are random numbers)**:

```cpp
void loop() { 
  // Mission code (runs continuously) 
  float dist = Ultrasonic(); 
 
  // Check the distance and behave accordingly 
  if (dist > max_dist) { 
    // No obstacles detected 
    forward(25, 10); // Keep moving forward at your designated speed. Play around with this number to find an optimal number  
  } else { 
    stopCar(); // Stop till a decision is made 
    int steering_angle = ServoScan(); // Scan the area and get optimal steering angle 
    if (steering_angle <= 90){ 
      rightTurn(abs(steering_angle-90), 250); // rotate to the desired steering angle at your designated angular speed. 
    } else { 
      leftTurn(abs(steering_angle-90), 250); 
    } 
    delay(10); // stop 0.1 seconds before moving forward again 
  } 
}
```
## Some Important Tips

**You may see that your robot does not respond promptly to obstacles**. If you have this issue, you can do two things: 

- Instead of moving a large distance at once, you could make the robot move shorter distances and check for obstacles more frequently. This allows the robot to react more promptly.
- Modify the `forward` function to intermittently check for obstacles as it moves. This can be done by incorporating checks after moving a small fraction of the desired distance. Add this to the end of your `forward` function: 

```cpp
    // Explanation: This line calculates the total time (in milliseconds) it should take for the 
    // robot to move the specified distance at the given speed.
    long time_to_move = distance / speed * 1000.0; // Total time to move the specified distance
     // the following line captures the start of the movement 
    // This records the current time (in milliseconds since the program started) 
    // right before the robot begins to move. This timestamp is used as 
    // a reference point to measure elapsed time.
    long start_time = millis();
    // the following line defines how often (in milliseconds) the robot should check 
    // for obstacles in its path. A shorter interval means more frequent checks. 
    long check_interval = 10; // Time interval to check for obstacles in milliseconds. 
   // This while loop runs as long as the elapsed time since start_time is less than time_to_move. 
   // It ensures the robot continues its intended movement for the calculated duration unless interrupted by detecting an obstacle.
    while (millis() - start_time < time_to_move) {
        // inside the loop, the Ultrasonic() function checks the distance to the nearest obstacle. 
        // If this distance is less than or equal to max_dist (the threshold distance at 
        // which an obstacle is considered too close), the stopCar() function is called to halt
        // the robot immediately, and the function exits (return).
        if (Ultrasonic() <= max_dist) {
            stopCar();
            return; // Stop moving if an obstacle is detected within the maximum detectable distance
        }
        // After each obstacle check, the program pauses for check_interval milliseconds 
        // (set to 10 ms above) before checking again. This brief delay controls how frequently the robot checks for obstacles
        delay(check_interval); // Wait for the check interval before checking again
    }
```

## Guidelines for the part 2 report

- Submit your **code** (note that your code should be written modular according to the above modules), a **good video** of your robot actively moving through an obstacle course avoiding them, and your presentation. You can submit one video per group but each group member should be able to answer questions when being asked. 
- Feel free to change how your robot does obstacle avoidance and your own twists. 
- I will ask questions from each group member about different parts of the project and they should be able to explain how things work. 
- No need for a written report. 
  
Good luck!