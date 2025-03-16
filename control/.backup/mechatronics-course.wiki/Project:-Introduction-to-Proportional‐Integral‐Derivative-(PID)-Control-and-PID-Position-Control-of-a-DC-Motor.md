## Introduction to PID Control

The script for the above video can be found [HERE](https://github.com/madibabaiasl/mechatronics-course/files/15223327/script_intro_to_PID.pdf). 

In previous labssons, we used **open-loop** methods to **control a DC motor**. An **open-loop system** is a system in which **we send a command and rely on the system to execute that command without checking how accurate the actual output is**. Some systems do not behave very well if they are left to behave on their own based on a control signal. This is why we sometimes resort to more complex schemes that check the **system’s actual output** and **compare it against the desired signal**, following which, they send some corrective control signals to regulate the output. **PID control**, which stands for **Proportional-Integral-Derivative control**, is a widely used feedback control algorithm in engineering and **automation**. It is designed to **regulate a system's output to a desired setpoint** by continuously adjusting a control input based on the difference between the setpoint and the measured output. PID is most widely used for a class of systems known as **linear systems** (systems whose mathematical models resemble a **linear differential equation**), one of which is the **DC motor** that we have dealt with before. Even though DC motors might have some really **small non-linearities** related to **gear-box backlash and friction**, they are considered as **“almost” linear systems**. 

***

### Intuition behind the PID controller

When you send **a control command** (like a **PWM signal**) to a plant/system (like **a DC motor**), we can represent the process using block diagrams, as shown in the figure below.

<figure>
<p align="center">
<img width="479" alt="open_loop_control" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/caa704c7-593d-4b10-a189-921a7ac667ee">
</p>
</figure>

You can map the required **PWM signal** to the **desired rotational speed through trial and error**, but this method would assume **perfect control over environmental disturbances**. For example, if the motor is **suddenly loaded** (e.g., **carrying a weight** or is **being stalled by some external loading condition**), the **system's mathematical model changes** and therefore would require a **new tuning trial** (you would most likely need to **increase the PWM value** to **cope with the increasing load**). Of course, this is **not a reliable procedure** for industrial motors. **Motors need to cope with external disturbances on their own by monitoring the feedback from a speed sensor to regulate the output**.

The **easiest way** to create such a controller is to make use of the **error signal** (the difference between the **desired setpoint** and the **actual output** **measured by the speed sensor**) to tweak the control action and hence obtain an automatically updated performance.

<figure>
<p align="center">
<img width="600" alt="closed_loop_control" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e3f3eeb3-1e52-4c37-8cfc-52fcb9e4fb6d">
</p>
</figure>

The controller in this case is called a **feedback controller**. It takes the **error signal** instead of a direct input from the user and computes the regulatory control action. PID is a member of the closed-loop feedback controllers’ family. PID executes the following operations on the error signal to produce a control signal to the process/plant:

- **Multiplying the error by a constant (proportional):** multiplying the error by a constant (called **gain**) considers the current error when computing the control signal.
- **Integrating the error and multiplying by a constant (Integral):** Integrating the error results in a knowledge of the system’s past, since the integration of all past errors informs the controller of how the system behaved in the past to adjust the control action accordingly.
- **Calculating the derivative of the error and multiplying by a constant (derivative):** computing the derivative is equivalent to computing the slope of the curve, which kind of tells the future of the system. The derivative describes the rate of change, which could be used to **predict the future behavior of the error**.
- **Combining the knowledge of the present, future and past:** by adding the outcome from all three operations, the control action is produced. The time-domain PID control law can be written down as follows:

$`u(t) = k_p e(t) + k_i \int e(t) dt + k_d \frac{de(t)}{dt}`$

Where, $`e`$ is the **error signal** as a function of time, $`k_p`$, $`k_i`$ and $`k_d`$ are the **proportional**, **integral** and **derivative** gains, respectively.

By **tuning** these **individual gains** (i.e., changing their values for the considered system), the **desired performance** can be obtained even in the presence of external disturbances, provided that the system is **linear** (PID works for some non-linear systems, as well).

***

### Numerical form of the PID controller

Even though the PID control law resembles a continuous integral-differential equation, it is **possible** to implement the **control law on a computing device using numerical techniques** by **discretizing** the associated function. I will show the numerical version of the control law without delving into the associated math:

$`u[k] = k_p e[k] + k_i T_s \sum\limits_{i=0}^{k}e[i] + \frac{k_d}{T_s} (e[k] - e[k-1])`$

Where k is the **index of the current discrete time instant** (in discrete form, we use integer indices instead of seconds) and $`T_s`$ is the **controller’s sampling time**. The sampling time is the amount of time between two successive control actions computation. **You are going to use the numerical form in the code (note how integral and derivative can be implemented numerically).**

***

### Writing Down a C++ PID function

Let's now **implement a PID function** based on the above numerical form. **Complete** the **missing information** from the code below: 

```cpp
// Function to compute PID control signal
float calculatePID(float setpoint, float currentPos, float prevError, float integral, float deltaT) {
    // PID constants 
    // do not worry about the values for now
    float kp = 2.0;
    float ki = 0.025;
    float kd = 0.0;

    // Calculate error
    float error = 

    // Calculate derivative of error
    // note how derivative term is implemented in numerical form
    float derivative = (error - prevError) / deltaT;

    // Update integral of error
    // note how the integral term is implemented in numerical form
    integral = integral + error * deltaT;

    // Calculate control signal
    float controlSignal = 

    // Update previous error
    prevError = error;

    return controlSignal;
}
``` 
Here, you simply wrote a couple of lines of code to implement a PID controller. Before, digital control, you had to create an analog signal like below to implement this:

<figure>
<p align="center">
<img width="600" alt="digital_pid_vs_analog_pid" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/7faefb61-3a50-4850-8965-2091bd318326">
</p>
</figure>

So, up to now you could implement the PID controller in a digital format. Now, let's continue to complete the project by implementing a PID **position** controller to control the position of a DC motor. 

## PID Position Control of a DC Motor

Based on what we have learned thus far about PID control, the closed loop control to control the position of a DC motor will be like the following:

<figure>
<p align="center">
<img width="600" alt="PID_position_control_dc_motor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/36b5dc2d-f26a-44d9-9d0d-9be3def21a42">
</p>
</figure>

You **have already implemented the PID part** (easy-peasy right?), and now need to get feedback from the DC motor's output to be able to compare it to the set point and feed the difference (error) to the PID controller. We saw that in order to get feedback, we need an element like a **sensor** to **measure the output** and since **we want to control the position**, we need a sensor that can measure the position of the motor's shaft for us and that's why we need **encoders**.  

***

### Encoders

**Incremental encoders** are sensors used to **measure** the **rotational position**, **speed**, and **direction** of rotating machinery. Unlike **absolute encoders** that provide the **exact position of a shaft within a full rotation**, incremental encoders generate **pulses** or **counts** that indicate **changes in position relative to a reference point**. They are commonly used in applications such as motor control, robotics, and industrial automation.

![incremental_encoder_absolute_encoder](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/443b413c-f35d-43be-9d97-dfecc933924d)

One type of widely used incremental encoders is **optical encoders**. The components of **optical incremental encoders** could be listed as follows:
- **Encoder Disk:** The encoder disk is a **rotating disk** with evenly spaced **slots** or **marks** around its circumference. As the shaft rotates, the slots pass through a light source and a photodetector, generating electrical pulses.
- **Light Source and Photodetector:** Incremental encoders typically use an optical mechanism consisting of a light-emitting diode (LED) as the **light source** and a **photodetector** (such as a phototransistor or photodiode) to detect changes in light intensity caused by the passing slots on the encoder disk.
- **Signal Conditioning Electronics:** The electrical pulses generated by the photodetector are processed by signal conditioning electronics to produce digital output signals corresponding to changes in position, speed, and direction.

<figure>
<p align="center">
<img width="450" alt="encoder_motor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/5cb6535b-d933-4a67-af99-1c1d6abc311d">
</p>
</figure>

As the shaft rotates, the slots on the encoder disk pass between the light source and the photodetector. When a slot blocks the light, the photodetector detects a decrease in light intensity, generating a **low signal** (e.g., logic level 0). When a slot allows light to pass through, the photodetector detects an increase in light intensity, generating a **high signal** (e.g., logic level 1). Each transition from **low to high** (**rising edge**) or **high to low** (**falling edge**) of the signal corresponds to a **discrete movement** or **increment** of the encoder shaft. These transitions generate **electrical pulses**, commonly referred to as **quadrature signals**, which **are used to determine both position and direction of rotation**. **Incremental encoders** typically produce **two quadrature signals**, labeled A and B. The **A signal leads the B signal by 90 degrees** for **clockwise rotation** and **lags it by 90 degrees** for **counterclockwise** **rotation**. By **analyzing the phase relationship between the A and B signals**, the **direction of rotation** can be determined. Each encoder has a significant property called **counts per revolution**, which means the **number of encoder counts** associated with **one complete revolution** or a **complete 360-degree rotation** (provided in the encoder/motor datasheet). A **nice visual explanation** (only one minute) is provided in this video: https://www.youtube.com/watch?v=zzHcsJDV3_o.

The motor that you have for this part has a **Hall Effect encoder** (which is a bit different than the optical encoder discussed above) on it and that's why there are more than two wires coming out of it:

<figure>
<p align="center">
<img width="374" alt="dc_motor_with_encoder" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/b5012dc2-156c-41b7-94d6-a3f1c09d6426">
</p>
</figure>

Here are the components and functionalities of a **Hall Effect encoder**:
- **Magnetic Disk:** Instead of **a disk with slots** as found in **optical encoders**, a Hall Effect encoder typically uses a disk with **magnetic poles** (north and south) arranged around its circumference. This disk is attached to the motor's rotating shaft. I opened our motor and you can see the magnetic disk in the video below:

![magnetic_disk_hall_effect_sensor](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/01a7d9cc-e027-4484-ae09-f30774832fa5)

- **Hall Effect Sensors:** These sensors **detect the magnetic field changes** as the magnetic poles on the disk pass by them. Hall sensors are semiconductor devices that generate a voltage when exposed to a magnetic field, thus sensing the presence and polarity of a magnetic field (you can see this sensor in front of the magnetic disk in the video above).

- **Signal Conditioning Electronics:** The output from Hall sensors is usually a **small voltage** that varies with **magnetic field** strength and direction. Signal conditioning electronics amplify, filter, and convert this analog signal into digital pulses that can be easily interpreted by a microcontroller or other digital system.

- **Output Signals:** Hall Effect encoders provide **two output signals**, typically known as **channels A and B** (like optical encoders). These signals are **phase-shifted**, usually by 90 degrees, to allow determination of both **the speed and direction of rotation**. The direction can be discerned by observing the order in which the signals change (similar to optical encoders again).

***

## Implementing a PID position Controller for a DC Motor Using Arduino

By now, you are familiar with [DC motors](https://github.com/madibabaiasl/mechatronics-course/wiki/Labsson-8:-DC-Motors,-Driving-and-Controlling-DC-Motors-Using-PWM-and-H%E2%80%90Bridge-Techniques,-Controlling-a-DC-Motor-Using-Potentiometer), and how to to use different drivers to start them. 

There are different methods to control these motors. The common are the speed and position control and you want to do the position control here. In order to accurately control the position of the dc motor, you saw that you need a **closed loop control system** and **PID** can be the sufficient control method here as the system is pretty much linear. 

The overall process is to **measure the current position** using the motor **encoder** and then **comparing it to the desired position** and if there is an **error**, the **PID control will issue a control signal that manipulates the output accordingly** to make it close to the set point. The Arduino uno will act as the controller (and you already wrote the PID control function) and will issue the control commands. 

***

### Hardware-software setup

**Required Hardware:**
- Arduino Uno
- Incremental encoder with A and B channels attached to a DC motor
- Breadboard
- Jumper wires 
- Motor driver (e.g., L298N)
- Power supply according to the motor's nominal voltage

***

**Wiring Instructions:**
- Connect the A and B channels of the encoder to digital pins on the Arduino board.
- Connect the encoder’s power wire to the 5V pin on Arduino.
- Connect the common ground of the encoder to the ground (GND) pin of the Arduino.

<figure>
<p align="center">
<img width="315" alt="encoder_wires" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/41b907ed-e874-4db0-a96a-0227a47c64cb">
</p>
</figure>

- Connect the dc motor wires to the out1 or out2 of the motor driver
- Set up the motor driver (L298N) like we did in class before

You should have a setup similar to this:

<figure>
<p align="center">
<img width="630" alt="setup_dc_motor_pos_control" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/26d97a33-9b7c-497b-87b3-531d96b0232e">
</p>
</figure>

***

**Coding instructions:**

- **Define Encoder Pins:** Declare and initialize variables for these pins that you used for the encoder (you can name them e.g., `encoderPinA` and `encoderPinB`). Note that, in Arduino Uno, **Pins 2 and 3** are capable of handling hardware **interrupts**. These are referred to as **interrupt 0** and **interrupt 1**, respectively. Therefore, for encoder pins, you can use these pins. 
- **Define Motor Control Pins:** Declare and initialize the pins that you used for motor PWM and direction control (you can name them e.g., PWM, dir1, dir2).
- Set up a variable for **tracking the encoder position** (`tickCount`):

```cpp
volatile int tickCount = 0; // Encoder tick count
// volatile is a variable that can change at any time. 
// this is required when using interrupts and make sure that it updates the values
// instead of considering it constant 
```

- Set up constants for the **encoder's properties** like **pulses per revolution** (`encoderPPR`) and gear ratio (`gearRatio`). These values can be found from the datasheet of the motor at this link: https://www.amazon.com/gp/product/B07GNGQ24C/

The multiplication of pulses per revolution and the gear ratio gives the **resolution of the encoder**. Thus, degrees per encoder tick can be calculated as:

```cpp
const float degPerTick = 360.0 / (encoderPPR * gearRatio); // Degrees per encoder tick
```
- You will also need variables for the target position or set point (`targetDeg`) which is the desired position that you want your motor to reach and some variables that will be used to calculate the PID control later: 

```cpp
long prevT = 0; // Previous time in microseconds
float eprev = 0; // Previous error
float eintegral = 0; // Integral of error
```

***
 
In the `void setup()`:

- Start serial communication to debug and monitor outputs.
- Set the pin modes for encoder pins (`encoderPinA`, `encoderPinB`) and motor pins (`PWM`, `dir1`, `dir2`). Note that encoder is a sensor and we will be **reading from those pins**. 
- Initialize prevT to the current time in microseconds: `prevT = micros();`
- Attach an **interrupt** to `encoderPinA` that triggers on the `RISING` signal to call the `readEncoder()` function that will read `encoderPinB` to determine the direction: `attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, RISING);` 

**Note:** We use an **interrupt** to **immediately** respond to changes detected by `encoderPinA`, **ensuring that every pulse from the encoder is captured in real-time**. This helps accurately measure the motor's position without missing any changes, even while other parts of the program are running.

***

Write some functions after the `void loop()` function for better organization:
- Use the `calculatePID()` function from earlier to calculate the control signal.
- Write a function `void readEncoder()` to update the encoder position whenever the encoder sends a pulse. For this first read `encoderPinB` to determine the direction of the encoder ticks. If it is positive, increment the `tickCount`, and otherwise decrement it. 
- Write a function `void setMotor(int dir, int pwmVal, int pwm, int dir1, int dir2)` to set motor speed and direction:

```cpp
void setMotor(int dir, int pwmVal, int pwm, int dir1, int dir2){
  analogWrite(pwm, pwmVal);
  // CW rotation
  if (dir == 1) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  // CCW rotation
  } else if (dir == -1) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
  }  
}
``` 

**This function will control the motor's speed and direction based on the PID output.** Note that the **control command that PID generates** can be used as the **PWM signal to control the speed of the motor**. If the **control command (PID output) is positive**, **the motor will rotate in one direction**, and **if it is negative, it will rotate in the opposite direction**. **The absolute value of the PID output is used to determine the speed of the motor, while the sign of the output determines the direction.** This method ensures that the motor adjusts its position towards the target efficiently, using real-time feedback from the encoder.

***

In the `void loop()` function: 
- Calculate the time elapsed since the last loop iteration to determine the sampling time that we had in the numerical version ($`T_s`$):

```cpp
long currT = micros();
float deltaT = ((float)(currT - prevT)) / 1.0e6; // Convert microseconds to seconds
prevT = currT;
``` 

In the above code, you **capture the current time in microseconds**, compute the difference from the last recorded time (`prevT`), and convert this difference to **seconds** to get `deltaT`, which is the **sampling interval**. You then update `prevT` to the current time for the next iteration.

- Read the **current position from the encoder** and **convert it to degrees**. First, disable interrupts to ensure `tickCount` is read without interruption, then convert this count to degrees using the predetermined `degPerTick`. Re-enable interrupts after reading.

```cpp
noInterrupts(); // Disable interrupts to ensure consistent reading
float posDeg = tickCount * degPerTick; // Calculate position in degrees. make sure to understand this equation. 
interrupts(); // Re-enable interrupts
``` 

- Use the PID control algorithm to compute the control signal. Call the `calculatePID` function with the current position, target position, previous error, integral of error, and sampling time. This function returns the control signal u.

```cpp
float u = calculatePID(targetDeg, posDeg, eprev, eintegral, deltaT);
``` 

- Convert the PID output into motor control signals for power and direction. The **absolute value** of `u` determines the PWM signal strength (`float pwr = abs(u);`). If `u` is **negative**, the **motor direction is reversed**:

```cpp
 int dir;
 if (u < 0) {
    dir = -1; // Set direction to -1 if u is negative
 }  else {
    dir = 1; // Set direction to 1 if u is zero or positive
 } // Direction based on sign of control signal 
``` 

**Ensure the PWM value does not exceed 255**:

```cpp
if (pwr > 255) {
 pwr = 255;
 } // Limit max PWM value to 255
``` 

- Send the **calculated power** and **direction** to the **motor** and **provide feedback through the serial monitor**. Use `setMotor` to apply the PWM and direction settings. Print the target and current positions to the serial monitor for debugging.

**Note: When the position reaches near the desired position, the error becomes smaller, and thus the control command becomes smaller and thus the amount of power given to the motor through PWM decreases and the motor stops.**

***

`TODO`: Set a target position (`targetDeg`) to some degree say 90 deg, and use these values for the PID: $`k_p = 2.0`$, $`k_i = 0.025`$, and $`k_d = 0.0`$. Upload the code to Arduino and verify that your motor is able to reach and maintain the set target position accurately. Monitor the serial output to check the current position of the motor in degrees and see how closely it matches the target position. Adjust the PID parameters and see what happens. 

## PID Parameters Tuning

The code, if applied directly, will most likely **not yield satisfactory results** (**although the numbers that I gave you at least will not make the system unstable to oscillate**). You might have a very noisy response or a very lazy response. This is because you haven’t ‘tuned’ your **PID gains**. This can be done using the **Ziegler-Nichols method**. The **Ziegler-Nichols method** is a widely used heuristic approach for tuning proportional-integral-derivative (PID) controllers. It offers a systematic procedure for determining initial PID controller parameters based on the **transient response characteristics of a system**. 

The Ziegler-Nichols method involves step-by-step procedures to determine the proportional, integral, and derivative parameters of a PID controller through **experimentation**. The key idea is to identify the ultimate gain and ultimate period of the system's response to a **step** input. 

Let's implement this method on our DC motor that you wrote the code for. 

- Step 1: Set the $`k_d`$, and $`k_i`$ gains to zero
- Step 2: Increase the **proportional gain** to the point where you see a **stable oscillation**. Call this gain $`k_u`$. 
- Step 3: Using the Python Code from Labsson 9, gather the data for `currT`, `targetDeg`, and `posDeg`. Make sure to print these values in the following format to be compatible with the Python code:

```cpp
Serial.print(currT);
Serial.print(","); 
Serial.print(targetDeg);
Serial.print(",");
Serial.println(posDeg);
``` 

- Step 4: Measure the period of oscillation $`T_u`$. To give you an idea of how to do this, I have visualized an output for you (you see that the output is oscillating around the setpoint which is 90 deg in this case):

<figure>
<p align="center">
<img width="396" alt="dc_motor_oscillation_kp_40" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/eeac33bf-f2b4-483f-a475-af26242051c2">
</p>
</figure>


- Step 5: Now use the Ziegler-Nichols table below to determine the final $`k_p`$, $`k_i`$, and $`k_d`$:

<figure>
<p align="center">
<img width="475" alt="Ziegler_Nichols_table" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/f60ead34-c009-421f-b4c6-3ba6a564afa8">
</p>
</figure>

**TODO:** These values are the tuned values and you should enter in the motor DC PID control code. After doing so, again visualize the position vs time using the Python code and see if the system response (DC motor position) is near the setpoint (desired position or target position). You can fine tune the values a bit around these values to get a better response. 

**TODO:** What you have done above is called **regulation** in control theory. We also have a **tracking controller** where the output should be able to track the desired input. To see the power of the PID controller that you designed, change the `targetDeg` from a constant number to a **sinusoidal function over time**. Repeat the same visualization and visualize desired position and the real output vs time and see if your designed controller can act as a tracking controller as well. 

## Effect of Each Control Mode on the DC Motor Output

In this part you are going to study the effect of each **proportional**, **integral**, and **derivative** control modes on **system's output**. 

The **proportional controller** changes the control output in **proportion** to the error. If the error increases, the control action increases proportionally and this means **more control action for large errors**:

$`u = k_p e`$

If the proportional controller gain is set **too high**, the system will begin **oscillating** and become **unstable**. If the proportional controller gain is set **too low**, then it will not respond adequately to **disturbances** or **set point changes**. Using proportional only control has the **drawback** of **offset** which is the **steady state error** that **cannot be eliminated by proportional control alone**. 

**TODO**: Show these with your motor (visualize the target position and position vs time using the Python code). 

***

In order to **eliminate** the **steady state error**, you need to introduce a new control mode which is the **integral control mode** to the proportional controller. Then the new **PI controller** can be defined as:

$`u = k_p e + k_i \int e(t) dt`$

What the integral mode control does is to **increment or decrement the controller’s output over time** to **reduce the steady state error**. The **steady state error** is the difference between the **desired final output** and the **actual output** of the system after it has settled and is no longer changing. Note: PI controller is the most widely used controller in industry. 

**TODO**: Design a PI controller for your DC motor control that makes the steady state error almost zero. 

***

You may have noticed a little **overshoot** in the response of the previous controller. Most control systems have this **overshoot** and in order to fix it we can add a derivative term.

Adding the integral mode controller make the control system a bit **sluggish** and especially in **motion control systems** there **may** be a need to introduce another term which is the **derivative control mode**. It is **not really used in industry** but in applications that need **speed**, it can be helpful.

The **derivative control mode** will produce an output that is based on the **rate of change of the error**. Therefore if the **error changes at a faster rate**, then **this controller will produce more control action**. If the **error does not change**, **this term will be zero**. The adding of the derivative mode will provide more control action **sooner** than the **P or PI control** and can respond to **disturbance** better and will shorten the time that it takes to reach the set point. So by adding the derivative control mode, we will get a complete form of the PID control that we saw at the beginning of this project. So, note that the proportional and integral control modes are essential for most control loops but the derivative mode is only useful in some cases. 

## Further Study

To get more intuition to PID controller, watch the awesome videos by MATLAB below:

https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y

## Guidelines for the report

Submit the following either via Canvas or email:
- the complete code of the project
- a video demonstration 
- a written report showing the results for each part

Good luck!