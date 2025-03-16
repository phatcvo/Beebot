## Introduction

Let's take this step, where we were first introduced to the following diagram that shows different components of a Mechatronics System (that we have discussed in previous labssons): 

<figure>
<p align="center">
<img width="783" alt="elements of a mechatronics system-background" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/3c090bb1-dbe4-4a66-8789-919fc1943a32">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

In this project, we want to start to see how these individual parts - actuators, sensors, and controls - all work in tandem to create a **complete Mechatronics system** like a wheeled mobile robot that we have. By the end of this part you should have completed the following tasks:

- **Assemble** the mobile robot and fully **charge** its battery
- Gain a basic understanding of **how the wheeled robot move**
- Program and calibrate the robot for **basic movements** (moving **forward**, moving **backward**, turning **left**, turning **right**, and coming to a **stop**)

## Assemble the Robot and Charge the Battery

Before everything else, you should do the following tasks first:
1. **Assemble the Mobile Robot:** Using the **manual** provided with your Elegoo Smart Car kit, assemble the robot (do this on a desk as it has many small parts that can be lost easily and there are no spare parts). Pay close attention to the assembly instructions, ensuring that each part is correctly installed (also pay attention to **orientation** of the robot and which side is the front and which side is the back).
2. **Fully Charge the Battery.** 
3. Get familiar with different parts of the robot. 
4. Put the switch on the shield in "upload" mode as we will program everything from scratch and need to be able to upload our own code. 

Assembly instructions can be found in the PDF below as well:

[Assembly_Instructions_mobile_robot.pdf](https://github.com/madibabaiasl/mechatronics-course-private/files/14744831/Assembly_Instructions_mobile_robot.pdf)

## Motion Control Basics of the Wheeled Robot 

After assembling your Elegoo Smart Car and ensuring the battery is fully charged, the next step is understanding and implementing motion control. This section will guide you through the basics of how your mobile robot can perform basic movements. By the end of this, you'll have a foundational grasp of how to **command the wheeled robot to move forward**, **backward**, **turn left**, **turn right**, and come to a **stop**. 

### Understanding the Wheeled Robot's Drive System

The mobile robot that you have is propelled by **DC motors connected to its wheels**. By now, you should have a good understanding of motor **drivers** and how we can use them **along the Arduino to control the DC motors**. The basic movements of the car are achieved by **varying** **the speed and direction of these motors**.

In labsson 8, you become familiar with **two motor drivers** (L298N and L293D) and saw that each **can drive two motors** and you also saw that by giving a `HIGH` signal to the En (Enable) pin of each channel, we can **activate the driver**, allowing current to flow to the motors. You could also use these pins to control the speed of the motors via Pulse Width Modulation (PWM). The input (IN) pins, typically labeled IN1 and IN2 for one motor and IN3 and IN4 for the second motor, are used to control the **direction of the motor**. By changing the logic level (`HIGH` or `LOW`) applied to these pins, you can control whether each motor spins **clockwise** or **counterclockwise**. The OUT pins (OUT1 and OUT2 or OUT3 and OUT4) were where the motors were physically connected to the driver. The behavior of the motor (whether it spins and in which direction) is determined by the signals applied to the EN and IN pins as mentioned.

The wheeled robot that we have has **4 wheels**. The **right wheels are paired to operate together**, as **are the left wheels**. This pairing is crucial for coordinated movements, allowing the robot to execute commands like moving forward, backward, turning left, and turning right with precision. Therefore, **one channel of the motor driver should control the right wheels and the other channel should control the left wheels**: 

<figure>
<p align="center">
<img width="390" alt="two_channel_motor_driver_mobile_robot" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/70f4358f-14e8-469e-8a22-9f2c7373b061">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Forward and Backward Movements**

To move **forward**, both pairs of wheels (left and right) are driven **forward**, meaning both motors are activated to move in a direction that pushes the car forward. Conversely, to move **backward**, the motors are activated in the opposite direction, pulling the car **backward**. This is achieved by setting the input pins (IN1 and IN2 for one set of wheels, IN3 and IN4 for the other set) to the appropriate logic levels as follows:

<figure>
<p align="center">
<img width="567" alt="wheeled_mobile_robot_fw_bw" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/25240711-2ff1-4548-989c-8220b53c2cee">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Turning Left and Right**

For executing turns with the Smart Car that you have, or similar four-wheeled robots, the basic principle involves creating a **differential** in **wheel speeds or directions** **between the left and right sides** of the vehicle. **This differential steering mechanism is what allows the robot to navigate left or right turns**. Let's break down how this works for left and right turns with a focus on the direction each set of wheels moves. 

For the robot to turn left, you will control the wheels such that:
- The left wheels move backward.
- The right wheels move forward.

This configuration causes the robot to pivot around its left wheels, effectively making a left turn. 

<figure>
<p align="center">
<img width="260" alt="wheeled_mobile_robot_left_turn" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/dd4d8f3f-9360-454c-a015-58506824c44c">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

For the robot to turn right, the opposite action is taken:

- The left wheels move forward.
- The right wheels move backward.

In this case, the robot pivots around its right wheels, causing it to turn right: 

<figure>
<p align="center">
<img width="260" alt="wheeled_mobile_robot_turn_right" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/cf0a7669-f673-4e69-a8a1-a3fca8a52105">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

This is how you can program them:

<figure>
<p align="center">
<img width="567" alt="wheeled_mobile_robot_right_turn_left_turn" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/76c2868a-73cb-4cee-a9ef-55c1f6b1290a">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

To **stop the robot**, both right wheels and left wheels should be stopped: Since the mobile robot that we have uses a motor driver that has inverter in it, you cannot set all the inout pins to `LOW`, thus to stop the robot you show set the enable pins (PWMA, and PWMB) to `LOW`.

## Guidance on How to Program the Basic Movements of Your Wheeled Robot  

The **motor driver** chip used in this robot is **TB6612FNG**. Like the L298N motor driver IC that you are familiar with, TB6612FNG uses **Pulse Width Modulation (PWM)** alongside **H-Bridge circuits** to regulate motor speed and direction. The chip in the red rectangular box is the TB6612FNG chip. 

<figure>
<p align="center">
<img width="250" alt="TB6612FNG_motor_driver" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/750ee827-0c38-42af-b4fe-cf2187e18c32">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The datasheet for this IC is at the link below:

[TB6612FNG_datasheet.pdf](https://github.com/madibabaiasl/mechatronics-course-private/files/14746167/TB6612FNG_datasheet.pdf)

The **shield** (or the extension board) that tops the Arduino on the Smart Car is essentially a printed circuit board (PCB) that **extends the capabilities of the Arduino**, making it straightforward to connect various components of the Smart Car, including motors, sensors, and power supply. This shield acts as a unifying platform that integrates the control of all these different elements into a single, manageable interface. You can specifically see that the shield accommodates connections for DC motors through the TB6612FNG motor driver. 

The schematic snippet showing part of the circuitry for the TB6612FNG motor driver chip is as follows:

<figure>
<p align="center">
<img width="613" alt="TB6612FNG motor driver and inverter" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/aba46978-8dfd-4c50-840e-c3fb20ffb701">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Important Note:** The **SN74LVC2G14** is an **inverter chip** that can **flip the logic level of the input signal**. If you input a `HIGH` signal, it outputs `LOW`, and vice versa. By passing the control signal through this inverter chip, the system can **automatically** generate the **opposite signal** for the other **direction control pin**. For example, if AIN1 receives a direct signal from an I/O pin, AIN2 can receive its inverted signal via the **SN74LVC2G14**. To learn more about this inverter, you can take a look at the website below:

https://www.ti.com/product/SN74LVC2G14

The pins work like the other motor drivers that we saw but **their names are a bit different**. Here is the explanation of each pin:

- **VM1, VM2, VM3** - These are the **motor voltage input pins** for the driver IC. They are connected to the Vin, which is the supply voltage.
- **PWMA, PWMB** (**like the Enable pins with PWM capability** on the L298N) - These are the inputs for pulse width modulation signals, which control the speed of the motors connected to the A and B channels. As you see from the figure above, they are connected to **pins 5 and 6 on the Arduino** (be sure to have this in mind when programming the robot).
- **AIN1 (pin 7 on Arduino), AIN2 (will be automatically the opposite logic of AIN1), BIN1 (**pin 9 on Arduino**), BIN2 (will be automatically the opposite logic of BIN1)**. These are the input pins (**similar to IN1, IN2, IN3, and IN4** in other drivers you saw) for controlling the direction of the motors. By setting these pins `HIGH` or `LOW`, you can control the forward and backward movement of the motors.
- **STBY** (**pin 3 on Arduino**): This is the standby pin. When set to `HIGH`, the motor driver is enabled. When set to `LOW`, the driver is disabled, and the motors will not run.
- **A01, A02, B01, B02**: These are the **output pins** from the motor driver that connect to the motors. A01 and A02 are the **outputs for one motor**, and B01 and B02 are the **outputs for another motor**.
- **M1, M2, M3, M4**: These labels represent the motors themselves. M1 and M4 are connected to the B channel of the driver, and M2 and M3 are connected to the A channel.

<figure>
<p align="center">
<img width="341" alt="motors_driver_pinout" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/51533617-8890-49e4-b52b-b12a50584638">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

From the above schematic, we can see that **two motors are controlled by the A channel of the TB6612FNG** (labeled as Right motors), and **two motors are controlled by the B channel** (labeled as Left motors). This kind of setup is typical for **differential drive robots**, where the speed and direction of the left and right sets of wheels can be controlled **independently** to allow the robot to move forward, backward, turn, and rotate in place.

Table below shows how you can control the basic motion of this robot:  

<figure>
<p align="center">
<img width="663" alt="basic_motion_new_driver" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/c473afe9-2e70-490e-a83c-467cdf38f17c">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

***
### Programming Guide #1
Based on what you have learned about motor control in labsson 8, use the information above and write an Arduino program where the car goes forward and then backward, then turns right and finally turns left (**do not test this on the ground** and read the important information below). For this part, do not worry about the speed control and set the PWM pins to `HIGH` in the `setup()` (or `analogWrite` 255 to them). Here are some important notes:
- Do not forget to set `STBY` pin to `HIGH` to activate the driver to provide current to motors. 
- Make sure to **test each motion** before writing the other motion to make sure that the robot is doing what it's supposed to do. While testing, **hold the robot in the air** as for example if you just write the forward motion without the backward one and put the robot on the ground, it will go and **crash** somewhere and you will not be able to catch it. 
- Use table above to implement the motions. 
- Save this code as `programmingGuide1.ino`.

**Make sure to show me that you have completed this step to check off this part.**
***

### Programming Guide #2

Now that you know how your mobile robot works, go ahead and create a **new sketch** and in this new sketch:

Rewrite the first code but this time organize it by introducing **functions**. When **organizing your Arduino sketches with functions**, you're essentially breaking down the code into **smaller, manageable parts**. This not only makes your code more readable and maintainable but also allows for easier debugging and reusability of code across different projects. Here’s how to structure your Arduino code with functions. 

**Adding Custom Functions**

You can define your **own functions** to perform **specific tasks**, in this case you will **write functions for the the car's forward, backward and left turn and right turn movements**. Here's how to write and call custom functions in Arduino. A function is defined with a structure similar to this:

```C++
returnType functionName(parameters) {
  // function body
}
```
- `returnType`: The type of value the function returns. Use `void` if it does not return a value.
- `functionName`: The name of the function. Follow the naming conventions and make it descriptive.
- `parameters:` The variables that are passed to the function. They are optional; your function may not need any parameters.

**Example Function**

Suppose you want to blink an LED. You could write a function like this after the `void loop()` function:

```C++
void blinkLED(int pin, long duration) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
  delay(duration);
}
```
This function turns an LED connected to a specified `pin` on for `duration` milliseconds, then off for the same duration.

**Calling a Function**

To call (use) this function in your sketch, you would use **its name** and provide the necessary **arguments** within your `setup()` or `loop()` functions, or even **within another function**. Here’s how you might call the `blinkLED` function:

```C++
void setup() {
  pinMode(13, OUTPUT); // Set the digital pin as output
}

void loop() {
  blinkLED(13, 1000); // Blink the LED on pin 13 every second
}
```

Now based on the above instructions:
- Create several functions after the `void loop()`, for the basic movements of the car. Create one function for **moving forward** (call it `void forward()`) and pass a variable to it (`float distance`) that specifies the amount of **distance** that the car should move forward.
- Write similar functions for **moving backward**, **turning right**, **turning left**, and **stopping the car**. Note that for turning right and left the **parameter** should be the **amount of angle** that the robot should rotate. 
- Note that at this stage, pass the function parameters, `distance`, or `angle` to the `delay()` function in each function. This way, when you call the function in the `loop()` function, it will run the motors for `distance` or `angle` milliseconds. Obviously this is **not distance** but you will calibrate the car in the next steps and then your car can move or rotate exactly the amount of distance or angle that you want. 
- Call each function in the `loop()` function, and make sure that they are working correctly. Add the function that you wrote for stopping the car at the end of the commands in the `loop()` function, so that the commands do not run forever. 
- Save this code as `programmingGuide2.ino`.

**Don't forget to show me that you have completed this step to check off this part as well.**
***

### Programming Guide #3: Calibration of the Wheeled Robot for Precise Distance

In the previous guide, you saw that when you passed `distance` parameter to the `forward` and `backward` functions, the car moves `distance` **milliseconds** which is obviously **time** and **not distance**. In order to make the robot go the **exact distance** that you want, you need to calibrate it. 

**Objective:** Calibrate your wheeled robot to move accurately to the specified distance in the specified directions (forward, backward). I will give you guidance for **forward motion calibration** and you will **do the exact same thing for backward**. 

- **Create a table** like the table below, where one column is **time (sec)** and the other column is the **distance** that the robot will travel in **that amount of time**. You can choose **whatever system of measurement** that you prefer and comfortable working with. 

<figure>
<p align="center">
<img width="331" alt="distance_calibration" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/b578ed8c-7f58-4941-8f50-6dd4a2eff522">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- Now, in the `loop()` function, command the robot to **move forward at maximum speed** **for different set durations**. **Don't forget to stop the car after moving forward for that amount of time**. Test and upload the code to robot's control board, then **set a starting point** on the ground and put the robot on the ground, turn it on, and let it go the distance in the amount of time that you specified. Measure the actual distance and write down this distance for that specified time. Repeat this action for different times and write down the measured distance in the table above. 
- Now, input these data in Excel, and find the linear **trend line**. The trend line will give you the relationship between distance and time in this format that you I am sure you **have encountered in your physics courses for 1D motion**: $`x = vt+x_0`$. The value for $`v`$ will give you **the maximum velocity of the robot (since you already set the PWM pins to 255)**. 
- From the above relationship between the distance and time, **rewrite** the equation **in terms of time** and **update** your **forward** function in the code to reflect this change. Pass this time to the **delay** function, and do not forget to multiply it by 1000 to convert it to milliseconds.
- At the end, call the `forward` function in the `loop` and command the robot to go to your desired distance. Verify that it is calibrated correctly.  
- Now, change the value of `PWMA` and `PWMB` that you already set to 255, to something in between, say 125. Again, call the `forward` function in the `loop` and command the robot to go to your desired distance. **What is the problem?** This problem arises from the fact that you calibrated your distance at the robot's maximum speed and by changing the speed, your calibration is not valid anymore. Follow along to the next guide to solve this issue.
- Save this code as `programmingGuide3.ino` and do not forget to check off this part with me. 

***

### Programming Guide #4: Calibrating for Variable Speeds

As noted in Programming Guide #3, **your robot's distance calibration was accurate at a specific speed (maximum speed, with PWM set to 255)**. To enable **precise distance movement at variable speeds**, you'll need to **calibrate the robot across a range of speeds**. This ensures that your distance commands remain accurate regardless of the speed at which your command the robot to move. **Here, the objective is to find the relationship between PWM number and the actual speed of the robot**. 

- Create similar tables as the previous step, but this time for **different PWM values** to find the relationship between PWM values and the robot's actual speed:

<figure>
<p align="center">
<img width="497" alt="pwm_speed_relationship" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/edf2212d-5d44-467c-bef0-aff596e9f047">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- For **each PWM value**, **repeat the distance calibration experiment** you conducted in Programming Guide #3. This involves commanding the robot to move forward at the chosen PWM setting for a fixed duration, measuring the distance traveled, and recording the results.

- Once you have collected the data, analyze it like before in Excel, find the trend line and then **the actual speed**. 

- Now you have one speed value for each PWM value, do the Excel analysis again but this time find the relationship between speed and the PWM value in this format: $`PWM = f(Speed)`$. This equation will give you the ability to set the PWM for each motor based on the real speed that you want your robot to move. Add a **polynomial of order 2** as the trend line since you will see that their relationship is not linear. 

- Now, update your `forward` function, and add one more parameter to it: `speed` (the desired speed that you want your robot to go at that speed) and in the function, add the equation that you calculated for the required PWM that if you set PWMA, and PWMB to this value, your robot will go at your desired speed (set these in this function as well). Now, set $`t = \frac{distance}{speed}`$, and pass this to the `delay` function. Don't forget to multiply it by 1000 to get milliseconds.
- Call the `forward` function in the `loop` function and command the robot to go a certain distance at your desired speed. Don't forget to stop the car and also in order to avoid the loop to run forever, add the following code after the stop function (explain why this will make the commands to be executed only once):

```C++
while(1==1){

}
```
 
- Test the program and verify that this time no matter what the desired speed is, the robot will go the requested distance. 
- Now do the **same calibration for the backward motion**. Be creative here and **think about a shorter path** instead of doing all the steps all over again.  
- Save this code as `programmingGuide4.ino` and do not forget to check off this part with me again. 

***
  
### Programming Guide #5: Calibration of the Wheeled Robot for Precise Angle (this time we will also include variable rotational speeds)

In guide #2, you saw that when you passed `angle` parameter to the `rightTurn` and `leftTurn` functions, the car rotates `angle` **milliseconds** which is again **time** and **not the specified angle**. In order to make the robot rotate the **angle** that you want, this time you need to calibrate it to get precise angle. 

**Objective:** Calibrate your wheeled robot to rotate accurately to the **specified angle** in the specified directions (right turn, left turn). This should work for **different rotational speeds**. 

- Create a table like the one below and collect data, where one column is **time (sec)** and the other column is the **angle (deg)** that the robot will rotate in that amount of time. You can use a protractor or any other suitable tool to measure the angle of rotation. Again do this at the motors' max speed. You should calculate how much the robot really rotated at the set time.  

<figure>
<p align="center">
<img width="331" alt="time_angle_turn_right_table" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/54ff3116-a9ec-4913-98f0-4b691bfd7777">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The rotation angle can be measured like the figure below:

<figure>
<p align="center">
<img width="331" alt="rotation_angle" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/4e54715c-c93d-472b-8ac8-74e395728aee">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- Now, input these data in Excel, and find the linear trend line. The trend line will give you the relationship between angle and time in this format: $`\theta = \omega t + {\theta}_0`$. The value for $\omega$ will give you the **maximum angular velocity** of the robot (since you already set the PWM pins to 255).

Similar to the issue with linear motion calibration at different speeds, your robot's angular rotation calibration is accurate **at a specific speed** (maximum speed, with PWM set to 255). To enable **precise angular rotation** at **variable speeds**, you'll need to calibrate the robot across a range of speeds.

- Create similar tables as in Programming Guide #4, but this time for different PWM values to find the relationship between PWM values and the **robot's actual angular speed**. For each PWM value, **repeat** the **angle calibration experiment** you conducted here. This involves commanding the robot to rotate at the chosen PWM setting for a fixed duration, measuring the angle of rotation, and recording the results.

<figure>
<p align="center">
<img width="496" alt="time_angle_different_pwms" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/291274f3-adfe-42a2-9434-c0504a13b690">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- Once you have collected the data, analyze it in Excel, **find the trend line**, and then the **actual angular speed** for each PWM value.
- Now you have **one angular speed value for each PWM value**. Do the Excel analysis again, but this time find the relationship between angular speed and the PWM: $`PWM = f(Angular Speed)`$. This equation will give you the ability to set the PWM for each motor based on the real angular speed at which you want your robot to rotate. 
- Update your `rightTurn` function, and add one more parameter to it: `angularSpeed` (the desired angular speed at which you want your robot to rotate). In each function, add the equation that you calculated for the required PWM. If you set `PWMA` and `PWMB` to this value, your robot will rotate at your desired angular speed. Now, set $`t = \frac{angle}{angularSpeed}`$, and pass this to the delay function. Don't forget to multiply it by 1000 to get milliseconds.
- Call the `rightTurn` function in the `loop` function and **command the robot to rotate a certain angle at your desired angular speed**. **Don't forget to stop the car and add the code to avoid the loop running forever**, as you did in Programming Guide #4.
- Test the program and verify that it is functioning correctly. 
- Repeat this for the `leftTurn` (think about a shorter path). 
- Save this code as programmingGuide5.ino and check off this part with me.

***

**Congratulations**! You have now **calibrated your wheeled robot for precise linear and angular motion at variable speeds**. Now do the following task. 

## Practical Task: Navigating an Obstacle Course

Use the calibrated forward, backward, right turn, and left turn movements to navigate the robot through a predefined obstacle course:

- Lay out a course that requires the robot to move straight, turn at specific angles, and navigate around obstacles. For example, the course could start with a straight path, followed by a 90-degree turn, a straight section, another turn, and a final straight path to the finish line.
- Mark the start and finish lines, and clearly define the path and turning points.
- Get creative and design whatever that comes to mind. Creative twists always have extra points. 

## Some Important Notes about Calibration

The **calibration accuracy** can be influenced by several factors:

- **Battery charge:** As the battery charge level changes, the **voltage supplied to the motors** may vary, affecting the **robot's speed and performance**. It's important to ensure that the battery is fully charged or at a consistent charge level during calibration and operation to maintain accuracy.

- **Assuming linearity:** The calibration process often assumes a **linear** **relationship** between the control variables (such as PWM values or time) and the resulting motion (distance or angle). However, in reality, the relationship may **not be perfectly linear** due to factors like motor characteristics, friction, and load variations. It's essential to be aware of these nonlinearities and consider them when interpreting the calibration results.

- **Tire slipping:** Depending on the surface conditions and the robot's weight distribution, the tires may experience slipping during motion. Slipping can occur when the robot accelerates, decelerates, or navigates on low-traction surfaces. Slipping introduces errors in the distance and angle measurements, affecting the calibration accuracy. To mitigate this, ensure that the tires have sufficient traction. For future, you may also consider using encoders or other sensors to compensate for slippage.

- **Tire wobble:** Uneven or loose mounting of the tires can cause wobbling during rotation. Wobbling tires introduce variations in the distance traveled and the angle rotated, leading to inaccuracies in the calibration. Regularly check the tire mounting and ensure that they are securely attached and aligned to minimize wobbling.

### What you have done here is called open-loop control

Note that what you have done here is **called open-loop control**. In open-loop control, the robot's motion is controlled based on predetermined commands or calibrations **without any feedback** from **sensors** to **correct or adjust the motion in real-time**. Open-loop control can be effective for **just simple tasks** and **well-defined environments**. Here are its limitations:

- **Sensitivity to disturbances:** Open-loop control does not account for external disturbances or changes in the environment. If the robot encounters obstacles, uneven surfaces, or variations in load, it may deviate from the expected motion without any corrective measures.

- **Accumulation of errors:** Since there is no **feedback** to **correct errors**, any inaccuracies in the calibration or variations in the robot's performance can accumulate over time, leading to significant deviations from the desired motion.

- **Lack of adaptability:** Open-loop control relies on predefined commands and does not adapt to changes in the robot's dynamics or the environment. If the robot's characteristics change (e.g., due to wear and tear or modifications), the calibration may become less accurate.

### To overcome these limitations, closed-loop control is often employed

To **overcome these limitations**, **closed-loop control** is often employed. In closed-loop control, **sensors** are used to provide **feedback** about the **robot's actual motion**, and this feedback is **compared with the desired motion**. The control system then adjusts the robot's commands or outputs to minimize the difference between the desired and actual motion. This allows for **real-time corrections**, **improved accuracy**, and adaptability to changes in the environment or the robot's dynamics.

Some common techniques used in closed-loop control for wheeled robots include:

- **Encoders:** Encoders attached to the wheels or motors provide information about the **actual rotation or distance traveled**. This feedback is used to compare with the desired motion and make necessary adjustments.

- **Inertial Measurement Units (IMUs):** IMUs, consisting of **accelerometers** and **gyroscopes**, provide information about the robot's **orientation**, **acceleration**, and **angular velocity**. This data can be used to **estimate the robot's position and orientation** and correct for any deviations.

- **Visual Odometry:** By **analyzing images from cameras**, visual odometry techniques can estimate the robot's motion and position based on the changes in the visual features of the environment.

- **Sensor Fusion:** Combining data from **multiple sensors**, such as **encoders**, **IMUs**, and **cameras**, through sensor fusion techniques can provide a more robust and accurate estimate of the robot's motion and position.

## Guidelines for the part 1 report

- **Check off** each programming task with me
- Submit your **code** and a **good video** of your designed course briefly explaining what is about. You can submit one video per group but each group member should be able to answer questions when being asked.  
- I will ask questions from each group member about different parts of the project and they should be able to explain how things work. 
- No need for a written report (to avoid hating me). 

Good luck!
