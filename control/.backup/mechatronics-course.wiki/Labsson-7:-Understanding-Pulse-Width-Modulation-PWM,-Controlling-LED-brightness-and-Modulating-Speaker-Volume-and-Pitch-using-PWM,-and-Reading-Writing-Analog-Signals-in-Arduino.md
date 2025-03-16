## Introduction

In this labsson, we will become familiar with **Pulse Width Modulation (PWM)** with a focus on **controlling LED brightness**, **modulating speaker volume and pitch**, and **reading analog signals in Arduino**. This labsson provides an understanding of how PWM enables precise **control over digital outputs to simulate analog behavior**.

## PWM Warmup Activity: `analogWrite()` in Arduino

`analogWrite()` is used to output a **simulated analog value** using Pulse Width Modulation (PWM) (we'll see what it is in a bit) on a specified **digital pin**. Unlike `digitalWrite()`, which can only output a `HIGH` or `LOW` state, `analogWrite()` can simulate **varying levels of voltage** by **rapidly switching the output between `HIGH` and `LOW` at a duty cycle corresponding to the desired analog value**. This is particularly useful for controlling the **brightness of LEDs** or **the speed of motors**.

It’s important to note that **not all digital pins support PWM**. Only specific pins are **PWM capable** (3, 5, 6, 9, 10, and 11 on the Uno). When some digital pins support PWM, it means they can output a Pulse Width Modulated (PWM) signal using hardware timers. Key Differences Between `digitalWrite()` and `analogWrite()` are:

- `digitalWrite()` can only set a pin to `HIGH` or `LOW`, while `analogWrite()` can **vary** the output in a range from **0 to 255**.
- Not all pins support `analogWrite()` since it requires PWM capability, but all digital pins support `digitalWrite()`.
- `digitalWrite()` is used for **binary state devices** like **turning an LED on or off**, while `analogWrite()` is suitable for applications requiring **variable output** like **adjusting LED brightness or motor speed**.

Syntax:

```cpp
analogWrite(pin, value);
```

Where pin is the number of the **PWM capable pin** you want to write to, and value is the duty cycle between **0 (always off)** and **255 (always on)**.

***
Explain why 0 volts corresponds to 0 and 5 V corresponds to 255 (think about the number of bits used to code the voltages between 0 volts and 5 volts). What about the voltages in between? Make sure to provide a mathematical relationship for converting known voltages to their corresponding PWM values and PWM values to their corresponding voltages. (**5 points**) 
***

Now to understand how `analogWrite()` works, do the following simple activity. 

**Class Activity: LED Brightness Control using PWM in Arduino (15 points)**

**What to submit:**
- A video showing the correct function and then briefly showing and explaining your two codes. Make sure to answer the question below when explaining the later code. 
- Your codes. 

In this activity, you will understand **how Pulse Width Modulation (PWM) works** and how it can be used to control the brightness of an LED using the `analogWrite()` function in Arduino. You will manipulate the brightness levels of an LED by changing the **duty cycle of a PWM signal**.

**Hardware:** Connect one LED through a current limiting resistor (a 120-ohm resistor should do the job) to one of the PWM pins of the Arduino. 

**Software:** Open your Arduino IDE and write a code to control the brightness of the LED using a PWM signal:
- Define a variable for LED pin, a variable for delay, and several variables for the LED brightness
- In `setup()` function, set the LED pin as an output
- In the `loop()` function, write the code to control LED brightness using `analogWrite()`. Include delays between changes in brightness to observe the changes clearly. Sample code below can give you an idea how to do it:

```cpp
void loop() {
  analogWrite(LED_pin, brightness_level_1); // Set brightness_level_1 to 0 corresponding to off case
  delay(dt); // wait for about 1 second 
  .
  .
  .
}
```
- Verify and upload the code to your board and explain what happens. 
- Now modify the simple code above and write it more efficiently using arrays and `for` loops. An array in Arduino is a collection of values of the same data type, stored in a single variable and accessed using an index (note that indices start from 0 and not 1). The syntax is as follows:

```cpp
datatype arrayName[] = {value1, value2, ...};
```
For example:

```cpp
int numbers[] = {10, 20, 30};  // Array of 3 integers
int x = numbers[1];  // Access second element (20)
numbers[2] = 50;  // Modify third element
```
Note that you can find the size of an array using `sizeof()` function:

```cpp
int length = sizeof(arrayName) / sizeof(arrayName[0]);
```
In the code above, `size(arrayName)` gets the total bytes of the array, and `sizeof(arrayName[0])` gets the bytes of one element. This byte size is the memory size in bytes of a variable or data type, which depends on the microcontroller type. The following numbers are for Uno's microcontroller, which is ATmega328P. `char` is 1 byte, `int` is 2 bytes, `float` is 4 bytes, etc. **Explain why the above code gives the length of an array.** 

**Note:** You can implement the project above **using different methods**. As an example, look at the **fading** example from Arduino and see how it is implemented using two `for` loops. 

***

In the above warmup activity, you created **analog-like signals** **from digital pins of Arduino**. In other words, the Arduino's digital pins that were capable of PWM were used to **generate signals that mimic analog output**. 

In other words, **we used a digital pin but created an analog-like signal**. `analogWrite()` function works with PWM. When we write `analogWrite(pin, 0)`, this is like `digitalWrite(pin, LOW)` and the signal that is provided as a voltage out of the pin capable of PWM will look like this:

<figure>
<p align="center">
<img width="588" alt="zero percent duty LED brightness" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/8d4dabca-e7a8-4dee-86c0-a250ee887363">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

And when we write `analogWrite(pin, 255)`, this is like `digitalWrite(pin, HIGH)` and the signal that is provided as a voltage out of the pin capable of PWM will look like this:

<figure>
<p align="center">
<img width="588" alt="100 percent duty LED brightness" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e6ea6512-e94f-4d3d-8dcc-800fc2e582dd">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

What if we write `analogWrite(pin, 127)` (127 is half of 255)? We may expect the voltage out of the pin to be constant 2.5 v but instead it looks something like this:

<figure>
<p align="center">
<img width="588" alt="50 percent pwm led brightness control" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/7a62d449-a9db-440b-9827-f44ba9880e33">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Because as we discussed, it is a **simulated analog signal** out of the **digital pin**. If you look at the signal above, you see that it is **simulating 2.5 volts** by averaging the time spent in the `HIGH` state (5 volts) and the `LOW` state (0 volts) across the pulse width modulation (PWM) cycle. This is achieved by setting the duty cycle to 50%, meaning the signal is `HIGH` for half the time and `LOW` for the other half within each PWM cycle. By rapidly **switching between these two states** and leveraging the inertia of electrical components and human perception, the PWM signal **effectively mimics a constant voltage of 2.5 volts**. **This principle allows digital systems, which can only output full-on or full-off states, to control devices in a way that appears analog that enables the fine adjustment of LED brightness, motor speeds, and speaker volumes, among other applications.**

***

Given our discussion on PWM above, if we write the `analogWrite(pin, 20)` command, what would be the **waveform** of the PWM signal produced by the digital pin, and what **analog voltage level** is it effectively **simulating**? How about `analogWrite(pin, 200)`? Make sure to calculate the duty cycles and effective voltages. On Tinkercard, use an Arduino and an oscilloscope to visualize these waveforms.(**10 points**)

***

After this warmup activity, let's officially see what PWM signals are. 

## Introduction to PWM signals

As we saw earlier, **Pulse Width Modulation (PWM)** is a technique used in electronics to generate **analog-like signals from digital devices**. It's commonly used for controlling the **intensity of LEDs**, the **speed of motors**, and other applications where **variable power levels** are required. Let's dive into a detailed tutorial on PWM. 

**Definition.** PWM is a method for **encoding analog information** in a **digital signal** by varying the **duty cycle of a square wave**.

**Duty Cycle.** It's the **percentage of time** the signal is high (ON) compared to the total time of one cycle, as shown in the figure below:

<figure>
<p align="center">
<img width="392" alt="duty cycling" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/3ca4cafb-6dce-4f51-b5aa-3245b5a69491">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

A PWM signal is a series of **ON and OFF states** with varying durations. The average voltage over one cycle determines the effective voltage. Higher duty cycles result in higher average voltage.

### Arduino PWM

As we saw in the warmup activity, Arduino boards have **built-in PWM capability** on certain pins (e.g., Arduino Uno pins 3, 5, 6, 9, 10, and 11). `analogWrite(pin, value)` is used to **generate PWM signals** on the specified pin.

### PWM Frequency and Resolution

**Frequency.** PWM signals have a frequency, which is the number of cycles per second. **Arduino default PWM frequency is around 490Hz for pins 3, 9, 10, and 11 and 976 Hz for pins 5 and 6**.

**Resolution.** It refers to the number of steps in the duty cycle. **Arduino Uno has 8-bit resolution (0 to 255)** as we used it in the warm-up activity. For an Arduino Uno, which has an 8-bit resolution, this means the duty cycle can be set to one of 256 different levels, ranging from 0 to 255. A value of 0 would mean the signal is always off, and a value of 255 means the signal is always on, with values in between controlling how bright an LED appears or how fast a motor runs by adjusting **how much time the signal spends in the on versus the off state** within each cycle. For example, **127 can be approximately 50% duty cycle**. 

### PWM Applications

- **Motor Speed Control.** PWM is widely used for controlling the speed of motors that we will see in the next labsson. 

- **Light Dimming.** Used in lighting systems to control brightness. We also saw an example at the beginning of this lesson. 

- **Audio Generation.** PWM can be used for simple audio generation, as we will see in the following activity. 

***

**Class Activity: Modulating Speaker Volume and Pitch with Arduino (15 points)**

**What to submit:**
- Videos showing that the requested parts work and a brief explanation of the codes.
- Codes. 
- Make sure to answer the questions in your report. 

Here, you will learn how to control the volume and pitch of a sound produced by a speaker connected to an Arduino. 

**Hardware setup:** Connect one terminal of the 8 Ohm Mini Speaker (its impedance is 8 ohms) to a PWM-capable digital pin (e.g., pin 9) on the Arduino and connect the other pin through a current limiting resistor to the ground. **Explain why you need this current-limiting resistor.**

<figure>
<p align="center">
<img width="392" alt="connecting a speaker to an arduino" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/132c78cb-18fb-44da-84cc-1983f34d506e">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Software:** 

**Controlling the pitch of the sound:** In the Arduino, you can use the `tone()` function to **generate a square wave of the specified frequency** (and thus a specific tone) on a pin. Here is an example code (try to change the frequency to have sounds with different pitches):


```cpp
int speakerPin = 9; 

void setup() {
  // No setup needed for this example
}

void loop() {
  tone(speakerPin, 440); // Generate a 440 Hz tone
  delay(1000); // Play the tone for 1 second
  noTone(speakerPin); // Stop playing the tone
  delay(1000); // Pause for 1 second
}
```

**Note:** `tone(pin, frequency, duration);` plays a tone with a specific frequency and for the specified duration. The `tone()` function generates a square wave at a specified frequency using a timer on the specified pin. It resembles PWM, but its duty cycle is always 50%.

Now use this `tone()`, and `noTone()` functions to generate music. Here is an idea of how to do it (**disclaimer**: I am not a musician so feel free to add anything needed just in case I missed it):

- Define the pin connected to the speaker. 
- Create an array with note frequencies and name it `melody[]`. 
- Use an array to define note lengths in milliseconds and call it `noteDurations[]`.
- Use a `for` loop to play each note using the `tone()` function. Don't forget to add a short delay in the amount of note durations. 
- Use **noTone(pin);** to turn off the tone after playing.
- You can put your melody in `setup()` to play only once or in the `loop()` to repeat. 

After the successful implementation of this part, now amplify the generated music using the LM386 module. You only need to change the hardware connections as follows:

<figure>
<p align="center">
<img width="565" alt="audio_amplifier" src="https://github.com/user-attachments/assets/434ee065-f1d1-4921-9509-d70e824a9159">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Turn the gain knob to control the volume of the amplified sound output. Note that this module is designed to drive speakers, and it already limits the output current to safe levels, so you do not need a current-limiting resistor in this case. This module works with high-impedance speakers like the one that we have since it only supplies a limited power (0.5 W - 1 W max), and if the speaker is low impedance and it draws more current than the module can safely provide, it can damage the amplifier module.   

**Note:** The difference between PWM and the `tone()` function:

<figure>
<p align="center">
<img width="678" alt="pwm_vs_tone" src="https://github.com/user-attachments/assets/6aa560dc-6d2e-41d0-a432-4d095d3970b7">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Based on the above differences, explain the difference between the following codes:

```cpp
int speakerPin = 3; 

void setup() {
  // No setup needed for this example
}

void loop() {
analogWrite(speakerPin,200);
delay(1000);
analogWrite(speakerPin,100);
delay(1000);
analogWrite(speakerPin,0);
delay(1000);
}

```

And

```cpp
int speakerPin = 3; 

void setup() {
  // No setup needed for this example
}

void loop() {
 tone(speakerPin, 440); // Generate a 440 Hz tone
  delay(1000); // Play the tone for 1 second
  noTone(speakerPin); // Stop playing the tone
  delay(1000); // Pause for 1 second
}

```

You can watch a cool animation of how a speaker works [HERE](https://youtu.be/RxdFP31QYAg). 

***

### PWM Limitations

- **Limited resolution** can result in less precise control.
- Some applications may require **additional low-pass filtering**. By using a capacitor at the output to smooth out the signal, we can get a real analog signal. 

## Reading Analog Voltages Using `analogRead()` Command

In the previous labsson, we could read a digital voltage (0 or 1) from a pushbutton using `digitalRead()` command. Here, we want to know how to read **analog voltages** because suppose that you have a **sensor** that changes its **resistance** based on environmental factors like temperature, light, or moisture. This change in **resistance affects the voltage across the sensor**. By reading these voltage variations with an Arduino's analog pin, we can interpret the sensor's data to quantify the changes in the environment. 

***
**Class Activity:** Reading the voltage (basically a voltmeter) from a voltage divider output using Arduino (**20 points**)

**What to submit:** A photo of your implementation, your code, calculations, screenshots of serial monitor and answers to questions

Go back to [Labsson 2](https://github.com/madibabaiasl/mechatronics-course/wiki/Labsson-2:-Resistors,-Variable-Resistors-AKA-Potentiometers,-and-Resistor-Ratings) and implement the voltage divider circuit. Then do the following:

- Connect the **junction between the two resistors** to one of the **Arduino's analog input pins (A0 through A5)**, and also connect the VCC and GND for the circuit.
- Open the Arduino IDE and create a new sketch.
- Initialize the analog pin you've connected to the voltage divider as an input. 
- In the `setup()` function, begin **serial communication** using `Serial.begin(9600);` to send data back to the computer over USB.
- In the `void loop()`:
  - Read the Analog Input: `int varName = analogRead(A0);` Note: `analogRead(A0);` reads the voltage at the analog input pin A0, where your voltage divider is connected. The Arduino **converts this analog voltage into a digital value** (ranging from 0 to 1023) which is stored in the variable (`varName`). This conversion is based on the reference voltage of 5V (for most Arduino boards), where 0 corresponds to 0V and 1023 corresponds to 5V. Print this number on the serial monitor and see what number the Arduino is reading. 
  - Write a line of code that converts the digital value to the corresponding voltage. In other words, this line should convert the digital value that you read between 0-1023 back into the corresponding voltage value in volts. 
  - Print the Voltage to the Serial Monitor, calculate the expected voltage and show that it is equal to the voltage that the serial monitor shows.
  - Do not forget to implement a delay between readings. 

**Question:** The Arduino's Analog to Digital Converter (ADC) represents analog voltage levels with digital values ranging from 0 to 1023. This means that the ADC divides the voltage range it can read (0V to 5V for many Arduino boards) into 1024 discrete steps. How many bits does the Arduino's analog-to-digital converter (ADC) use to represent analog voltage levels?

***

**Class Activity: Measuring the voltage output of a potentiometer (20 points)**

**What to submit:** Photos of your implementations, your codes, calculations, screenshots of serial monitor, videos if asked with brief explanation of the code and answers to questions

Again go to Labsson 2 and review how a "pot" works (include a short review in your report as well). Now do these things:
- Based on what you have learned in Labsson 2 and this lesson about analog inputs, explain how you can hook up a pot to Arduino. Implement this using a pot, breadboard and Arduino. Make sure to provide a photo of your implementation in your report.   
- After wiring your potentiometer, you can use the same method above about reading the analog inputs to display the output voltage of the potentiometer. Rotate the knob to get different voltage readings on the serial monitor.

**Questions:** 
1. Print out the value that Arduino reads from the analog input. What is the minimum value and what is the maximum values. Make sure to provide screenshots.
2. Now, convert this digital value that serial monitor shows back to the analog voltage. What is the minimum and maximum values. Make sure to provide code and screenshots in your report. 
3. What is the potentiometer's role here? 

- Now use this measured voltage from the potentiometer to control an LED's state. Implement a code segment that lights up the LED `if` the voltage surpasses a specific threshold (say 4 volts). Don't forget to use a current limiting resistor for the LED, initialize the LED pin, and also turn off the LED `if` the voltage is below 4 volts. Feel free to play around with different conditions for the `if` statement. Provide your circuit, code, and a video of your implementation with a brief explanation of the code. 
- Now, see if you can use the read values from the analog input to create a Dimmable LED where the LED's brightness is gradually changing. Note that Arduino changes the voltage read from the analog input (using `analogRead()` command) to a number between 0 and 1023. However, when you want to `analogWrite()` to an LED, the range will be between 0 and 255.  So, the value read from the analog input needs to be **mapped from a range of 0 to 1023 to a range of 0 to 255**. Submit your calculation, code, and a video of the implementation with a brief explanation of the code. 
- Now use the potentiometer to control speaker volume through PWM (either use the amplifier to drive the pot or use a current limiting resistor). Explain why at 255, the speaker has no sound. Submit code, video and answer to the question.  

***

## Guidelines for the labsson 7 report

- The grading criteria are as follows:
  - Abstract (5 points)
  - Each activity and question according to respective points mentioned throughout the text (total = 85 points)
  - A conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - References (disclose the use of AI) (5 points)

- Some notes: 
  - You can upload the video to your SLU OneDrive and put a link to it in the report. Make sure to give me and the TA access to the video. You can also directly upload it to the assignment.
  - The code can be submitted as a separate file or as a GitHub link. 
  - Creative twists to projects have extra credits. 

Good luck!
