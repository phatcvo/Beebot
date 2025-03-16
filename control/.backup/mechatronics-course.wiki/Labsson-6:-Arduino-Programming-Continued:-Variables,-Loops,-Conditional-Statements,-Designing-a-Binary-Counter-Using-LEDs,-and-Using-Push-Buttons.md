In this labsson, we will continue with Arduino programming by introducing coding concepts such as variables, loops, and conditional statements. Furthermore, we will apply these concepts in a practical scenario by designing a **binary counter using LEDs**. We will also explore the use of **push buttons** in circuits, addressing challenges such as floating voltage values by implementing solutions like pull-up and pull-down resistors. 

## Arduino C++ Variables

In Labsson 5, we saw that by defining the LED pin number as an **integer variable** (`int LED_PIN = 13;`), we only need to change this line of code in case we needed to change the pin that controlled the LED. This exemplifies the fundamental concept of **variables** as **placeholders for data** that can be modified and reused throughout the program. Variables enhance the flexibility and readability of our code, making it easier to update, maintain, and understand. You also saw that we set these variables at the beginning of the code before the `void setup()`. 

In Arduino programming, variables are used to **store and manipulate data**. Here are the basic types of variables in Arduino:

1. **int:** Used to store integers (whole numbers).

   ```cpp
   int myNumber = 42;
   ```

2. **float:** Used to store floating-point numbers (numbers with a decimal point).

   ```cpp
   float temperature = 25.5;
   ```
3. **char:** Used to store single characters.

   ```cpp
   char myChar = 'A';
   ```
4.  **boolean:** Used to store true or false values.

   ```cpp
   boolean isOn = true;
   ```
5. **String:** Used to store sequences of characters (text).

   ```cpp
   String myString = "Hello, Arduino!";
   ```

We use the terms "**declaration**" and "**initialization**" for variables in Arduino programming. This is how we do that:

```cpp
// Declaration
dataType variableName;

// Initialization
dataType variableName = value;
```

Examples:

```cpp
int age;               // Declaration
int age = 25;          // Declaration and Initialization

float temperature;     // Declaration
float temperature = 23.5;  // Declaration and Initialization

char grade = 'A';      // Declaration and Initialization

boolean isWorking = true;  // Declaration and Initialization

String message = "Hello, World!";  // Declaration and Initialization
```

### Variable Scope

The scope of a variable defines where in the code the variable can be used. In Arduino, variables declared outside any function (**global scope**) are accessible throughout the entire program. Variables declared inside a function (**local scope**) are only accessible within that function.

```cpp
int globalVar = 10;  // Global variable

void setup() {
  int localVar = 5;   // Local variable, only accessible within setup()
}

void loop() {
  // globalVar is accessible here
  // localVar is NOT accessible here
}
```

Understanding variables and their types is crucial for writing effective Arduino programs, as they allow you to store and manipulate data to control various aspects of your project. 

## Looping: for Loops in C++

In C++, a `for` loop is a control flow statement that allows you to repeatedly execute a block of code based on a condition. The syntax for a `for` loop is as follows:

```cpp
for (initialization; condition; update) {
    // Code to be repeated
}
```

Example: 

 ```cpp
   for (int i = 0; i < 5; i++) {
       // Code to be repeated
   }
   ```

Let's break down each part of the `for` loop for this example:

- **Initialization:** `int i = 0;` This part initializes a loop control variable `i` to 0. It's executed only once at the beginning of the loop.

- **Condition:** `i < 5;` Before each iteration of the loop, the condition is checked. If this condition evaluates to `true`, the loop continues; if it evaluates to `false`, the loop terminates. Here, the loop will continue as long as `i` is less than 5.

- **Iteration (Update):** `i++` After each execution of the loop's body, the update expression is executed. Here, `i++` increments `i` by 1. This incrementation moves the loop towards its termination condition (i.e., i reaching 5).

- **Code to be repeated:** The comment `// Code to be repeated` is a placeholder for any statements or commands you want to execute repeatedly. In each iteration, this block of code will run until the loop condition (`i < 5`) becomes `false`.

The flowchart of the for loop is as follows:

<figure>
<p align="center">
<img width="509" alt="for_loop_flowchart" src="https://github.com/user-attachments/assets/10376192-d01a-45cf-95a4-57e84ac61712">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

## Looping: while Loops in C++

Another fundamental **control flow mechanism** in C++ (and thus in Arduino programming) is the **while loop**. A while loop allows code to be **executed repeatedly** based on a **condition**. It is particularly useful when the **number of iterations is not known** before the loop starts. The basic syntax of a while loop is as follows:

```cpp
while (condition) {
    // Code to be executed as long as condition is true
}
```

Here are the key components of a `while` loop:

**Condition:** A **boolean expression** evaluated before each iteration of the loop. If the condition evaluates to `true`, the loop's body is executed. If it evaluates to `false`, the loop terminates.

For example, in the example below, the loop continues to execute as long as `i` is less than 5. The statement `i++;` inside the loop updates the value of `i` with each iteration, ensuring that the loop will eventually terminate when `i` becomes 5.

```cpp
int i = 0;
while (i < 5) {
    // Code to be executed
    i++;
}
```

**Choosing Between Loop Types**
- Use a `for` loop when the number of iterations is known or can be determined before entering the loop.
- Use a `while` loop when the number of iterations is not known and depends on dynamic conditions that may change during the execution of the loop.


## Conditional Statements in C++

Conditional statements are a fundamental aspect of programming in C++, including Arduino programming, allowing the execution of different code blocks based on **certain conditions**. These statements enable decision-making processes within the code. The primary conditional statements in C++ are `if`, and `switch`.

The `if` statement evaluates a condition and executes a block of code if the condition is `true`. Its basic syntax is:

```cpp
if (condition) {
    // Code to execute if condition is true
}
```

Example:

```cpp
int temperature = 30;
if (temperature > 25) {
    // Code to execute if temperature is above 25 degrees
}
```

An `else` clause can be added to an `if` statement to execute a block of code when the `if` condition is not met (`false`):

```cpp
if (condition) {
    // Code to execute if condition is true
} else {
    // Code to execute if condition is false
}
```

Example:

```cpp
int temperature = 20;
if (temperature > 25) {
    // Code to execute if temperature is above 25 degrees
} else {
    // Code to execute if temperature is 25 degrees or below
}
```

For **multiple conditions** that need to be evaluated in sequence, the `else if` statement can be used. This allows for more complex decision-making processes.

```cpp
if (condition1) {
    // Code to execute if condition1 is true
} else if (condition2) {
    // Code to execute if condition1 is false and condition2 is true
} else {
    // Code to execute if both condition1 and condition2 are false
}
```

Example:

```cpp
int temperature = 20;
if (temperature > 25) {
    // Code if temperature is above 25
} else if (temperature > 15) {
    // Code if temperature is 16 to 25
} else {
    // Code if temperature is 15 or below
}
```

The `switch` statement is used for making a decision from **multiple alternatives** based on the value of a variable. It is particularly useful when comparing the same variable against several constant values. The syntax is as follows:

```cpp
switch (variable) {
    case value1:
        // Code to execute when variable equals value1
        break;
    case value2:
        // Code to execute when variable equals value2
        break;
    ...
    default:
        // Code to execute if none of the above conditions are met
}
```

Example:

```cpp
char grade = 'A';
switch (grade) {
    case 'A':
        // Code to execute for an A grade
        break;
    case 'B':
        // Code to execute for a B grade
        break;
    // Additional cases as needed
    default:
        // Code to execute if none of the cases are met
}
```

## Designing a Binary Counter Using LEDs

In Labsson 4, we learned about binary numbers and we saw that binary numbers are a **numerical system** that uses only two digits: 0 and 1. This **base-2** system is fundamentally different from the base-10 system that we commonly use (**decimal system**), which consists of digits from 0 to 9. Now, let’s use 4 LEDs to create **a binary counter made up of four bits**, where **each LED represents a single bit**. You can easily connect the 4 LED’s as we did before (note that each LED should be assigned a digital output).

<!-- <figure>
<p align="center">
<img width="438" alt="binary counter" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/8eb72789-6910-4fb2-a131-4cebc0cdab64">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure> -->

Now, according to the calculations we have shown before in labsson 4, we know that **4 bits could represent decimal numbers from zero and up to 15**. **We need our counter to display the numbers from 0 to 15 successively with a small delay, but in binary format using the LEDs**:

<figure>
<p align="center">
<img width="409" alt="binary counter example" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/8f2f0941-3abe-4703-a740-c053f2d126c9">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>


**Note:** In digital electronics, the fundamental concept is that everything is in the form of **switches** that are **either on or off**. This binary system is the basis of all modern computing and digital circuits. For example, computers are essentially **vast networks of millions of tiny transistor switches**, flipping on and off like tiny LEDs to process information.

**A quick note about simple binary addition to find the binary equivalent of numbers above:**

Binary addition is similar to decimal addition, but with simpler rules:

- \(0 + 0 = 0\)
- \(0 + 1 = 1\)
- \(1 + 0 = 1\)
- \(1 + 1 = 10\) (carry the 1)

***

**What you need to do:**
- Modify the code that you wrote in labsson 5 to turn LEDs on and off and implement this binary counter. 
- Use looping whenever necessary, and you can. 
- Remember that since you have 4 LEDs, you will need **4 digital pins** defined as outputs to be able to control these 4 LEDs.  
- Use **variables** whenever possible to make code modification easy.

**What to submit:**
- submit your code and make sure that you add enough comments on what each line will do (**5 points**)
- submit a video first showing that your binary counter works and then showing the code and briefly explaining what each line of code does (**5 points**)
***

## Push Buttons

A push button is an **electromechanical device** used to initiate or control an action in a system or machinery. It operates by transmitting an electrical signal when pressed, activating a specific process or function. A push button has different components:

**Contacts.** A push button typically has **two or more contacts**. When the button is pressed, these contacts are connected, allowing current to flow.

**Actuator.** The part of the button that is pressed or actuated to close the contacts.

### How a Push Button Works

In its default state, the contacts of the push button are **open**. When you press the button, the contacts close, creating a **continuous path for current flow**. 

<figure>
<p align="center">
<img width="409" alt="push button in open and short" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/25904723-e3dc-49be-bc74-e850f0808b05">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>


But we can’t just use push buttons directly in a circuit, because in their open state, the contacts are connected to nothing basically giving an **unknown or floating voltage value**. You will see this floating behavior while **reading from digital inputs** shortly in the activity. For this reason we need Pull-Up and Pull-Down resistors. 

### Pull-Up and Pull-Down Resistors

The purpose of using pull-up and pull-down resistors is to ensure that the voltage at the input pin of a microcontroller is **well-defined** when the button is **not pressed**.

**Pull-Up Resistor:** A pull-up resistor connects the input pin to a voltage source (usually VCC or 5V). When the button is not pressed, the input pin is pulled high (because it is open circuit and the current is zero thus there is no voltage drop across resistor). This way, the read pin which is one of the digital pins on Arduino sees 5v and reports back "1". When the button is pressed, the input pin sees 0 volts and reports back "0". Note that because we are **reading** from the pin, it should be defined as **input**. 

<figure>
<p align="center">
<img width="588" alt="pull up resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/30175100-292c-43f7-9bbe-4be6bfd90b9b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Pull-Down Resistor:** A pull-down resistor connects the input pin to ground (GND). When the button is not pressed, the input pin is pulled low. Here, when the button is not pressed, the read pin sees 0v and reports back "0". When the button is pressed, the current flows and the digital read pin sees the 5v and reports back "1". 

<figure>
<p align="center">
<img width="588" alt="pull down resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/2472de9a-2d4b-4560-87b9-1cc722ab5eea">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

For the **4-lead push buttons** that we have in our kit, the leads pointing towards each other are connected to each other and you can get the switching action between the leads that are not connected with each other:

<figure>
<p align="center">
<img width="392" alt="4-led push button" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/41fe7c00-62bc-4cda-aaa7-79e5e19aed20">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

***
**Objective 1:** Connecting a **push button through a pull-up resistor** to an Arduino, and reading the **button's state** via one of the Arduino's digital input pins through the Serial Monitor.

Based on this goal, this is a guide on what you need to do for the circuit and the code:

- Connect the push button through a pull up resistor to Arduino's 5 v 
- We want to read off of the connection between this lead and the pull up resistor using one of Arduino's digital pins (decide what pin you want to use for this purpose). 
- The push button's other lead should be connected to GND of Arduino.
- Open up your IDE and the Serial Monitor (open it using a button on the upper right corner of the IDE). 
  - at the beginning of the code, define a variable for the read pin and assign it to a digital pin 
  - open up your serial monitor using: `Serial.begin(9600);` which as you may recall 9600 is the baud rate. 
  - **define the read pin to be "input" as we are reading from it**. 
  - in the `loop` function, read the value from the read pin using `digitalRead` function (do not forget to declare it before the `setup` function)
  - now print the value on the Serial monitor using `Serial.println` function.
  - put a small delay at the end for easy reading
  - now verify and upload the code to your Arduino and check your serial monitor, what is it printing? then, push and hold the button, now what is it printing? explain why? (**3 points**)
  - now disconnect the resistor and read from the digital pin directly, do you see the fluctuation? (**3 points**) 

**What to submit:**
- a short description of the objective (**5 points**)
- a photo of your completed circuit for both with the resistor and without the resistor (**5 points**)
- a video showing each of the above parts work as expected and then briefly explaining the code (**7 points**)
- your code (**5 points**)
- answers to questions above

***

**Simplifying the Circuit with Arduino's Internal Pull-Up Resistor**

Instead of connecting an **external pull-up resistor** to the push button and the Arduino's 5V supply (which is the common practice in electronics), you can utilize the **Arduino's internal pull-up resistor**. This simplifies your circuit by reducing the number of external components needed. Here's how to modify your setup:

1. Connect one lead of the push button to an Arduino digital pin. Choose a digital pin (for example, pin 2) to connect to one side of the push button.
2. Connect the other lead of the push button to the Arduino's ground (GND).
3. Activate the internal pull-up resistor in your code. This is done by configuring the pin mode to `INPUT_PULLUP` in the setup function of your Arduino sketch.

**What to submit:**
- your code (**5 points**)

With the push button now operational and the **Arduino accurately interpreting its state**, we can use this functionality to do something. 

***

**Objective 2:** Utilize the operational push button (the input) to control an LED (the output) to implement a simple digital circuit.

- Now, on one side of the breadboard, connect one LED using a **current limiting resistor** to one digital pin of the Arduino. Don't forget to ground the shorter pin. 
- In your program, initiate a variable specifically for the LED and assign it to a designated digital pin on the Arduino board. Don't forget to **set the pin mode to output**. 
- Now in your `loop` function, use the `if` condition to **check the state of the push button**; if the button is pressed (indicating an input of `LOW` due to the **pull-up configuration**), then turn the LED on by setting the corresponding digital pin to `HIGH`. Conversely, if the button is not pressed, turn the LED off by setting the pin to `LOW`.
- Once you have completed the setup and programming, upload the code to your Arduino board. Press the push button and observe whether the LED turns on and release the button to check if the LED turns off. If it is a bit sluggish, change the delay time.

**What to submit:**
-  a photo of your circuit (**6 points**)
- a video showing that it works and then briefly explaining the code (**7 points**)
- your code (**6 points**)
***

**Objective 3:** Implementing a **Toggle Switch** Using a Push button to Control an LED. 

Here, we want to enable **a single button press** to toggle the LED’s state between on and off, rather than requiring the button to be held down for continuous illumination. Note that when the push button was not pressed, the digital pin read a `HIGH` signal due to the pull-up resistor configuration and when the push button is pressed, the digital pin read a `LOW` signal as you also saw on serial monitor:

<figure>
<p align="center">
<img width="392" alt="previous and current state of the push button" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/9fe3b68c-bdfd-4144-b7f8-5c4cb42429de">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Now, the logic should be that **when the previous state is different from the current state, then a toggle action (if it was 0, it should be 1 now and vice versa) should occur**. This means if the button was not pressed in the last cycle (indicating a `HIGH` state due to the pull-up configuration) and is pressed now (indicating a `LOW` state), the LED's state should change. This approach uses the edge detection technique, where the transition from `HIGH` to `LOW` triggers the action, ensuring that the LED toggles between on and off with each press of the button.

- Modify your code to introduce a **boolean variable** that **tracks** the LED's current state (on or off). Within the loop function, implement logic to detect the transition from a non-pressed to a pressed state of the push button. Upon detecting this transition, invert the LED's state:
  - Define a **boolean variable to store the LED's state** (it is either 0 or 1) and **a boolean variable to remember the last state of the push button**.
  - In the loop, first, **read the push button's current state**, and then **compare it with its state during the last iteration**. If a change from `HIGH` to `LOW` is detected (button press), toggle the LED's state.
  - Incorporate a short delay after detecting a button press to avoid bouncing effects, which can cause the circuit to misinterpret a single press as multiple inputs.

- Upload the revised code to your Arduino. Test the toggle functionality by pressing the push button. Each press should change the LED's state from off to on or on to off, regardless of how long the button is pressed.

**What to submit:**
- a short summary of the objective (**5 points**)
- a video showing that it is working and then briefly explaining the code (**10 points**)
- your code (**8 points**)

***

<!-- 
**Further Challenge (if you have extra time and it just has extra credits):** Try to create a lighting control in which you change the level of the LED light using the push button. Starting from no light at all, every time you push the button, the light should grow stronger. Once you reach full illumination, the LED should reset itself back to the no light status (or use another push button to gradually decrease it to no light).
-->

## Guidelines for the labsson 6 report

- The grading criteria are as follows:
  - Abstract (5 points)
  - Each activity and question according to respective points mentioned throughout the text (total = 85 points)
  - A conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - References (disclose the use of AI) (5 points)


- Some notes: 
  - You can upload the video to your SLU OneDrive and put a link to it in the report. Make sure to give me and the TA access to the video. You can also directly upload it to the assignment.
  - The code can be submitted as a separate file or as a GitHub link. 

Good luck!
