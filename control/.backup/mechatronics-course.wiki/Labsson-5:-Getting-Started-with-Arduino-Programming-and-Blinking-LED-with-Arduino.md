We finished the previous labsson by installing the Arduino IDE. In this labsson, we are going to start programming the Ardunio Uno and do our first projects. Let's get started. 

## List of hardware and software needed
- Arduino UNO board with USB cable
- a Computer with the Arduino IDE installed
- Breadboard and jumper wires
- Resistors
- LEDs
- Power Supply (if you decide to disconnect the board from the computer)

## Getting Started with Arduino Programming

First, let's open the **Arduino IDE** to familiarize ourselves with its interface. To do this, click on the Arduino shortcut located on your desktop. Upon clicking, a window will appear as shown below:

<figure>
<p align="center">
<img width="764" alt="arduino IDE start page" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/80cab1ef-0280-4c94-8b9d-723a5e2de54f">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

By default, Arduino uses a **C++-ish programming language**. In Arduino programming, the `void setup()` and `void loop()` functions play crucial roles in controlling the behavior of the microcontroller. These functions are part of the Arduino sketch, which is the program you write and upload to the Arduino board. Let's take a closer look at each of these functions:

**void setup():** The **setup() function** is called once when the Arduino board starts running the program. It is typically used for **initializing variables**, **configuring pins**, and **performing any setup tasks** that need to be executed **only once**. Here's a basic structure of the setup() function:

```C++
void setup() {
  // Code for initialization and setup tasks
}
```

**void loop()**: The **loop() function** is the heart of an Arduino sketch. Once the setup() function has executed, the Arduino enters an **infinite loop**, **continuously executing the code inside the loop() function**. This is where you place the **main logic** and **actions** of your program. The structure of the loop() function looks like this:

```C++
void loop() {
  // Code for the main logic and actions
}
```
### Choosing the Communication Port and the Correct Board of Arduino

When you connect your Arduino board to your computer via USB, the board is assigned a **communication port**. To check the **COM port** to which your Arduino is connected, you can follow these steps:

***

**On Windows:**

1. Plug your Arduino board into a USB port on your computer using a USB cable.

2. Press `Win + X` and select "Device Manager" from the menu.

3. **Locate the Arduino:**
   - In the Device Manager window, look for the "Ports (COM & LPT)" section.
   - You should see an entry with the name "Arduino" or "USB Serial Port" and a COM port number assigned to it.

***

**On macOS:**

1. Plug your Arduino board into a USB port on your computer using a USB cable.

2. Open the Terminal application.

3. **Run the following command:**

   ```bash
   ls /dev/tty.*
   ```

Look for a device listed as `/dev/tty.usbmodem*` or `/dev/tty.usbserial*`. The asterisk (*) represents additional characters assigned to your specific Arduino board.

***

**On Linux:**

1. Plug your Arduino board into a USB port on your computer using a USB cable.

2. Open a terminal window.

3. **Run the following command:**

   ```bash
   dmesg | grep tty
   ```

Look for an entry indicating the connected Arduino, such as `/dev/ttyUSB0` or `/dev/ttyACM0`.

***

Additionally, you can check the COM port in the **Arduino IDE**:

- Open Arduino IDE
- Go to the "Tools" menu
- Select the "Port" submenu
- You should see the available COM ports, and the one with the checkmark is the port to which your Arduino is connected. 

<figure>
<p align="center">
<img width="658" alt="com port that arduino is connected to" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/ca593cc0-ad8b-4bf4-82a1-e24116ab1617">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

By identifying the COM port, you can ensure that you are uploading your Arduino sketches to the **correct device**. If the COM port is not recognized, you may need to install drivers or troubleshoot connectivity issues.

You should also check that you have selected to program the **right Arduino board**, as follows:

- Go to the "Tools" menu
- Click on the "Board" submenu
- Choose the Arduino Board Model (for example: Arduino Uno)

## Project: Blinking LED with Arduino

Let's first write the first program for our first project which is the blinking LED. The "Blink" program is a simple and classic example that we can use. In this example, we'll go through the "Blink" program step by step. The **program will blink an LED** (which by now you know how it works) connected to **pin 13** on the Arduino Uno.

Here's the "Blink" program code (**try to build up to this program by first turning the LED on, then off and then adding delay**):

```cpp
// Define the LED pin
int LED_PIN = 13;

// Setup function - runs once when the Arduino is powered on or reset
void setup() {
  // Set the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

// Loop function - runs repeatedly after the setup function
void loop() {
  // Turn on the LED
  digitalWrite(LED_PIN, HIGH);
  
  // Pause for 1000 milliseconds (1 second)
  delay(1000);
  
  // Turn off the LED
  digitalWrite(LED_PIN, LOW);
  
  // Pause for another 1000 milliseconds
  delay(1000);
}
```
Now, let's break down the program step by step:

**Define the LED Pin:**
   ```cpp
   int LED_PIN = 13;
   ```
This line declares an integer variable named `LED_PIN` and assigns it the value 13. This is the pin to which the LED is connected on the Arduino Uno.

**Setup Function:**
   ```cpp
   void setup() {
     pinMode(LED_PIN, OUTPUT);
   }
   ```
The `setup` function is called once when the Arduino is powered on or reset. In this function, we use the `pinMode` function to set the `LED_PIN` as an **output**, indicating that **it will be used to send signals OUT to the LED**. Note that pins can serve different purposes like inputs or outputs. 

**Loop Function:**
   ```cpp
   void loop() {
     digitalWrite(LED_PIN, HIGH);
     delay(1000);
     digitalWrite(LED_PIN, LOW);
     delay(1000);
   }
   ```
The `loop` function runs repeatedly after the `setup` function. Inside the `loop`, we have four lines of code that turn the LED on, pause for one **second**, turn the LED off, and pause for another **second**, creating a blink effect.

**`digitalWrite`:**
- `digitalWrite(LED_PIN, HIGH);` sets the `LED_PIN` to a high (5V) voltage, turning the LED on.
- `digitalWrite(LED_PIN, LOW);` sets the `LED_PIN` to a low (0V) voltage, turning the LED off.

**`delay`:** `delay(1000)` pauses the program for **1000 milliseconds (1 second)**. This creates a one-second interval between turning the LED on and off.

Now after writing the code, you can check whether or not the compiler would produce any errors by clicking the **tick button on the menu**. 

<figure>
<p align="center">
<img width="523" alt="compile button arduino IDE" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/9bc21173-5c8b-45ba-8403-c37ec170e1f9">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

If the **program is properly compiled**, we can now **upload it to the board** by clicking the **upload button** right next to the compiling button.

<figure>
<p align="center">
<img width="544" alt="upload button arduino IDE" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/df240f99-0d86-4272-a9ca-b6eff83c2953">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Take a short video of your implementation (**10 points**)

***

- By uploading the program above, the LED on the Arduino Uno board should start to blink. Why? (**10 points**)

- Based on what you have learned in Labsson 3 about designing a **current-limiting resistor** for an **LED**, design a circuit that incorporates this resistor in series with an LED and connect it to **pin 13** of your **Arduino UNO** and see the **LED blink**. This will ensure the LED operates safely at the correct current level without being damaged by excessive current. Submit a short video and a photo of your circuit. (**10 points**)

- Change the **blinking periods** to get different blinking patterns by tuning the delay period. Submit your code and video (**10 points**). 
- Replace the pin 13 with another pin number of your choice. Submit your code (**10 points**).  
- Change the program and instead of controlling one LED, **control two or more LEDs**. Feel free to get creative and for example design **traffic lights** or **light chaser** or a **Morse Code** for a word like SOS (find the dots and dashes for this and implement them using time delays) or whatever comes to your mind. You should at least implement one project. Submit your code and video. (**35 points**) 

**Notes:**

- Ensure the power supply voltage matches or is slightly higher than the LED's forward voltage.
- Double-check the polarity of the LED to avoid reverse connection.
- Use the calculated resistor value to prevent excessive current through the LED.

***

## Guidelines for the labsson 5 report

- The grading criteria are as follows:
  - Abstract (5 points)
  - Each activity and question according to respective points mentioned throughout the text (total = 85 points)
  - A conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - References (disclose the use of AI) (5 points) 

Good luck!