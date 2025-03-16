## Introduction

In this Labsson, we will delve into **digital control systems** and their pivotal **role in modern mechatronics applications**. This lesson is designed to guide you through the transition from theoretical foundations to practical applications, emphasizing the shift from analog to digital communication methods and the significance of **Arduino** in **prototyping** and development. Through exploring the nuances of digital versus analog signals and the functionalities of microcontrollers, particularly Arduino boards, you will gain insights into the precision, flexibility, and innovation that digital electronics bring to the field of mechatronics. This introduction sets the stage for a deeper understanding of how digital technologies are revolutionizing the design, control, and implementation of mechatronic systems, preparing you for hands-on experiences that will solidify your knowledge and skills in this dynamic and interdisciplinary field.

## Digital vs. Analog Communication 


### Analog Communication in an Example of Robotic Arms in a Factory

Imagine a state-of-the-art production facility tasked with assembling intricate components of a smartphone, a process that demands high precision and flexibility. The facility employs advanced **robotic arms**, which are integral to the assembly line, performing tasks such as placing microchips on circuit boards, soldering tiny connections, and assembling delicate touchscreens.

Let's first consider a scenario where we control the joints of these robotic arms through **analog signals**. The **movement of each arm** was directed by **varying voltage levels or wave frequencies**. For instance, to rotate the joints, the control system uses a direct relationship between the **voltage level** and the **angle** to which the robotic arm should rotate. For example, 0 volts could represent a starting position (0 degrees), and 5 volts could represent the maximum rotation angle of the joint, say 180 degrees.

The control unit generates **analog voltage signals** within this 0 to 5-volt range to **direct the joint's rotation**. The **actual voltage level** sent to the robotic joint determines its **rotation angle**. For instance, a voltage of 2.5 volts would rotate the joint to a position corresponding to 90 degrees, halfway through its range of motion.

In this system, even **minor fluctuations in the voltage signal** could lead to **incorrect positioning**. For example, if **noise** in the system causes the **voltage to vary slightly**—say from 2.5 volts to 2.6 volts—the arm might rotate to a position slightly past 90 degrees, potentially **misplacing components** or **missing the target workstation**. This precision issue becomes critical when placing tiny, sensitive components where a small deviation can result in assembly errors. Let me show this with an example. Suppose you are communicating the number 12. Because of noise and interferences say due to transmitting over a long wire, the message on the receiver side will get will look something like this:

<figure>
<p align="center">
<img width="645" alt="analog with noise" src="https://github.com/user-attachments/assets/2d1a2e07-5a3a-47d0-af74-47858732a91c">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

You can see that on the receiver side, the number can be read anything but the number 12. For these challenges, we have to resort to digital communication. Now, suppose you change the number 12 to binary form, which is 1100, and try to transmit this binary number:

<figure>
<p align="center">
<img width="645" alt="digital with noise" src="https://github.com/user-attachments/assets/5ebbe85b-655c-4dab-8159-2215eca41373">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

You can see that the number 1100 is still recoverable from the noisy digital signal. This number is exactly the number 12. 

### Transitioning to Digital Communication in an Example of Robotic Arms in a Factory

To address these challenges, the facility upgraded the control systems of the robotic arms to **digital communication protocols**. Let's explore a similar scenario where a **robotic arm's joint rotation** is controlled through **digital communication**. Consider the same robotic arm designed to rotate to position itself over different workstations. However, in this scenario, the control system uses **digital signals** to **direct the arm's movements**.

The joint's **rotational angle** is controlled by **digital signals**, which are sequences of **binary codes** (0s and 1s). **Each binary code** corresponds to a **specific rotation angle of the arm**. For example, the binary code "0000" might represent 0 degrees (the starting position), and "1111" (in a **simple 4-bit system**) could represent 180 degrees (the maximum rotation).

The **digital system divides the joint's rotational range into discrete steps**, with each step represented by a unique binary code. This allows for very **precise control** over the joint's position. For instance, if the system uses a **4-bit code**, it can represent **16 distinct positions ($`2^4 = 16`$)**, with each code corresponding to a specific angle (e.g., "0101" might represent 60 degrees, "1010" might represent 120 degrees, and so on). 

***

**Class Activity:** In the above scenario where we want to represent the joint angles between 0 and 180 with a 4-bit digital system, write the **digital codes** and the **corresponding angles**. Create a table, and include these data: decimal value, binary equivalent, and corresponding angle. (**17 points**)

***

**Digital signals** are inherently more **resistant to noise**. Even if a signal is distorted, as long as it can be identified as closer to a 0 or a 1, the correct command can be recovered. This ensures that the robotic arm **reaches the intended position with high accuracy**, regardless of environmental noise.

Suppose that the robotic arm is now tasked with placing high-precision components onto a circuit board, requiring exact positioning for proper assembly. To direct the arm to a 96-degree position, the control system sends a specific binary command, such as "1000". This command is interpreted by the arm's control unit, activating the motor to rotate the arm precisely to the 96-degree position. The digital control system can adjust the arm's position with high precision to enable it to place components accurately on the circuit board. Even in a noisy electrical environment, the arm's position is not affected because the **digital signals retain their integrity** to ensure the arm rotates **exactly** to the intended angle.

From this example, we can say that **analog communication** transmits **information** using **continuous signals** that vary in **amplitude** or **frequency** to represent different data. This method has been foundational in early telecommunications and control systems due to its direct mimicry of natural phenomena. However, its application in precision-demanding fields like mechatronics reveals inherent challenges that we saw in the example above. 

**Digital communication**, in contrast, represents information using **discrete binary codes** (0s and 1s). This method offers several advantages over analog communication, particularly in applications requiring **precision**, such as mechatronics. 

<figure>
<p align="center">
<img width="430" alt="digital vs analog signals" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/2052ff34-f639-4616-aa9e-4f3e806bd461">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The analog signal can be digitized by **sampling it at regular intervals** and then quantizing the sampled values into discrete levels, a process typically performed using an **analog-to-digital converter (ADC)**. Different levels will ultimately be binary-encoded. If the sampling rate is high enough, the analog signal will then be recovered from the digital signal using a **digital-to-analog converter (DAC)**. 

<figure>
<p align="center">
<img width="645" alt="digitizing_analog" src="https://github.com/user-attachments/assets/feffc4c9-1839-480d-9dc8-9134efcedf5a">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### Digital Signal Transmission - Parallel Communication

Note that you need **one wire** to transmit **every bit** of information. For example, for the 4-bit example described above, where digital signals are used to control a robotic joint's rotation with binary codes, each bit of the binary code represents a distinct state (0 or 1). To communicate this 4-bit binary code from the **control unit** to the **robotic arm**, you would need **at least 4 wires**, with each wire carrying one bit of the binary code. Each of the 4 wires would be responsible for transmitting one of the bits in the binary code. For example, if the binary code to rotate the arm to a 96-degree position is "1000", one wire would transmit the first '1', the second wire would transmit the '0', the third wire would transmit the '0', and the fourth wire would transmit the final '0'. In addition to the 4 wires for the binary data, systems often include a common **ground** wire to complete the electrical circuit. This ground wire is not strictly part of the data transmission but is critical for the proper operation of electrical circuits.

<figure>
<p align="center">
<img width="239" alt="4-strand wire to transmit 4 bits" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/306cdee2-96ff-46a9-ac0d-3c07fc645424">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

With **one wire**, you can transmit **1 bit of information**, such as **"on" and "off" states**, which correspond to digital values of 1 and 0, respectively. This basic form of digital communication allows for the simplest control actions, such as **turning a device on or off**. **2 wires** would enable the transmission of **2 bits simultaneously**, increasing the number of representable states to 4 ($`2^2 = 4`$). This allows for more complex instructions, such as "**off, low, medium, high**" for a device setting. **3 wires** increase the capacity to 3 bits, enabling **8 distinct states** or commands ($`2^3 = 8`$). This further complexity can control devices with multiple modes that offer a broader range of operations. **4 wires**, as previously discussed, communicate a **4-bit binary code**, supporting **16 distinct commands or states** ($`2^4 = 16`$). This setup can handle even more sophisticated control requirements, suitable for devices or systems needing a variety of inputs or operational modes. **8 wires** would be required to communicate an **8-bit binary code**, significantly enhancing the digital communication system's capacity. With 8 bits, you can represent **256 distinct states or commands** ($`2^8 = 256`$). This leap in capability allows for much more detailed instructions or data to be transmitted that support complex operations and control tasks. For instance, in the context of mechatronics, an **8-bit system** could precisely **control** the **speed and direction of a motor**, select from multiple modes of operation, or convey detailed sensor data.
 
The **more wires (or bits)** you add, the **exponentially greater the number of states or commands** that can be represented and transmitted that enhance the system's ability to convey complex instructions and perform multifaceted control tasks. However, increasing the number of wires also increases the complexity and cost of the communication system.

In **parallel digital communication**, a **"bus"** refers to a communication system that transfers data between various components of a computer or network. For example, in the example below, we have 8 parallel wires (which is an 8-wire bus) that communicate a byte between two systems:  

https://github.com/madibabaiasl/mechatronics-course/assets/118206851/4f12a6f6-870a-4cff-a81e-a57942fa5f6d

The **decimal value of each bit** in this case is:

<figure>
<p align="center">
<img width="488" alt="decimal value of each bit" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/f7975154-08d7-42fc-9469-32c86996f45a">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

***

**Class Activity:** What decimal (base of 10) number does the binary (base of 2) number above can transmit? What if you wanted to communicate the number 255, what would be the binary value for it? (**17 points**)

***

### Digital Signal Transmission - Serial Communication

**Serial communication**, in contrast to the parallel communication described above, involves sending **data one bit at a time over a single wire or channel**. This method is more **efficient** for long distances or when minimizing physical wiring is a priority, as it significantly reduces the complexity and cost of the cable connections between devices.

In **serial communication**, the timing of each bit is crucial for accurately transmitting and receiving data. Unlike parallel communication, where multiple bits are sent simultaneously across several wires, serial communication **sends bits one after another** over a single wire or channel. This **sequential transmission** means that the **sender and receiver must precisely synchronize** their clocks or agree on a timing protocol to interpret the data correctly.

Serial communication relies on **pre-agreed timing settings** (**baud rate**) between the sender and receiver to ensure that both interpret the start and end of each bit correctly. The data stream is organized into **packets**, with each packet beginning with **a start bit**, followed by **the data bits**, and one or more **stop bits** to signal the end of the packet. Start bit signals the **beginning of a new data packet**. It's usually a transition from a high voltage (idle state) to a low voltage. Data bits are the actual binary data being transmitted. The number of data bits can vary, but **8 bits** (1 byte) is common. Stop bits mark the **end of the data packet**. There can be **one or more stop bits**, indicating the line is returning to the idle state (usually high voltage) and ready for the next packet.

<figure>
<p align="center">
<img width="488" alt="data packet" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/c01bb9c0-b0e4-4436-a6d4-7f9e28d72673">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The **baud rate** is a critical parameter in **serial communication** that defines the **number of signal changes (symbols) per second**. In simple cases where each symbol represents one bit, the baud rate directly corresponds to bits per second (bps). Both the **sender and receiver must agree on the baud rate** to ensure accurate data transmission and reception.

**Accurate timing** ensures that the receiver can distinguish between individual bits in the data stream, even though they are sent sequentially over the same wire. If the timing on the sender and receiver sides does not match precisely, bits can be misinterpreted, leading to errors in the transmitted data. Therefore, both parties must start "listening" to a bit at the correct moment and for the correct duration to accurately reconstruct the original message from the sequence of bits.

**Example:** Say we want to send the binary data "1010" over serial communication, we should follow a specific sequence of steps to encode and transmit the data according to the agreed-upon protocol parameters, such as baud rate, data bits, and stop bits. Here's how the process would typically unfold:

1. Both the **sending and receiving devices** need to agree on the **Baud rate** which specifies the **speed of the transmission**, in **bits per second (bps)**.

2. **Start Bit**: Transmission begins with the sender placing a start bit on the communication line. The start bit signals the receiver that a new data packet is coming. The start bit is always one bit long and is typically a 0 (a transition from high to low voltage in the signal).

3. **Sending the Data** "1010": After the start bit, the data bits are sent **in sequence**. For the binary data "1010", each bit is transmitted one after another. Since we are working within an 8-bit data format and our data is only 4 bits long, we need to decide on the remaining bits. In many protocols, unused leading bits are set to 0. So, the transmitted data might be padded to fit the protocol's data bit length, resulting in "00001010" for an 8-bit system. So, The transmission sequence for "1010" (padded to "00001010" for 8-bit compatibility) and one stop bit would look like this on the communication line:

| Start | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 0 | Stop |

4. **Stop Bit:** Finally, the transmission concludes with a stop bit to indicate the end of the data packet. The stop bit is at a **high voltage** level (1), marking a return to the idle state. If the protocol specifies more than one stop bit, the line remains high for the duration of those bits.

Note that the duration each bit (start bit, data bits, and stop bit) is held on the line depends on the **baud rate**. For instance, at a baud rate of **9600 bps**, **each bit is transmitted for approximately 1/9600 seconds (about 104 microseconds)**. So, **every 104 microseconds, one bit will be sent**. So, the receiver reads the data every 104 microseconds. So, if 104 microseconds is passed and the **signal is low**, then it will **transmit 0**, if it is **high**, it will **transmit 1**. 

For our example, let's illustrate the timings for transmitting the binary data "1010" over serial communication with a specific baud rate of 9600 bps (bits per second). This baud rate means that each bit is transmitted for 1/9600 of a second which is about 104.17μs. Assuming we are sending the binary data "1010" with a simple 8-bit data format (padding with leading zeros to "00001010"), and one stop bit, the sequence would be:

- Start bit (0)
- Data bits (00001010)
- Stop bit (1)

Here's how the timing breaks down for each part of the transmission:

- Start Bit: 0 for 104.17 µs
- First Data Bit (0): 104.17 µs (cumulative 208.34 µs)
- Second Data Bit (0): 104.17 µs (cumulative 312.51 µs)
- Third Data Bit (0): 104.17 µs (cumulative 416.68 µs)
- Fourth Data Bit (0): 104.17 µs (cumulative 520.85 µs)
- Fifth Data Bit (1): 104.17 µs (cumulative 625.02 µs)
- Sixth Data Bit (0): 104.17 µs (cumulative 729.19 µs)
- Seventh Data Bit (1): 104.17 µs (cumulative 833.36 µs)
- Eighth Data Bit (0): 104.17 µs (cumulative 937.53 µs)
- Stop Bit (1): 104.17 µs (cumulative 1041.7 µs)

Here we communicated 00001010 which is the decimal number 10. 

***

**Class Activity: Serial Communication Protocol Design and Analysis**

**Scenario.** Imagine you are working on a mechatronics project that involves **designing a serial communication protocol between two devices**: a **sensor module** and a **control unit**. The sensor module sends sensor readings to the control unit for processing and decision-making. The communication needs to be robust, and efficient to ensure reliable operation under varying environmental conditions.

**Task.** Design a serial communication protocol that meets the following requirements and answer the related questions.

1. **Data Packet Structure: (17 points)**
- Each data packet should contain a start bit, data bits, and stop bits.
- The sensor data to be transmitted is in a range from 0 to 1023 (e.g., temperature, pressure, etc.). 
- Considering the data range, **determine** an appropriate **number of data bits**.

1. **Baud Rate: (17 points)**
- The sensor module sends data every 100 milliseconds. Given this transmission frequency, **propose a suitable baud rate**. Justify your selection based on the data packet structure you've designed.

1. **Transmission Sequence: (17 points)**
- Outline the sequence of bits (including start bit, data bits, and stop bits) for transmitting a sensor reading of 523. Assume the data bits are transmitted most significant bit (MSB) first.
- Calculate the **total transmission time** for one packet at your chosen baud rate.

***

**Digital information can be transmitted using light through fiber optic cables or wirelessly via radio waves. Regardless of the transmission medium, the fundamental principles of encoding, modulation, and signal propagation remain the same.**


## Brief introduction to microcontrollers

A microcontroller is a compact **integrated circuit** that incorporates a **processor core**, **memory**, and **programmable input/output peripherals**. It is designed to execute specific tasks and control various devices in embedded systems (systems that contain embedded computers). Microcontrollers are widely used in a variety of applications, ranging from consumer electronics and automotive systems to industrial automation and IoT (Internet of Things) devices.

<figure>
<p align="center">
<img width="420" alt="a microcontroller" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/5238724d-6edb-4668-b3b7-a2725d9c47b6">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Key features of microcontrollers include:

<figure>
<p align="center">
<img width="630" alt="core features of microcontrollers" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/9a0e3c18-bf54-4fa8-bfcb-66232c526383">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- **Processor Core.** Microcontrollers have a central processing unit (CPU) that carries out the instructions stored in their memory.

- **Memory.** Microcontrollers typically have both program memory or flash memory (where the software or firmware is stored, non-removable) and data memory or RAM (used for storing variables and runtime data, which is volatile).

- **Peripherals.** Microcontrollers come with built-in peripherals such as timers, counters, communication interfaces (e.g., UART, SPI, I2C), analog-to-digital converters (ADC), and digital-to-analog converters (DAC). These peripherals enable microcontrollers to interact with the external world using low-voltage electrical signals.

- **Input/Output (I/O).** Microcontrollers have pins or ports dedicated to interacting with external devices, sensors, and actuators. These I/O pins allow the microcontroller to **read inputs** and **control outputs**. 

- **Clock Source.** Microcontrollers require a **clock signal** to synchronize their operations. The clock source determines the speed at which the microcontroller executes instructions. Think of it as the **heart beat** of the microcontroller.

- **Programming.** Microcontrollers are programmed using a high-level programming language like **C or assembly language**. The code is typically compiled and then loaded onto the microcontroller's **memory**:

<figure>
<p align="center">
<img width="1121" alt="compiling a programming and uploading it to the arduino" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/1aa56ff0-9c9b-4a95-8a7e-d34df9f46b57">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Microcontrollers find applications in various fields, including robotics, home automation, medical devices, automotive control systems, smart appliances, and more. Their versatility, compact size, and cost-effectiveness make them essential components in the development of embedded systems and electronic products. **Popular microcontroller families** include those from companies like **Atmel** (now a part of Microchip), **Microchip**, **STMicroelectronics**, and **Texas Instruments**, among others.

## Arduino as an OPEN-SOURCE PLATFORM

Arduino is an open-source electronics platform that consists of **both hardware and software components** [1]. It is designed to simplify the process of creating interactive projects or prototypes by providing an **easy-to-use development environment**. Arduino boards use **microcontrollers** at their core, and they are widely popular among hobbyists, students, and professionals for a range of applications. The name "Arduino" has an interesting origin that goes back to the history of the project. Arduino's co-founder, Massimo Banzi, named the platform after a bar in Ivrea, Italy, called "Bar di Re Arduino" (King Arduino Bar in English). The story goes that Massimo and the other co-founders used to meet at this bar while working on the initial development of the Arduino project at the Interaction Design Institute Ivrea.

Key features of Arduino include:

- **Arduino Boards.** These are the physical hardware components that form the core of the Arduino platform. Arduino boards come in various models, each with its own set of features and capabilities. The most commonly used board is the **Arduino Uno**, but there are others like **Arduino Mega**, **Arduino Nano**, and more.

<figure>
<p align="center">
<img width="532" alt="uno-mega-nano" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/b23b4e8d-4c21-44c8-b31f-68da80ef9c6c">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- **Arduino IDE (Integrated Development Environment).** The Arduino IDE is a software tool that allows users to write, **compile** (i.e. change into the machine language), and upload code to the Arduino board. It provides a simple and user-friendly interface for programming Arduino microcontrollers using a simplified version of the C++ programming language.

- **Arduino Language.** Arduino programming uses a **variant of C++** with additional libraries and functions specifically tailored for ease of use. Users can write code to control the behavior of the microcontroller and interact with various sensors, actuators, and other electronic components.

- **Open-Source.** One of the key principles of Arduino is its open-source nature. Both the hardware designs and software code are open to the community, encouraging collaboration, modification, and improvement. This has led to a large and active community of users and developers.

- **Extensible.** Arduino can be easily extended by adding "shields," which are **additional circuit boards** that can be plugged into the main Arduino board to provide **extra functionalities**, such as **WiFi**, **Bluetooth**, **motor control**, and more.

## Arduino as a BOARD

Let's focus on the hardware differences between a typical Arduino board and a generic microcontroller.

1. **Integrated Components:**
   - **Arduino Board.** Arduino boards typically come with an integrated **microcontroller**, voltage regulators (you can source 6-20 V into the power connector and the voltage regulator will regulate it down to 3.3 - 5 V on the board), crystal oscillators, USB interfaces, and often have built-in LEDs and buttons. These components are designed to simplify the prototyping process [2].
   - **Generic Microcontroller.** A generic microcontroller often comes as a standalone chip without the additional integrated components. Users need to design and implement external circuitry, such as power regulation and communication interfaces, to use the microcontroller effectively.

<figure>
<p align="center">
<img width="925" alt="arduino board" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/05039eb4-e6d8-4cb8-bc85-2732a05f0f2d">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

2. **Form Factor:**
   - **Arduino Board.** Arduino boards usually follow standardized form factors, making them compatible with a variety of shields (additional boards that can be stacked on top for added functionalities). Common form factors include the Uno, Mega, Nano, etc.
   - **Generic Microcontroller.** A generic microcontroller is typically a standalone chip with no standardized form factor. Users have the flexibility to integrate the microcontroller into custom-designed circuits.

3. **Ease of Use:**
   - **Arduino Board.** Arduino boards are designed to be user-friendly, especially for beginners. The integrated components, standardized form factors, and simplified programming environment make them easy to use for prototyping and learning.
   - **Generic Microcontroller.** Using a generic microcontroller may require more advanced hardware design skills. Users need to design and implement circuits based on the microcontroller's datasheet, and programming is often done at a lower level.

4. **Prototyping vs. Production:**
   - **Arduino Board.** Arduino boards are well-suited for **prototyping** due to their simplicity and ease of use. However, they may not be the most efficient or cost-effective choice for large-scale production.
   - **Generic Microcontroller.** In production, designers often choose generic microcontrollers to optimize costs and tailor the hardware to specific requirements.

5. **Cost:**
   - **Arduino Board.** Arduino boards may include additional components and features, which can contribute to a higher cost compared to a standalone microcontroller.
   - **Generic Microcontroller.** A standalone microcontroller chip tends to have a lower initial cost, but the overall cost may increase when additional components and features are considered.

In summary, the main hardware differences lie in the integrated components, form factor, ease of use, and suitability for prototyping or production. Arduino boards provide a convenient and user-friendly platform for quick prototyping, while generic microcontrollers offer more flexibility but may require additional design effort. The choice depends on the specific needs of the project and the expertise of the user.

## Arduino IDE download

To download the Arduino IDE (Integrated Development Environment), you can follow these steps:

1. Go to the official Arduino website at https://www.arduino.cc/.

2. On the Arduino website, find the "Software" tab or section. This is usually located in the main menu.

3. Arduino IDE is compatible with Windows, macOS, and Linux. Click on the appropriate download link based on your operating system.

4. Choose the version of the Arduino IDE that you want to download. The website might offer both the stable version and a beta version. For most users, the stable version is recommended.

5. Once the download is complete, follow the installation instructions for your specific operating system. The installation process is generally straightforward.

   - **Windows.** Run the installer (.exe) file and follow the on-screen instructions.
   - **macOS.** Open the downloaded .dmg file, drag the Arduino IDE icon to the Applications folder, and follow any additional instructions.
   - **Linux.** Extract the downloaded archive, navigate to the extracted folder, and run the `install.sh` script. Follow any prompts during the installation.

6. After the installation is complete, you can launch the Arduino IDE. If you're using Windows, you can find it in the Start menu. On macOS, it will be in your Applications folder. For Linux, you can launch it from the terminal or use the application menu, depending on your desktop environment.

You have now successfully downloaded and installed the Arduino IDE on your computer. We will use it to write, compile, and upload code to Arduino boards for our projects.

## Some Extra Resources

- [1] Arduino website. "https://www.arduino.cc/education." *Arduino Education*, 2024.
- [2] Internet of things website. "https://bdavison.napier.ac.uk/iot/Notes/microprocessors/arduino/.", 2024.

## Guidelines for the labsson 4 report

- The grading criteria are as follows:
  - Abstract (5 points)
  - Each activity and question according to respective points mentioned throughout the text (total = 85 points)
  - A conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - References (disclose the use of AI) (5 points)

Good luck!