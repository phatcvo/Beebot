## Introduction

In the previous labsson, you were introduced to the concept of **Pulse Width Modulation (PWM)**, and you used it to control LED brightness, modulate speaker volume and pitch, and you also mastered **reading analog signals** in Arduino. We learned the principles of PWM and you saw that PWM is a technique that allows for the **simulation of analog behaviors** using **digital outputs**. 

Building upon this foundational knowledge, in this labsson, we aim to further study the applications of PWM in more advanced scenarios, such as **controlling the speed of DC motors**. In this labsson, we will study DC motors, how they operate, and how we can drive and control them using PWM and H-Bridge techniques. We will also explore the use of a potentiometer for intuitive control of a DC motor's operational parameters. 

## Warm-Up Activity: Controlling DC Motor Speed and Direction

**What to submit: (21 points)**
- A video implementation along with brief explanation of the code. 
- Your code. 
- Answer to questions if any. 

**Objective:** This activity aims to introduce you to the basics of **controlling a DC motor's speed and direction using an Arduino**. By engaging in this exercise, you will learn how to use **digital** and **PWM signals** to manipulate a motor's operational characteristics.

**Materials Needed:**
- 1 x Arduino Uno
- 1 x DC Motor (use the one that you have in your kit)
- 1 x Motor Driver (for now let's use the L293D IC that you have in your kit)
- Jumper wires
- 1 x Breadboard
- External Power Supply (depends on motor requirements, for now use the power supply module in your kit)
- Fan to see the direction of the motor

**Important note 1:** You cannot just **plug the DC motor into the Arduino like LEDs** because DC motors require **more current** than an **Arduino pin can supply**, and the **voltage requirements of a DC motor may exceed what an Arduino can safely handle**. Directly connecting a motor to an Arduino without proper interfacing can lead to **damage of the Arduino's output pins**, or in severe cases, the entire **microcontroller**. To safely control a DC motor with an Arduino, it is essential to use a **motor driver** or an **H-bridge circuit** that can handle the **motor's current and voltage requirements**. These components act as **intermediaries**, **receiving control signals from the Arduino and using an external power source to drive the motor**, thus protecting the Arduino from potential damage.

For the above reason, we use a **motor driver like the L293D IC**, which **can handle higher currents and voltages than the Arduino pins can** directly manage. The L293D IC acts as an **interface** between the Arduino and the DC motor that can **effectively translate the low-current control signals from the Arduino into the higher-current signals needed to drive the motor**. 

**Important note 2:** Both the **power supply** and the **motor driver** are **motor dependent**, meaning their specifications must be chosen based on the **requirements of the motor being used**. The **power supply must provide sufficient voltage and current to meet the motor's operational needs**, while the **motor driver** must be capable of **handling the motor's maximum current draw** without **overheating or failing**. From this you can conclude that you **cannot control a bigger motor** with the **current setup** and you will need different power supply and motor driver for that. 

***

(**21 points**) First, look up the requirements of the DC motor (look for voltage and stall current ratings) in your kit and make sure that the L293D motor driver (look for the current it can supply and its operational voltage) and the power supply module (remember the important ratings?) can provide those requirements. Write a summary of your findings here. 

- What does the stall current of the dc motor mean?
- Can you use the aforementioned components to operate a 10 HP (horsepower) DC motor? Why? 

***

### Step-by-Step Guide for Building the Motor Speed and Direction Control Circuit

The first step to control the speed and direction of your DC motor is to build the circuit. Follow the steps below to do this. 

1. Put the power supply module, and the L293D IC on the breadboard. Pay attention to the positive and negative pins of the power supply and make sure that they are corresponding to those of the breadboard. 
2. Note that the jumpers on the power supply module can be hooked to pins on either side and they can control whether the output voltage is 5 v or 3.3 v. Choose where the jumpers should be based on the requirements. 
3. Make the ground connection: make sure that your Arduino, power supply module, and the motor controller all have a common ground
4. Get familiar with the L293D IC pins:

<figure>
<p align="center">
<img width="392" alt="L293D pins" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/3f583e3c-1fdb-4f8c-9e28-9e157d01075e">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- Note where pin 1 is and how the pins are numbered. 
- This chip can control the speed and direction of two appropriately-sized DC motors and thats why there are two of everything. Pins on the left side can control one motor and pins on the right can control another motor. So, since we have only one motor, we will use the pins on one side (for example left), so all the following statements are based on these pins. 
- Pin 1 (EN1): Controls the speed of the motor through PWM, so where should you connect that on Arduino and what line of code is needed for this? Figure this out and then connect it to an appropriate pin on Arduino. 
- Pin 2 (IN1) and Pin 7 (IN2) can control the direction of the motor. By digitalWriting one pin to `HIGH` and the other to `LOW`, the motor can turn in one direction, and if you change the order, the turning direction will change. Assign two digital pins on Arduino for these pins and make the connection. 
- Pin 3 (OUT1) and Pin 6 (OUT2) are the output pins that directly connect to the motor you want to control. When you apply input signals to control the direction (through pins 2 and 7 for this pair), OUT1 and OUT2 deliver the power from the source (connected to the L293D's supply pin for the motors) to the motor. Now, based on this explanation, connect the red wire of the DC motor to Pin 3 and the black wire of the DC motor to pin 6. 
- Pin 4 (GND) is the ground so connect this to the common ground. 
- Pin 8 ($`V_s`$) is the supply voltage pin for the motors.

### Steps to Writing Your Arduino Code for Controlling Speed and Direction of the DC motor

1. Setup your pins. Assign two digital pins to controlling the motor's rotation direction and assign one PWM pin to control the motor's speed. Also declare and initialize a variable for motor speed (that will take a number between 0-255). 
2. In the `setup()` function, define your pin modes (you know the drill). 
3. In the `loop()` function, write the code to control the direction and speed of the dc motor. 
4. Now upload the code to your Arduino board. Experiment by changing the PWM values to see how it affects the motor's speed. How low can you get with this number? Why?
5. Reverse the motor's direction by swapping the `HIGH` and `LOW` states assigned to direction pins. 
6. Feel free to **play around** with the code to achieve different actions from the motor. For example, write a couple lines of code and try to turn off the DC motor after continuous rotation for some time.  
 
## Some Physics Behind DC Motors

Let's first watch a short video together:
https://youtu.be/CWulQ1ZSE3c

DC motors come in various types, each designed to serve specific applications and operational requirements. Whether it's the simplicity of a **brushed motor**, or the efficiency of a brushless motor, the **underlying principles of operation** remain consistent across all the variations. In this part, we will first study the **core principles** that govern the operation of DC motors. These principles are universal and lay the foundation for understanding how these motors convert **electrical energy** into **mechanical motion**. While our focus will predominantly be on **brushed DC motors**, it's important to remember that the concepts we cover are applicable to all types of DC motors.

***

### Some Basic Principles

Let's start with a touch of nostalgia and a dash of science. Remember when you were a child, and you created a **simple electromagnet**? You took a nail, wrapped a wire around it, and connected the ends of the wire to a battery. You observed that the nail became a magnet, capable of picking up small metal objects. This experiment was your first encounter with electromagnetism:

<figure>
<p align="center">
<img width="606" alt="electromagnet_childhood" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/d57334a6-d06d-4616-9fc9-a0745648c6d8">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Here, the **electric current** flowing through the inductor **created a magnetic field around the nail**, turning it into an electromagnet. So, by applying **voltage** to an inductor, you can create a magnetic field around it. 

![electromagnet_childhood_experiment](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/f2708415-d0b4-4498-8779-5a3546410ac0)

**Alternatively**, instead of running an electric current through an inductor to create a magnetic field, imagine **moving a magnet around an inductor** (a coil of wire). This movement **induces an electric current in the inductor** (electromagnetic induction). 

![electromagnetism_faraday_law](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/0e7d0b82-9d33-4a45-90bc-601e366bedd6)

When you move a magnet in proximity to a coil of wire, you are changing the **magnetic flux** (the amount of magnetic field passing through the coil) over time. According to **Faraday's law of electromagnetic induction**, this **change in magnetic flux** induces an **electromotive force (EMF) in the coil**, which **drives an electric current through** the circuit connected to the coil. This process is the basis for **electric generators**, where **mechanical energy (the movement of the magnet)** is converted into **electrical energy (the induced current)**.

Now, let's see how some of these principles are used to generate **rotational motion** as seen in a DC motor. Imagine we have a **permanent magnet** and an **inductor** placed within this **magnet's field**. When **DC voltage is applied**, current flows through the coil, turning it into an electromagnet, as previously explained. This electromagnet, created by the current flowing through the coil, has a north and a south pole. The **magnetic field generated by the coil interacts with the magnetic field of the permanent magnets**. The coil (current-carrying conductor) experiences a force in the magnetic field of the permanent magnets. In the context of our DC motor, this force acts on the coil, causing it to rotate (pay attention to the direction of the forces).

![lorenz_force_dc_motor](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/2413e3cf-293b-4f81-99cc-bd5b9c6d960e)

But there is an odd thing in the above animation. When the conductor is at a **90-degree angle** to the magnetic field lines (perpendicular), the torque (the force that causes rotation) on the coil becomes zero (the direction of the force is up at the top and down at the bottom). This causes the conductor to lock in vertical position. So, **what can we do to solve this issue?** Let's talk about this in the next section. 

### Physics Behind Brushed DC Motors

To solve the above issue, brushed DC motors use a **commutator**. A commutator is a **mechanical switch** that plays a crucial role in the operation of brushed DC motors by **reversing the direction of current through the motor's windings at precisely the right moments to ensure continuous rotation**. 

https://github.com/madibabaiasl/mechatronics-course/assets/118206851/19352a26-61ba-4dfe-9ae5-dac56bf3720a

As you see in the animation above, a commutator is made up of segments of conductive material, typically copper, attached to the motor's rotor (the rotating part of the motor). Each segment is connected to different parts of the rotor winding. As the rotor spins, these segments make and break contact with **stationary brushes** (made of carbon or other materials) that are **connected to the power supply**.

When the rotor (and thus the commutator) begins to turn, the **brushes** remain in contact with the commutator segments. As each segment rotates past the brushes, the electrical connection between the power source and the rotor windings is broken and then re-established with the next segment. This action **reverses the direction of the current** in the windings every half turn (or every half cycle of the rotor's rotation).

By systematically reversing the **current's direction** in this manner, the commutator ensures that the magnetic poles created by the current in the rotor always oppose the stationary magnetic poles of the **stator** (**the non-moving part of the motor**), thus maintaining **continuous rotation**. Without this mechanism, the motor would not be able to convert DC electrical energy into mechanical energy efficiently.

***

There is **another issue** with the above design related to the **motion of the armature as it reaches the 90-degree position**. This issue concerns the **smoothness of the motor's operation**, particularly as the armature (the rotating part of the motor that includes the coils) aligns with the magnetic field lines. At this point, just as the commutator switches the current's direction to maintain rotation, the armature experiences a moment where the electromagnetic forces acting upon it are transitioning. This transition can lead to less smooth motion. 

https://github.com/madibabaiasl/mechatronics-course/assets/118206851/787ab4db-b4b4-4aeb-ab3e-6c46b0d161ed

To solve the issue of less smooth motion as the armature reaches the 90-degree position, adding more coils to the armature can be an effective strategy. This approach aims to ensure that as one coil is passing through the critical transition point, other coils are positioned in such a way that they are not at this point and thus can continue to produce torque without interruption (each coil has their own set of commutators):

https://github.com/madibabaiasl/mechatronics-course/assets/118206851/6bc90c8f-e87f-4d07-ada4-cb13ee848eb4

The more coils you add, the smoother the rotation will be:

![more_coils_smooth_motion_dc_motor](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e26df721-dcff-4b16-97c2-4b3f1398757e)

***

<!-- ### Summary of Physics Behind Brushed DC Motors

<figure>
<p align="center">
<img width="392" alt="a brushed dc motor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/8ab56062-296c-4a0d-90dc-e49737fa82ee">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Brushed DC motors are among the simplest and most widely used types of electric motors, known for their reliability, simplicity, and ease of control. A brushed DC motor operates on the principle of **electromagnetic induction - wrong!!!!**. It consists of a **stator** containing fixed permanent magnets (the stationary part) and a **rotor/coil** (the rotating part) connected through a **commutator** and **brushes** to a **voltage source**. 

Brushed DC motors leverage the interaction between the **magnetic fields generated by the stator** and the **rotor** to produce **rotational motion**. The basic components and their functions in this process are key to understanding how these motors operate:

- **Stator**: The stator typically contains a set of permanent magnets or electromagnets arranged in a circular fashion to produce a magnetic field.
- **Rotor**: The rotor consists of a coil of wire wound around a central core. This coil, known as the armature, is connected to the commutator.
- **Commutator**: The commutator is a segmented cylindrical device mounted on the rotor shaft. It provides electrical connections between the armature windings and the external circuit through stationary brushes.
- **Brushes**: Brushes are conductive contacts that maintain electrical contact with the commutator as it rotates. They deliver current to the armature windings.-->

### Concluding Marks About the Physics Behind DC Motors

While brushed DC motors are among the most common and widely understood types of DC motors, due to their simplicity, reliability, and ease of control, it's important to recognize that they represent just one category within a broader family of DC motors. Other types of DC motors include **brushless DC (BLDC) motors**, and **coreless or ironless DC motors**. Despite the differences in construction and operation among these various types of DC motors, **the underlying principle that governs their functionality remains consistent**: **the conversion of electrical energy into mechanical motion through principles of electromagnetism**. 

Brushless DC motors, for example, **eliminate the need for brushes and commutators** by using **electronic commutation**, which significantly reduces wear and increases efficiency, reliability, and lifespan. See a simulation at the link below:

https://www.nanotec.com/us/en/knowledge-base/brushless-dc-motors-animation

Coreless or ironless DC motors, on the other hand, feature a rotor without an iron core, which greatly reduces inertia that enables very quick acceleration and deceleration. 

## Principles Behind DC Motor Control

At the start of this labsson, and in the warmup activity, we controlled the DC motor's direction and speed. The speed and direction of a brushed DC motor can be controlled by varying the **voltage applied to the motor terminals** and **by reversing the polarity of the applied voltage**. This is typically achieved using electronic circuits such as **H-bridge configurations** and **pulse-width modulation (PWM) techniques.** 

**H-bridges** are used to facilitate the reversal of motor direction. An H-bridge is an electronic circuit that controls the direction of current flow through a load, such as a motor, to be controlled, enabling bidirectional motion. An H-bridge typically consists of four **switches** (transistors) arranged in an "H" configuration, hence the name. The load, usually a motor, is connected between the two legs of the "H," and the switches control the flow of current through the load as shown in the figures. **You can see that in the left figure, the left side of the motor is connected to positive and the other side to negative, so it will turn in one direction and in right figure you can see that this is reversed so the direction will be reversed.**

<figure>
<p align="center">
<img width="392" alt="H-bridge_1" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/60a669b0-06d0-438f-82b8-f56dbeca3090">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>
 
<figure>
<p align="center">
<img width="392" alt="H-bridge_2" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/a09c1abe-c630-4576-8970-765e756a89c2">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The operation is as follows:

- **Forward Direction (Clockwise Rotation for Motors).** To drive the motor in the forward direction, the upper-left and lower-right switches are closed (turned on), while the upper-right and lower-left switches are open (turned off). This configuration allows current to flow from the positive supply through the upper-left switch, through the motor, and then back to the negative supply through the lower-right switch, causing the motor to rotate in the desired direction.

- **Reverse Direction (Counterclockwise Rotation for Motors).** To drive the motor in the reverse direction, the upper-right and lower-left switches are closed, while the upper-left and lower-right switches are open. This arrangement reverses the polarity of the voltage across the motor, causing it to rotate in the opposite direction.

**Speed Control Using PWM:**

We have already studied that we can control the speed of a DC motor using a technique called PWM. By changing the duty cycle, you can change the amount of the power delivered to the motor and you can control its speed:

<figure>
<p align="center">
<img width="588" alt="PWM_motor_speed" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/a73172fb-d914-45c8-a2ec-74e82d2cd806">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The **L293D IC** that we used in the warmup activity has an inner **H-bridge** that uses the same principle above to control DC motors. As we saw, it has **two H-bridges**, meaning it can control **two DC motors** simultaneously in both clockwise and counterclockwise directions. In the link below, you can see the inner circuit of the L293D IC that we used to drive the DC motor and how the H-bridge concept is used to change the motor direction:

https://www.engineersgarage.com/l293d-pin-description-and-working/

## Class Activity: Controlling the Speed and Direction of the DC Motor with Another H-bridge Motor Driver IC L298N

What to submit: (**21 points**)

- A video implementation along with a brief explanation of the code.
- Your code.
- Answer to questions if any.

**Objective:** In this class activity, we will explore how to control the speed and direction of a DC motor using the **L298N H-bridge motor driver IC**. The L298N is a powerful, **dual H-bridge driver** (it can control the speed and direction of two DC motors) designed to handle **larger current loads than the L293D** (it can handle currents up to 2A per channel), making it suitable for driving **a wide range of DC motors**. It operates at higher voltages, typically up to 46V. 

- Why does the L298N IC come with a heat sink? 

The pins on the IC itself is like this:

<figure>
<p align="center">
<img width="392" alt="L298N_pinout" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/598b7ff3-cc40-4c01-a4c3-37010a2d5378">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The L298N module that incorporates the above IC looks like this:

<figure>
<p align="center">
<img width="392" alt="L298N-Module-Pinout" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/7bb12a2c-d310-46be-a394-411f71be8f63">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Based on what you have learned about L293D, drive your motor using this module. **You may need a power supply more than 5v** to run the motor efficiently. Connect the positive side of the rail to 12v on L298N, and supply a voltage more than 5v say 6 v. Note that EN is short for Enable and it will power the motor. We saw that we can send a PWM to turn the motor on and off really fast to control the speed. 

**Notes on some of the jumpers on the driver:** The L298N motor driver module often comes with jumpers on some of its pins, mainly for configuring the built-in 5V regulator and controlling motor speed settings. The 5V EN (Enable) Jumper enables L298N’s onboard 5V regulator, meaning it provides 5V output (it will step down the input voltage to 5v for the logic circuit and without that jumper you will need to externally provide 5v to this jumper). If the jumper is removed, the user must supply an external 5V power source to the module. If motor enable jumpers (EN A & EN B) are present, then the motor channels (A and B) are always enabled. If removed, the enable pins must be controlled using a PWM signal to vary motor speed.

**Note:** L293D and L298N are two examples of DC motor drivers. There are newer versions of motor drivers but the underlying principles in all of them are the same. Examples can be seen in the link below that are newer versions of motor drivers:

https://www.pololu.com/category/11/brushed-dc-motor-drivers

## Class Activity: Control the Speed and Direction of the DC Motor Using a Potentiometer

What to submit: (**22 points**)
- A video implementation along with a brief explanation of the code.
- Your code.
- Answer to questions if any.

Now get inspiration from the dimmable LED project and design a circuit to control the speed/direction of the DC motor using a potentiometer. Imagine what your motor should do when you turn the knob (its value changes between 0 and 1023 as you saw before) and design your own project.  Note that this method is applicable to many projects for example you can control the tone of a buzzer using this method as well. 

## Guidelines for the labsson 8 report

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