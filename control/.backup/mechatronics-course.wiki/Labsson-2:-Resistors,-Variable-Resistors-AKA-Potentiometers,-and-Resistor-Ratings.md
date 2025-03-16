## Introduction

In the previous lesson, we learned that the essence of electricity lies in the **movement of electrons**, which are responsible for the **flow of electric charge** in **conductive materials**. This flow, as we learned, is what constitutes **electricity**. In conductors, **electrons** move freely, enabling the efficient transfer of energy as **electrical current**. In contrast, insulators, with electrons tightly bound to their atoms, resist this flow, making them crucial in preventing **unwanted current** in electrical systems.

Now, let's think back to the class activity on the **conductivity of water and saltwater**. We observed that pure water, being a poor conductor, had **higher resistance** compared to saltwater, which showed **lower resistance** due to the ionization of salt. We also explored how electricity can be measured in terms of **current** (amperes) and **voltage** (volts). We delved into **Ohm's Law**, connecting these measurements with **resistance**. Just as we discussed **batteries** and their varying **capacities** and **voltages**, today, we will focus on a key component in controlling and managing these electrical properties in circuits - **resistors**.

Understanding **resistors** and their **variable** counterparts is crucial in designing and implementing efficient electrical circuits and thus mechatronics systems. As we proceed, we will explore:

- The various **types of resistors** and their functions in circuits.
- How to read **resistor values** and understand their importance in circuit design.
- The role of **variable resistors** and how they differ from their fixed counterparts.
- The crucial factors to consider when **selecting resistors** for different applications.
;
## Warm-Up Activity #1: The Effect of Resistance on the Flow of Electricity (All points: 16, Summary: 2 points, Each question according to the below points)

Before we dive into the details, let's do a quick activity to study the effect of resistors on the flow of electricity (current) in a simple circuit. 

- First go to https://www.tinkercad.com/ and sign up and open up circuit simulation. 
- Design a very simple circuit using a **breadboard**, a **fixed resistor**, a red **LED**, and a **power supply** (**3.5 points**). The electrical current should go **through the resistor**, then **through the LED**, and finally go back to the power supply. For now, take the value of the resistor to be $`1K\Omega`$ and the power supply should **supply 5 V** to the circuit. Notice the light of the LED. **Calculate the current** by hand using **Ohm's law** (**suppose that the red LED drops 1.89 V**) and **verify** it with the current that the power supply shows (**3.5 points**). 

**Start of the note about breadboards** 

For most breadboards, the layout and connection points are standardized to facilitate easy assembly and modification of circuits. The central area of the breadboard consists of **rows of holes**, divided into two sections by a central notch. Each row is typically a series of five connected holes, isolated from adjacent rows. These are called **terminal strips** and the connections are like this:

![terminal strip connections breadboard](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/cf0bf544-cd7d-4e22-b1f3-c483b0d5f8ad)

To insert a component into the terminal strips of a breadboard, gently push its leads into the holes of the terminal strip, ensuring each lead goes into a **separate row** to avoid **short-circuiting**.

On either side of the terminal strips are usually one or two columns of holes, running the length of the breadboard. These are known as **bus strips**. They are typically used for **power connections** – one row for the **positive** supply and one for the **ground** (or negative) supply. Components and rows on the terminal strip are then connected to these power lines as needed. The connections are like this:

![bus strip connections breadboard](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/2a1af158-ab42-4931-85df-9cf18ac3d8c6)

**End of the note about breadboards** 

- Now **change the resistor** to $`10K\Omega`$ and notice the difference in LED's light. Calculate the current and compare it to the one shown on the power supply. What do you conclude? Note: This time, the LED will drop about 1.77 V (**3.5 points**). 

- Play around with different values of the resistor. What happens when the value of the resistor is too low say $`1\Omega`$? With a multimeter, measure how much voltage will be dropped on the diode this time? (**3.5 points**)  

**Make sure to take screenshots/photos and write notes as they will be helpful for writing the lab report.**

## Warm-Up Activity #2: Make your own variable resistor using pencil lead (All points: 9, Summary: 2 points, Each question according to the below points)

Pencil lead, being made of **graphite**, a form of carbon, is an excellent material to create a makeshift resistor. Actually, **carbon** is the material that most resistors that we are familiar with are made of. A typical fixed resistor can be seen in the following figure:

<figure>
<p align="center">
<img width="246" alt="carbon resistors" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/957d0496-ea46-41b5-ae14-dae57659f91d">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Carbon** is also used in the structure of **variable resistors** known as **potentiometers**. 

- Get your pencil and create a **strip of carbon** with pencil lead drawn back and forth. This will create a resistive path using the graphite from the pencil, which acts as a form of variable resistor (**3.5 points**).
- Now get your **multimeter** and put it in the **Ohm setting**. **Measure the resistance** of this strip **at different points** along it. What is your observation (**3.5 points**)?

As we saw in the warm-up activities above, a **resistor** is an essential component in an electrical circuit that **restricts the flow of electricity**, thereby **controlling** and managing the **current**. This ability to modulate current is crucial for **protecting** **sensitive components** like LEDs and ensuring the circuit functions as intended. With this understanding, we can now delve deeper into the specifics of resistors, their types, applications, and selection criteria in this lesson. 

## Understanding Fixed Resistor Color Codes: 3, 4, and 5 Band Resistors (All points: 12.5, Summary: 2 points, Each question according to the below points)

**Fixed resistors** are marked with **colored bands** that indicate their **resistance value** and **tolerance**. The color codes can be extracted from the **Color Code Chart** below: 

https://neurophysics.ucsd.edu/courses/physics_120/resistorcharts.pdf

Here's a guide to understanding the **color coding system** for 3, 4, and 5 band resistors.

### 3-Band Resistors

These are the **simplest** type, typically found in **older or less precise** applications. To read their value follow these steps:

- The **first two bands** indicate the **first two digits of the resistance** value.
- The **third band** (multiplier) indicates the **power of 10** which is the number that the two digits should be multiplied.
- They have **±20% tolerance**

**Class Activity:** Find the resistance value of the following resistor. Express this value as a range based on the resistor's tolerance (**3.5 points**). 

<figure>
<p align="center">
<img width="219" alt="3 band resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/c8543390-9f84-410d-a8fd-0da3f2f2daf0">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### 4-Band Resistors

These are **more common** and include a **tolerance band**. To read their value, you can follow the following steps:

- The **first two bands** represent the **first two digits** of the resistance value.
- The **third band** (multiplier) functions the same as in 3-band resistors.
- The **fourth band** (tolerance) indicates the **tolerance** of the resistor, which is the range within which the actual resistance value may vary. **Gold (±5%)** and **Silver (±10%)** are common tolerance colors.

**Class Activity:** Find the resistance value of the following resistor. Express this value as a range based on the resistor's tolerance (**3.5 points**). 

<figure>
<p align="center">
<img width="236" alt="4 band resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/582b5d94-62f5-4c3f-83c3-d5366c7029a5">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### 5-Band Resistors (precision resistors)

These offer **higher precision** with an additional digit for resistance value. This is how you can read their values:

- The **first three bands** indicate the first **three digits** of the resistance value.
- The **fourth band** (multiplier) functions like the third band in 3 and 4-band resistors.
- The **fifth band** (tolerance) is the same as the fourth band in 4-band resistors, providing the **tolerance** value. The tolerance band here **can be any of the eligible colors** and **not just gold or silver**. For example, if the fifth band is brown, then the actual resistance is within ±1% of the value. You can extract the tolerance value for different colors from the color code chart below:

https://www.codrey.com/tools/resistor-color-code-calculator/ 

In 5-Band Resistors, you maybe confused about which end to read the value because the **5th band can be any color**. There is no straightforward way to solve this but you can look at the manufacturer's information or even better **measure it using your multimeter**. 

**Class Activity:** Find the resistance value of the following resistor. Express this value as a range based on the resistor's tolerance (**3.5 points**). 

<figure>
<p align="center">
<img width="364" alt="precision resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/245ca08c-537c-46f6-ae9a-e7d2a7559a31">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Note that a **schematic drawing** for a **fixed resistor** is as follows:

<figure>
<p align="center">
<img width="255" alt="schematic diagram resistance" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/d6c30978-9dcb-4e1a-a48f-331424facf68">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

<!--**Class activity:** Calculate the **resistance value** of some of the resistors in your kit using the **color code chart** and **verify** that using your **multimeter**.--> 

## Series and Parallel Resistors (All points: 29, Summary: 2 points, Each question according to the below points)

Resistors can be connected in a circuit in two ways: in **series** or in **parallel**. Each configuration has its own characteristics and rules for calculating the total resistance.

### Series Resistors

In a series circuit, resistors are connected end-to-end, so the **same current** flows through each resistor. 

<figure>
<p align="center">
<img width="383" alt="resistors in series" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/196a6789-52e7-4cca-88c0-2c989f3f39a6">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The **total resistance** in a series circuit is the **sum of all individual resistances**. If you have resistors $`R_1, R_2, R_3, ..., R_n`$ connected in series, the total resistance ($`R_{total}`$) is given by:

$`R_{total} = R_1 + R_2 + R_3 + ... + R_n`$

The **voltage drop** across each resistor in series is different and **depends on its resistance**. According to Ohm's Law, V = RI, where V is voltage, I is current, and R is resistance. The **same current** flows through each resistor in a series circuit.

**Class Activity: Series resistors**

- Simulate two resistors of the values of $`1k\Omega`$ and $`2k\Omega`$ in series in Tinkercad and read the resulting resistance using a multimeter (**3.5 points**). 

<!--Implement the same with a bunch of resistors on your breadboard and verify the simulation result with your multimeter as well.--> 
 
- Connect the resistors to a **power supply of 5 V** and calculate and verify the **voltage drop** by each resistor. What do you observe? Make sure to prove mathematically what you see on the multimeter (**3.5 points**).

**Class Activity: Voltage Divider (3.5 points)**

Consider the following circuit:

<figure>
<p align="center">
<img width="367" alt="voltage divider" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/fb2ca34e-85a1-48ba-b95c-b67bd3194e99">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The power source supplies 15 v but we want the **max voltage going to the computer** to be 3.3 v. This means we should drop 3.3 v across the second resistor and consequently 11.7 v across the first resistor. Suppose we choose a nice **high resistor** (10k) for the second resistor. Based on these information, **design** the value of the **first resistor** so that you can supply 3.3 v to the computer. 

**Note:** This circuit is called a **voltage divider** where we divided the incoming voltage into usable pieces. 

### Parallel Resistors

In a parallel circuit, resistors are connected across the same two points, and each resistor has the **same voltage** across it ($`V_{source} = V_1 = V_2 = .... = V_n`$).

<figure>
<p align="center">
<img width="371" alt="resistors in parallel" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/ab18b101-9b98-4c64-a115-28a9f067bfdc">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The total resistance in a parallel circuit is found by taking the **reciprocal** of the sum of the reciprocals of each individual resistance. For resistors $`R_1, R_2, R_3, ..., R_n`$ in parallel, it is given by:

$`\frac{1}{R_{total}} = \frac{1}{R_1} + \frac{1}{R_2} + \frac{1}{R_3} + ... + \frac{1}{R_n}`$

The **total current** entering the parallel combination (total current drawn) is equal to the **sum of the currents** through each resistor (drawn by each resistor) ($`I_{total} = I_1 + I_2 + .... + I_n`$). Ohm's Law applies to each resistor individually.

**Class Activity: Parallel resistors**

- Simulate (**3.5 points**) two resistors of the values of $`1k\Omega`$ and $`2k\Omega`$ in parallel in Tinkercad and read the resulting resistance using a multimeter (show this by calculation (**3.5 points**) as well). 

<!---Implement the same with a bunch of resistors on your breadboard and verify the simulation result with your multimeter as well.---> 

- Connect the resistors to a power supply of 5 V and **calculate and verify (using a multimeter)** the **current** passing through the **combination** of two resistors (**3.5 points**). Now calculate and verify using a multimeter the current passing through **each** resistor. What do you observe (**4 points**)? 

**Note about measuring the current using the multimeter**

To measure the current, you need to **connect the multimeter in series with the circuit component**. This means you have to **open the circuit** and insert the multimeter such that the **current flows through the multimeter**.

**Important Notes for Real Multimeters:** 

- Be aware of the **current rating of the circuit** and **ensure** your multimeter can handle it. **Exceeding the current rating of the multimeter can lead to blowing a fuse or damaging the multimeter**. If unsure about the current, start with the **highest current setting** and **then move to lower settings** as needed.
- **Never try to measure current directly across a power source like a battery or a power supply**; this **will create a direct short** and could damage the multimeter or cause injury.
-  Always **turn off the multimeter** after use to **conserve battery life** and prevent accidental damage to the multimeter or the circuit you're testing. 

**End of the note about measuring the current using the multimeter**

- Suppose you need a $`1k\Omega`$ resistor but you have two $`2k\Omega`$ resistors. What would you do (**2 points**)?

## Important Ratings for Resistors (All points: 5.5, Summary: 2 points, Each question according to the below points)

Resistors come with various ratings that determine their suitability for different applications. Here are the primary ratings to consider:

1. **Resistance Value and Tolerance.** As we saw this at the start of this Labsson, the resistance value is measured in ohms ($`\Omega`$), and it indicates the **amount of resistance** a resistor provides to the **flow of current**, and the **tolerance** is expressed as a **percentage** (e.g., ±1%, ±5%) which indicates how much the actual resistance can **vary** from the stated value. A **smaller tolerance** means more **precision**.

2. **Power Rating.** Measured in watts (W), it specifies the **maximum power** the resistor can **dissipate without damage**. Common values are 1/8W, 1/4W, 1/2W, 1W, etc. Using a resistor with an **insufficient power rating** can lead to **overheating** and **failure**. Resistors directly convert the **power** they drop into **heat** so they need to be big enough to dissipate the heat without burning up. 1/4W resistors are the most common resistors and the resistors in our kit are of this type. 

**Example:** Suppose we have a 10 ohm resistor and we run 200 mA through it. 

<figure>
<p align="center">
<img width="423" alt="power rating for resistors" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/830abbfa-097c-4dd7-841f-de65235f297d">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The voltage drop across the resistor will be 2 V. Neither of the voltage or current is high, but let's calculate the power:

$`P = VI = 2V\times 0.2A = 0.4 W`$ 

This power is larger than the 0.25 W rating which will lead to burning out the 1/4 W resistor. 

Note: The **wattage rating** of a resistor determines its **size**; **higher wattage** resistors are **larger** to provide more surface area for efficient heat dissipation.

<figure>
<p align="center">
<img width="400" alt="resistors with different wattages" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/29fc1302-f23a-4397-8e98-b985bee7fea0">
<figcaption> <p align="center">Photo credit: www.electronicshub.org</figcaption> </p>
</p>
</figure>

3. **Temperature Coefficient.** This rating indicates how much the **resistance value changes** with **temperature**, usually expressed in **parts per million per degree Celsius** (ppm/°C). A **lower coefficient** is preferable for applications requiring stable resistance across temperature variations. Parts per million here refers to **very small change in resistance** relative to the resistor's **total resistance**. As an example, if you have a 1,000-ohm resistor with a temperature coefficient of 25 ppm/°C, and the temperature increases by 1°C, the resistance would change by 0.025 ohms (25 parts per million of 1,000 ohms).

4. **Voltage Rating.** It specifies the **maximum voltage** that can be applied across the resistor **without causing breakdown or damage**. This is often related to the physical size of the resistor, as larger resistors can handle higher voltages.

**Class Activity: Finding resistor ratings**

Choose a resistor from the list below and read and explain its ratings (**3.5 points**):

https://www.digikey.com/en/products/filter/through-hole-resistors/53

## Note on power supplies and how to choose them (Summary: 2 points)

The power supply works by first **stepping down** the input AC voltage to a lower level suitable for the system using a **transformer**. After this, the rectifier **converts alternating current (AC) to direct current (DC)**. It typically employs **diodes** to allow current to pass in **one direction** only, turning AC into pulsating DC. After rectification, the pulsating DC is still **not smooth**. A **filter capacitor** is used to **smooth** out these pulses to produce a more **stable DC output**. After this, a **voltage regulator** ensures a **stable** and consistent output. The regulator **adjusts** the voltage to **the desired level** and maintains it. This **compensates for fluctuations** in **input voltage** or **changes in load**. 

<figure>
<p align="center">
<img width="788" alt="power supply schematic" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/2cbe937c-a9ab-4be7-a33d-4e0331341ee2">
</p>
</figure>

The power supply that we have can operate on either **110V or 220V input voltages** that cater to different regional power standards (**please pay attention that before using it choose the proper standard** - choosing 220 here in the US can damage your power supply but not choosing 110 in Europe where the standard is 220 V). It has an adjustable **output voltage range** of 0-48 V DC that is suitable for loads requiring voltage in this range. The **current specification** is at maximum 10 A. It's important to note that the **current output** is **dependent** on the **load (device)** and is **not adjustable**. This maximum current specification is suitable for devices needing up to 10A.

<figure>
<p align="center">
<img width="282" alt="power supply" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/15b3aefa-feba-4b18-9e70-aad259b1c71b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

If you open this power supply, you can find the components below:

<figure>
<p align="center">
<img width="405" alt="inside a power supply" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/b3705af5-990c-4ee7-9c91-1645b9557af8">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The **large yellow component**, with a metal core surrounded by coils of wire is the **transformer** that steps down the input voltage to a lower level. **Rectifier** is not very visible but typically includes a **series of diodes**; it should be on the circuit board, likely near the transformer, to **convert AC to DC**. The large cylindrical black components are **filter capacitors** that smooth out the DC current after rectification. The **green board** is the **control circuitry** that manages the output and protection features.

So, with the above explanation about our current power supply, we can see the important factors on choosing the right power supply for our application. The key factors to consider when selecting a power supply are **voltage** and **current**:
- Ensure the power supply can provide the **correct voltage levels required by your components** (motors, sensors, microcontrollers, etc.).
- Choose a power supply that can **handle** the **maximum current draw of your system**. Consider the **peak current requirements**, not just the average, to avoid **overloading** the power supply.

## Variable Resistors (Potentiometers or Pots) (All points: 16, Summary: 2 points, Each question according to the below points)

Variable resistors are components whose resistance can be adjusted within a **certain range**. You built one yourself in the warm-up activity. The most common type is the **potentiometer**, often referred to simply as a "pot." Potentiometers find applications in controlling electrical devices where **adjustments** of **current** or **voltage** are needed.

A potentiometer consists of a **resistive element**, typically a **carbon** or **metal film**, and a **movable wiper**. The wiper slides over the resistive element, changing the effective resistance.

<figure>
<p align="center">
<img width="480" alt="potentiometer cross section" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/84c32943-8306-4459-9e5d-f83d0dd0fe6f">
<figcaption> <p align="center"</figcaption> </p>
</p>
</figure>

The **schematic diagram** of a potentiometer is:

<figure>
<p align="center">
<img width="234" alt="pot schematic diagram" src="https://github.com/madibabaiasl/mechatronics-course-private/assets/118206851/d7a3e40b-109b-4b94-aef5-effe9ec32585">
<figcaption> <p align="center"</figcaption> </p>
</p>
</figure>

As the **wiper moves along the resistive material**, it **varies the resistance** between the **wiper** and **each end of the resistor**. This allows **control** over the **current flow** or **voltage output**.

For example if the wiper is in this position:

<figure>
<p align="center">
<img width="202" alt="pot in a certain position" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/4f3e13d6-e3cc-4ddf-82f8-f47f02bfe73b">
<figcaption> <p align="center"</figcaption> </p>
</p>
</figure>

Then the **resistance between the middle pin and the left pin** is **lower** than the **resistance between the middle pin and the right pin** because you guessed it right there is **less resistive material**. The **resistance between two outer pins** is the **max resistance** and it is **equal to the value of the pot**. 

In **linear Potentiometers**, the resistance changes **linearly** with the movement of the wiper, and **logarithmic Potentiometers** change resistance **logarithmically**, often used in **audio for volume control** as the human ear perceives sound logarithmically. For example, if you have a 10kΩ linear potentiometer and the wiper is at the midway point, the resistance from one terminal to the wiper and from the wiper to the other terminal is 5kΩ each. **Ohm's Law** (V = IR) is still **valid** here. 

The wiring digram for a Pot can be depicted as the figure below:

<figure>
<p align="center">
<img width="449" alt="wiring diagram pot" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/5877d0de-5dcc-4bf8-a280-4bbc30d9a84f">
<figcaption> <p align="center"</figcaption> </p>
</p>
</figure>

**Class activity: Exploring Potentiometer Behavior**

- Get a known potentiometer and first measure the resistance between the outer terminals. How much is it and why (**3.5 points**)?
- Now turn the wiper half way and measure the resistance between the middle terminal and each of the outer terminals. How much is it and why (**3.5 points**)? 

**Class activity: Control the brightness of an LED using a pot**

- Use an LED, a 1kΩ potentiometer, a power supply (set it to 4 V), and connecting wires to make an LED dimmer. Connect the potentiometer in series with an LED. Apply 4 V voltage to the circuit. Adjust the potentiometer and observe the change in LED brightness. Here is the schematics of the circuit (**3.5 points**):

<figure>
<p align="center">
<img width="406" alt="controlling LED brightess with Pot 2" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e4824173-a66e-4a36-972e-6785446dfdf7">
<figcaption> <p align="center"</figcaption> </p>
</p>
</figure>

Note that in practice, we also use **another fixed resistor** to protect the LED from over current but for the sake of this activity, we used just a potentiometer. 

- Replace the pot with your handmade variable resistor (or pencil lead directly), and see if it also works (**3.5 points**). 

## Instructions for the Labsson 2 Report

- The grading criteria are as follows:
  - each activity and question according to respective points mentioned throughout the text (total = 90 points)
  - a conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - references (disclose the use of AI) (5 points)

- Some useful notes:
  - Keep it concise and to the point.
  - You can use any text editing software that you are comfortable with, like **Google Docs** or **Latex**. 
  - make sure to provide photos of the activities done when needed
  - include the labsson title and your name in the report 