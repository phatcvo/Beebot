## Electricity, Conductors and Insulators 

Electricity, at its core, is all about the **movement of electrons**. These subatomic particles are not just orbiting the nuclei of atoms; they are the key players in the **flow of electric charge**, especially within conductive materials. This **flow of electrons** is what we commonly refer to as **electricity**. 

![flow of electricity](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/94086170-42f7-4313-8d20-700a3951ce0a)

These electrons can jump from one atom to another, a process known as electron transfer. This movement is crucial in the **conduction** of electricity in materials. In **conductors**, such as **metals**, electrons move freely and transfer energy, which manifests as **electrical current**. In **insulators**, electrons are tightly bound to their atoms and do not move freely. This lack of free electron movement means that insulators do not conduct electric current well. They are used in electrical systems to prevent unwanted flow of current to provide protection and enhance safety. Materials like **rubber**, **glass**, and certain **plastics** are common examples of insulators.

Note here that there is no such thing as a **perfect non-conductor** of electricity. While materials commonly referred to as insulators, such as rubber, glass, and plastic, **significantly resist the flow of electrons**, they are not absolute barriers to electrical conductivity. Under certain conditions, such as **extremely high voltages**, even these materials can conduct electricity to some extent.

What about **water**? Is water a conductor or an insulator? 

### Class Activity: Conductivity of Water and Saltwater (all points: 17, summary of what is done: 5 points)

**Materials needed:** 
- Safety goggles
- Two containers 
- Pure water
- Table salt
- a spoon for stirring
- a digital multimeter with probes

**Safety First:**
- Wear the safety goggles.
- **Never** touch the water while the multimeter probe is in water. Note that the multimeter measures resistance by passing **a small known current** (from the internal battery) through the component or material being tested and then **measuring the voltage drop** across it (Ohm's law). 

**Now measure Conductivity:**
- Set the digital multimeter to measure resistance.
- Record the resistance for distilled water (**4 points**).
- Repeat the same process with the saltwater solution (**4 points**).

What is the difference between pure water resistance and saltwater resistance? What do you conclude? (**4 points**) 

## Voltage, Current, Resistance, Ohm's law, and Power

Electricity can be measured in two different ways: by **current** and by **voltage**

**Current Measurement (Amperes):** Current is measured in amperes (A), which quantifies the **flow of electrons** that carry electric charge.

**Voltage Measurement (Volts):** Voltage is measured in volts (V) and represents the **potential difference** between two points in an electric field. It is basically the **pressure** that pushes the electrons that carry electric charge to move. In other words, voltage is the **driving force** that causes electric current (the flow of electrons) to flow in a circuit. Without sufficient voltage, electrons will not move through the material. 

These two measurements are interconnected by **Ohm's Law**, which states that $`V = RI`$, where $`R`$ is the resistance in ohms ($`\Omega`$). As a simple example, if a device has a resistance of $`4\Omega`$ and carries a current of $`3A`$, then the voltage across the device would be: $`V = 3 A \times 4 \Omega = 12 V`$. 

**Resistance** in an electrical circuit can be likened to **friction** in a mechanical system. Just as friction opposes the motion of objects moving across a surface, electrical resistance opposes the flow of electrons through a material. It's a measure of how much a material resists the flow of electric current.

![voltage-current-resistor](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/84d738e9-9fb8-4c66-9199-e99e0c08de2b)

### Class activity: Voltage, Current, and Batteries (34 points in total) 
- How much is the voltage that pushes the electrons in an AA and AAA battery? (**2 points**)

- Now get your multimeter (set it to **DC voltage** setting) and measure the voltage of both AA and AAA batteries. Are the numbers the same as your speculation? (**2 points**)

- AA batteries are larger than AAA batteries, so why are their voltages the same? (**3 points**)

- Consider the human body's resistance to be $`10,000 \Omega`$, what is the current resulting from touching the ends of the AA or AAA battery? (**3 points**) 

- The human body can typically feel an electric current above 1 mA, and pain is felt at currents above 5 mA. However, the current needed for serious harm is much higher and usually requires voltages far above what standard batteries can provide. Based on this, discuss why it’s safe to touch the ends of the battery with your fingers. (**3 points**)

- 9 v batteries in your kits typically comprise six individual 1.5V cells connected in series. These cells are often similar to the AAAA size, smaller than AAA, and are enclosed within the 9V battery's casing. By connecting these cells **in series**, their individual voltages add up, resulting in a total output of 9 volts. 

<figure>
<p align="center">
<img width="290" alt="9v battery casing 6 one half v" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/1b4633d0-ad1d-49a1-b4fb-4fef2586f78b">
<figcaption> <p align="center">a 9V battery comprises six AAAA size batteries.</figcaption> </p>
</p>
</figure>

While the **voltage** of the battery is **increased** by this series configuration, the **capacity** (the amount of charge the battery can store) remains **the same** as that of an individual cell. Hence, 9V batteries typically have a **lower capacity** than AA or AAA batteries. By this explanation, what application do you think $`9V`$ batteries are suitable for? (**3 points**)  

- A car battery is a 12 V battery (which is not a high voltage) but why **short-circuiting** the poles of the car battery can be **dangerous**? What do you conclude? (**3 points**)

- Another battery type commonly used is **Lithium-ion** batteries. Lithium-ion batteries typically have a **higher voltage per cell** compared to alkaline batteries. A single lithium-ion cell has a nominal voltage of about 3.7 volts. They can store more energy for their size or weight than most other types of batteries. Based on this information, where do you think they are most suitable to be used? Why are AA and AAA batteries not suitable for these applications? (**3 points**)

<figure>
<p align="center">
<img width="199" alt="lithium ion battery" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/d569a6d5-b657-4bc6-8779-445e04fb68c9">
<figcaption> <p align="center">a Lithium-ion battery with nominal voltage of about 3.7 V.</figcaption> </p>
</p>
</figure>

**Important safety tip:** Lithium-ion batteries should be charged correctly because improper charging can lead to various safety risks, including overheating, battery damage, and in extreme cases, fires or explosions. It's crucial to use the correct charger and follow the manufacturer's guidelines for charging lithium-ion batteries. Overcharging or using an incompatible charger can cause the battery to become unstable, potentially leading to thermal runaway where the battery overheats and can violently release its stored energy. Additionally, always charge lithium-ion batteries in a safe area away from flammable materials and regularly check for signs of damage or unusual behavior.

- Compare the **capacity (mAh)** of alkaline batteries (AA and AAA) and Lithium-ion batteries. What does a capacity of 1200 mAh mean? (**3 points**) 

- How does the **current draw** of a device affect the duration a battery will last, and what happens to the battery's depletion rate when the current draw is higher? (**3 points**)

- Get one of your LEDs and try to light it up using one AAA battery. Can you do that? Why? Now take two AAA batteries and connect them in series. How about now? Read (or measure with a multimeter) the forward voltage requirement for the LED and answer based on that voltage. (**3 points**) 

- Based on what you have learned thus far, summarize **how to choose the best battery** for a specific application. What are the **two important factors** that you should consider when choosing a battery for your application? (**3 points**) 

**Important Safety Information for Batteries:** Store batteries in a secure location, ensuring that there are no exposed wires or metal objects that could accidentally short-circuit the battery poles. This can potentially cause a fire.

Note that the **schematic drawing for a battery** in circuits is as follows: 

<figure>
<p align="center">
<img width="133" alt="schematic diragram for batteries" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/d794452a-689b-4840-b5b4-2df96e6bf2e4">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### Power in Electrical Systems (17 points total, summary of what you have learned = 5 points)

In the context of electrical systems, power is defined as the **rate at which electrical energy is converted into another form of energy**, such as heat, light, or mechanical energy. It is a measure of how much work can be done by the electricity over a certain period of time. The standard unit of electrical power is the watt (W), which is equivalent to one joule per second.

In a Direct Current (DC) system, power (P) is calculated as the product of voltage (V) and current (I): $`P = VI`$. This equation illustrates that power is directly proportional to both the voltage, which drives the electric charge, and the current, which is the flow of the electric charge.

**Example:** If a DC circuit has a voltage of 12 volts and a current of 2 amperes, the power would be $`P = 12V \times 2A = 24W`$. This means the circuit is **consuming** or **delivering** 24 watts of power. 

**Example:** The power in a DC circuit indicates the rate at which **energy is used or supplied**. For example, a battery **discharging** at a rate of 24 watts is **supplying** energy to the circuit at that rate. 

**Class activity:** Consider a 24W device that is powered by a 12V battery with a capacity of 24 ampere-hours (Ah). Based on these specifications, calculate how long the device could theoretically run on a full charge. (**2 points**)

**Example:** Power **dissipated** in a DC circuit, especially in **resistive** components like resistors, often manifests as **heat**. The power dissipated in a resistor can be calculated using $`P = RI^2 \text{ or } P = \frac{V^2}{R}`$, where R is the resistance.

**Class Activity:** In the circuit below, calculate the power dissipated by each resistor. (**4 points**)

<figure>
<p align="center">
<img width="254" alt="power dissipation by each resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/6dacd6bd-211a-4e72-8d22-c67850e4b951">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Class Activity:** Your laptop has a power adapter that specifies an output of say 19 volts and 3.42 amperes. Using this information, calculate the power draw of the laptop in Watts (**4 points**). Additionally, discuss how the laptop's power consumption might vary under different usage conditions, such as when it's performing intensive tasks compared to when it's in idle mode (**2 points**).

Now let's see what the two fundamental types of electricity are. 

## DC and AC Electricity 
 
There are two fundamental types of electricity: **Direct Current (DC)** and **Alternating Current (AC)**. DC is the unidirectional flow of electric charge (from negative to positive in batteries like the circuits above), often seen in **batteries** and solar cells. On the other hand, AC is characterized by the **periodic reversal of the direction of electron flow**, as commonly used in household power supplies.

In DC, the electric charge flows in **one direction**. This is in contrast to AC, where the charge **changes direction periodically**. DC typically maintains **a steady voltage level** over time, making it suitable for applications that require a stable and consistent voltage. The most common sources of DC power are **batteries**. Batteries convert chemical energy into electrical energy, providing DC voltage.

**Alternating Current (AC)**, on the other hand, is a form of electric current where the flow of electric charge periodically reverses direction. In AC, the direction of the current flow **changes back and forth**, usually in a **sinusoidal** waveform, though it can also be triangular or square in certain applications. 

![AC electricity](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/0bb7c94d-9ff7-4eb2-ad90-18328839d470)

**Frequency** is a key characteristic of AC and refers to the number of times the current changes direction per second. It's measured in hertz (Hz). For example, in the United States, the standard frequency is 60 Hz, while in many other parts of the world, it's 50 Hz. So, it means that the polarity between two poles of a socket reverses 60 times per second. In an AC power supply, like a standard household outlet, the current alternates or 'swings' back and forth. This oscillation happens at a frequency of 60 Hz in many countries, including the United States, meaning the direction of the current changes from positive to negative and back again 60 times each second. 

The **amplitude** is another characteristic of an AC waveform which is its peak value (either positive or negative).  In a sinusoidal AC wave, which is common in power systems, this is the maximum extent of the wave above or below the zero line.

**Interesting Historical Context:** DC was the first type of electrical current to be widely used. Thomas Edison was a major proponent of DC in the early days of electricity distribution, leading to the famous **"War of Currents"** between Edison's DC systems and Nikola Tesla's AC systems. AC eventually became more popular for general power distribution due to its ease of transforming voltages.

<figure>
<p align="center">
<img width="299" alt="Edison and tesla" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e5728a58-2168-40eb-847d-a6eedb9439b5">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>


### Example: An Outlet in the United States

- In the U.S., a standard household electrical outlet typically has two primary **poles or slots**, which are referred to as the **"hot" pole** and the **"neutral" pole** and a third part called the **"ground"** (or "earth"). The Hot Pole (or Hot Wire) carries the **live or active electrical current**. Neutral Pole (or Neutral Wire) **completes the circuit** by providing a path for the electrical current to **return to the electrical source** after passing through the device. The **Ground Pole** (or Ground Wire) is a **safety** feature designed to **protect against electrical shocks**. The ground pole is **connected to the earth** and provides a direct path for the electrical current to the ground in case of a **fault**, like a short circuit. 
- In the U.S., a standard household electrical outlet typically has a voltage of about **120 volts AC** between the "hot" pole and the neutral pole. 

<figure>
<p align="center">
<img width="200" alt="outlet in us" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/06ce227f-a1c9-4d1e-958b-69a70bbe236e">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

AC is generated by **AC generators** which produce electricity by **rotating a coil within a magnetic field** to induce an alternating current. AC's voltage can be easily increased or decreased using **transformers**, making it ideal for long-distance transmission. High voltages (and low currents) are used for transmission to **reduce energy loss** ($`P = RI^2`$), and then the voltage is stepped down for safe usage in homes and businesses.

## Practical Problem (17 points in total)

Suppose, you are part of a team tasked with designing a new **quadcopter drone** intended for aerial photography. 

<figure>
<p align="center">
<img width="200" alt="a drone for photography" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/3bbbbc6c-297e-4a9c-81c7-3785eb27c8ff">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The drone is equipped with **four motors**, each consuming an average of **15 watts of power** during flight. The drone also includes a **flight controller** and **FPV (First Person View) system** that collectively consume an **additional 10 watts**. The drone's operational requirements state that it should be capable of at least **20 minutes of continuous flight time**. Therefore your design parameters are as follows:

- Each motor's power consumption: 15W
- Flight controller and FPV system's power consumption: 10W
- Desired flight time: 20 minutes
- Drone's total weight capacity for the battery: 300 grams
- Battery voltage (you may choose a standard voltage for multirotor drones, e.g., 11.1V for a 3S LiPo battery)

Based on these:

1. Calculate the **total power consumption** of the drone. (**2 points**) 
2. Determine the **total energy** required for 20 minutes of flight. Express this in terms of Watt-hours. (**3 points**) 
3. Choose a battery (**3 points**) that will provide the necessary energy while not exceeding the weight capacity of the drone (calculate the minimum needed capacity first (**3 points**)). You may refer to a list of common battery capacities and weights online. Give justification why that battery is a suitable design choice. (**3 points**)
4. Calculate the **current draw** from the battery and verify if the chosen battery can safely provide this current (choose a 20% safety margin). (**3 points**)

**NOTE:** In **rechargeable batteries** like **Lithium Polymer (LiPo)** or **Nickel-Metal Hydride (NiMH)**, there is a concept called the  **discharge rate of the battery**, often represented as a **"C" rating**. It determines **how quickly the battery can release its energy**. It's a key metric for applications like drones, where power needs can vary significantly. The C-rating is a measure of the rate at which a battery can be discharged relative to its maximum capacity. For example, a **1C discharge rate** means the battery can be discharged at a **current** that will deplete its entire capacity in **one hour**. Similarly, a **2C rate** would deplete the battery in **half an hour**, and a **0.5C** rate would deplete it in **two hours**. You can calculate the maximum discharge current by multiplying the C-rating by the battery capacity. For example, a 1000mAh battery with a 1C rating can provide 1000mA, or 1A, for one hour. If the same battery has a 2C rating, it can provide 2000mA, or 2A, but only for half an hour. You can take into consideration this rating as well when choosing your battery. 

## Instructions for the Labsson 1 Report

- The grading criteria are as follows:
  - an abstract summarizing what you have learned in this labsson (5 points) 
  - each activity and question according to respective points mentioned throughout the text (total = 85 points)
  - a conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - references (disclose the use of AI) (5 points)

- Some useful notes:
  - Keep it concise and to the point.
  - You can use any text editing software that you are comfortable with, like **Google Docs** or **Latex**. 
  - make sure to provide photos of the activities done when needed
  - include the labsson title and your name in the report 

Good Luck!