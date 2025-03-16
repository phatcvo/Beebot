## Capacitors

### Introduction

Capacitors are fundamental components in electronics. Capacitors are ubiquitous in electronic circuits, and their importance in mechatronics cannot be overstated. As devices that **store and release electrical energy**, they find extensive use in **filtering**, **timing**, and **energy storage** applications.

<figure>
<p align="center">
<img width="472" alt="capacitors" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/80fe9ca4-d791-4e1b-9b6b-f08b68c8deb9">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

At its core, a capacitor is a simple device consisting of **two conductive plates** separated by an **insulating material** known as the **dielectric**. When a voltage is applied across these plates, an electric field is created, allowing the capacitor to store energy in the form of an **electrostatic charge**. This ability to **store and release charge** makes capacitors invaluable in managing power supply fluctuations that can provide stability in sensitive electronic circuits and controlling the timing and frequency of electrical signals.

**Capacitance** is the fundamental property defining how much charge a capacitor can store. The maximum voltage the capacitor can handle without failure specifies its **voltage rating**.

Let's begin with a warm-up activity to get a hands-on feel of what capacitors are and how they function.

### Warm-up Activity #1:  Build your own simple capacitor (All points: 17, Summary with a photo of your finished capacitor: 5 points, Each question according to the below points)

In this activity, you will construct a **simple capacitor** using **everyday materials**. This will help you understand the **basic principles** of how a capacitor works, including concepts of **capacitance**, **dielectric material**, and the effect of **surface area** and **separation distance** on capacitance.

**Safety Notes:**
- Be **careful** when handling the **aluminum foil** to avoid **cuts**.

**What you will need:**
- Aluminum foil (two sheets)
- Parchment paper (two sheets): they must be **longer** and **wider** than aluminum foils to avoid the foils touching each other
- Two electrical leads
- Multimeter capable of measuring capacitance
- ruler, scissors, tape, large-sized straw

**What you should do:**
- Cut two rectangular pieces of **aluminum foil**, each measuring about 55 cm long. These will act as the **capacitor plates**.
- Cut two pieces of **parchment paper** slightly **larger** than the foil. The parchment papers will act as the **dielectric** – the insulating layer between the two conductive plates.
- Place the parchment paper on a flat surface.
- Lay one piece of aluminum foil on top of the parchment paper.
- Place the second piece of parchment paper over the first foil sheet, ensuring it fully covers it.
- Finally, place the second foil sheet on top of this layer. Make sure the foil sheets do not touch each other; they should be separated by the parchment paper.
- Tape one **electrical lead** to each piece of foil. Ensure **good contact between** the clip and the foil.
- Utilize a single large-sized straw to carefully roll the sheets, ensuring that the leads remain accessible and extend outward. Tape the roll.  
- Measure the **capacitance** using your **multimeter** set to measure **capacitance**. It will likely be in the range of a **few nanoFarads (nF)** Take a picture of your measurement (**3 points**).

My **fancy** capacitor looked something like this:

<figure>
<p align="center">
<img width="472" alt="make your own capacitor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/64ab9dd7-c950-4b8a-b0b8-fbbe761f53ec">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Answer these questions:**
- How will changing the surface area of the plates affect the capacitance (**3 points**)?
- What is the role of the **dielectric** material (**3 points**)?
- How might the **thickness** and **type** of **dielectric** material affect the capacitance (**3 points**)?

Now let's see how a capacitor works. 

### How a Capacitor Works

When a capacitor is connected to a **battery**, the battery applies a **potential difference** (voltage) across the capacitor's plates. Here's what happens:

**Charging Phase:**
- Initially, the capacitor is **uncharged**, and its **voltage is zero**.
- Once **connected to a battery**, **electrons** start **accumulating** on the plate connected to the **negative** terminal of the battery. Simultaneously, **electrons** are **removed** from the plate connected to the **positive** terminal, creating a **positive charge**.
- This **accumulation of charge on the plates** creates an **electric field** in the **dielectric**, leading to **energy storage** within the **electric field**.
- The **charging continues** until the **voltage across the capacitor** **equals the battery voltage**.
- The equation for the voltage across the capacitor during the charging phase is

$`V_c(t) = V_s (1-e^{-\frac{t}{RC}})`$

Where $`V_s`$ is the supply voltage. 

Here is an animation of what happens (note that here the dielectric material is the air):

![how capacitor works](https://github.com/madibabaiasl/mechatronics-course/assets/118206851/de767050-d582-4446-863d-fb20c8df68be)

**Discharging Phase: (note the questions with points)**
- The **dielectric** has **high resistance** but it is **not infinite**, so **over time** this causes **the capacitor to discharge**. 
- When the capacitor is **disconnected from the battery** and **connected to a circuit (like a resistor)**, it begins to **discharge**, **releasing its stored energy**.
- During **discharging**, the **voltage** across the capacitor **decreases exponentially** over time, following the equation $`V(t) = V_0 e^{-\frac{t}{RC}}`$, where $`V_0`$ is the initial voltage. 

<figure>
<p align="center">
<img width="434" alt="capacitor discharging_1" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/dbd00a78-928b-432b-a977-c901845e5f09">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Some Useful math about capacitors:**
- The amount of **charge** $`Q`$ on the capacitor is **directly proportional to the voltage** $`V`$ across it. Mathematically, $`Q = CV`$, where $`C`$ is the **capacitance** of the capacitor (usually measured in Farads (micro or nano)).
- The **energy** $`E`$ stored in a capacitor is given by $`E = \frac{1}{2}CV^2`$. 
- The **current** $`I`$ through the capacitor during **charging or discharging** can be expressed as $`I = C \frac{dV}{dt}`$, where $`\frac{dV}{dt}`$ is the rate of change of voltage across the capacitor. Based on the above formula for the voltage of capacitor during discharging, **find an equation for the current during discharging** (**7 points**). 
- In an RC circuit (a circuit with a resistor and a capacitor), the time constant $`\tau`$ is a crucial factor. It is given by $`\tau = RC`$, where $`R`$ is the resistance and $`C`$ is the capacitance. The time constant represents the time it takes for the voltage across the capacitor to reach approximately 63.2% of its final value during charging, or to drop to about 36.8% of its initial value during discharging. Prove this (**7 points**). 

**Frequency Response:**

Capacitors respond **differently** to **DC and AC** electricity. When a capacitor is connected to a DC power source like a **battery**, it starts accumulating charge. The current is **initially high** as the **voltage** difference between the uncharged capacitor and the battery is **maximum**. As the capacitor charges, this **voltage difference decreases**, leading to a **decrease in the charging current**. When the capacitor is **disconnected** from the power source and connected to a load (like a resistor), it releases its stored energy. The discharging current initially is high and decreases over time as the capacitor loses its charge (as we saw above). From here, we can conclude that **DC cannot pass through the capacitor** as it gets **blocked**. 

Now, after the capacitor is fully charged, we **reverse** the **polarity** of the **battery**. Again we get a **spike of current** which quickly **drops to zero**: 

<figure>
<p align="center">
<img width="300" alt="charge discharge capacitor battery" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/60bd81eb-329a-4606-9552-6ddfb7aa3c9b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

What if we do this **faster** like in the case of **AC electricity**? With **AC**, **before the electrons drop** to nothing and the **current goes to zero**, the **polarity is reversed** and this will **continue the current**:

<figure>
<p align="center">
<img width="303" alt="capacitor ac electricty" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/38dc3706-e248-4ebd-b179-f6df5a162a2b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Therefore, we can conclude that capacitors **block DC** but **allow AC** to pass. This property is utilized in circuits to **separate AC and DC components** of a signal. The **faster the alternating current**, the **faster it passes through the capacitor**. In fact, in **AC circuits**, capacitors exhibit **reactance** (Similar to **resistance** in DC), which varies with **frequency**. The **capacitive reactance** $`X_C`$ is given by $`X_C = \frac{1}{2 \pi fC}`$, where $`f`$ is the frequency of the AC signal. From this formula, we see that as the **capacitance increases**, the **reactance decreases**, and as the **frequency increases**, the **reactance also decreases**. This property is essential in **filtering** applications, as capacitors can block **low-frequency signals** while allowing **higher-frequency signals to pass**.

Note that the Capacitance ($`C`$) of a **parallel plate capacitor** is directly proportional to the **surface area** ($`A`$) of the plates. The formula for capacitance is given by $`C = \varepsilon \frac{A}{d}`$, where $`\varepsilon`$ is the **permittivity** of the **dielectric material** and $`d`$ is the **distance between the plates**. Increasing the plate area increases the capacitance. This is because a **larger area** allows **more charge** to be **stored** on the plates for a given voltage across the capacitor. A **larger capacitance** (due to larger plate area) means that, for the same rate of voltage change, the current will be higher. 


The schematic drawing for a capacitor in circuits is:

<figure>
<p align="center">
<img width="221" alt="capacitor schematics in circuits" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/5ab14c22-b187-4536-b10c-a9811efc25a7">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Some capacitors can be **polarized** meaning that they can have positive and negative sides like one of type of the capacitor that we have in the kit:

<figure>
<p align="center">
<img width="275" alt="polarized capacitor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/ed1aa18a-9717-4473-94b1-9f896e27a9d8">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### Capacitors Important Ratings 

Understanding the important ratings of capacitors is crucial for their **proper selection** and use in mechatronics systems. Here are the key ratings for capacitors:

1. The **Capacitance Value** that we talked about and is usually given in Farads (F), microfarads (µF), nanofarads (nF), or picofarads (pF). It indicates the **amount of charge** a capacitor **can store at a given voltage**.

2. Another important rating is the **Voltage Rating**. It specifies the **maximum voltage** a capacitor can handle. Remember I told you **anything can conduct electricity if we apply enough voltage** to it? The insulator inside the capacitor has a **breakdown voltage**. Therefore, if we **exceed this voltage**, we literally can make the capacitor a **short circuit**. Always choose a capacitor with a **voltage rating higher than the circuit's maximum operating voltage**.

3. **Polarity** (for polarized capacitors like electrolytic) indicates the correct orientation of the capacitor in the circuit. Incorrect polarity can lead to capacitor failure.

The above ratings are important ratings of the capacitors, but there are other ratings as well. For example, **tolerance** indicates how much the actual capacitance can vary from the stated value. Common tolerances are ±5%, ±10%, and ±20%. **Critical applications** may require capacitors with tighter tolerances. **Temperature Coefficient** describes how capacitance **changes with temperature**. This can be important in applications where the capacitor will experience **temperature variations**. **Equivalent Series Resistance (ESR)** is a measure of the **internal resistance** of the capacitor. This affects how **quickly** a capacitor can charge and discharge. Low ESR is desirable for **high-frequency applications**. **Leakage Current** is the small amount of **current** that flows through the capacitor when **it's fully charged**. This is important for applications where **capacitors must hold a charge for a long time**. **Dielectric Type** determines many of the capacitor's performance characteristics. Common types include **ceramic**, and **electrolytic**. When selecting a capacitor, it's crucial to consider all relevant ratings for your application. 

**Class activity:** Review several capacitors' datasheets from the link below and make sure that you understand their key ratings. Discuss how these ratings would affect the capacitor's performance in different circuit applications (**7 points**).

https://www.digikey.com/en/products/category/capacitors/3


### How to Read the Capacitance Value of Common Capacitors

Capacitor values are typically **marked on their bodies**, but the marking methods vary depending on the **capacitor type** and **size**.

For polarized **Electrolytic Capacitors** that you have in your kit and many other types of capacitors, values are **directly printed in µF** (microfarads). There also includes the **voltage rating**. In these capacitors, **polarity** is **marked**, usually with a negative (-) stripe. 

For **Ceramic Capacitors** that you also have in your kit, they usually have some digits written on them. The **first two digits** indicate the **capacitance value**, and the **third digit is the multiplier** and the result is in pF (Picofarads). For example, '104' means 10 followed by 4 zeros, i.e., 100,000 pF or 100 nF. Remember that Micro is $`10^{-6}`$, Nano is $`10^{-9}`$ and Pico is $`10^{-12}`$. 
  
### Series and Parallel Capacitance

When capacitors are connected in a circuit, they can be arranged in either series or parallel configurations. 

**Capacitors in Series**

In a series arrangement, the capacitors are connected end-to-end, with the positive plate of one connected to the negative plate of the next. The formula for calculating the total or equivalent capacitance ($`C_{eq}`$) of **capacitors in series** is similar to the formula for **resistors in parallel**:

$`\frac{1}{C_{eq}} = \frac{1}{C_{1}} +\frac{1}{C_{2}}+\frac{1}{C_{3}}+...`$

The equivalent capacitance of a series combination is always **less** than the capacitance of any single capacitor in the series. In a series circuit, the **total voltage** across the capacitors is the **sum of the voltages** across each individual capacitor. The **voltage** is **inversely** proportional to the **capacitance** ($`V = \frac{Q}{C}`$), meaning a capacitor with a smaller capacitance will have a higher voltage across it. The **same charge** flows through each capacitor in series. The **charge on each capacitor** is the **same** and equal to the **total charge**.

**Capacitors in Parallel**

In a parallel arrangement, the positive plates of all capacitors are connected together, and all the negative plates are also connected together. The formula for calculating the total or equivalent capacitance ($`C_{eq}`$) of **capacitors in parallel** is similar to the formula for **resistors in series**: 

$`C_{eq} = C_1 + C_2 + C_3 + ...`$. 

This means that the equivalent capacitance is simply the sum of the individual capacitances. The **equivalent capacitance** of a parallel combination is always **greater** than the capacitance of any single capacitor in the parallel group. In a parallel circuit, the **voltage** across each capacitor is the **same** and equal to the **total voltage** across the parallel combination. The **total charge** stored in the capacitors is the **sum of the charges** stored in each individual capacitor. A **larger capacitance** in parallel will store **more charge** at the **same voltage** ($`Q = CV`$).

### The Hidden Dangers of Scavenging Old TVs and Appliances for Electronics 

Back in the day when I was at school, electronic components were not as affordable as they are now. So, some people, especially those interested in electronics, would take apart old devices like TVs, VCRs, and microwaves to find useful parts for their projects. This might sound like a cool treasure hunt, but it was actually pretty risky!

Inside these devices, there are capacitors. By now, you know that capacitors are like energy storage tanks. Even after you unplug the device, these capacitors can still be full of electricity. Touching them could be very dangerous because they can release all that stored energy very quickly. By touching them, you can drop a huge, sudden burst of electricity through your body. At the very least, it could give you a strong shock and throw you back. But in the worst-case scenario, it could be so powerful that it stops your heart. 

## Diodes and LEDs

### Introduction 

Diodes are the simplest type of **semiconductor devices** (they are **semiconductors** because they conduct electricity under certain circumstances). At its core, a diode is a device that **allows current to flow in one direction** but not the other. This **unidirectional** behavior makes diodes invaluable in circuits for tasks like **rectification** using 4 diodes (**converting AC to DC**), **voltage regulation**, and **signal modulation**.

### Warm-up Activity: Measuring Forward Bias Voltage of a Silicon Diode and an LED (7 points - provide photos and values of the measurements)

In this activity, we want to measure the **forward bias voltage** of a **silicon diode** and a **Light Emitting Diode (LED)** using a multimeter. 

- Set the multimeter to the **diode check function**. This setting allows the multimeter to apply a **small voltage across the diode** to **measure the forward voltage drop**.
- Connect the multimeter probes to the diode (**red to anode**, **black to cathode**).
- Observe the reading on the multimeter, which should display the forward voltage drop and record it.
- Repeat the measurement process with an **LED**. 

### How a Diode Works

The heart of a diode is the **P-N junction**, formed by joining **P-type** (**lack of electrons**) and **N-type** (**excess of electrons**) **semiconductors**. This junction creates a **depletion zone** (where **electrons balance out** meaning there is no shortage or excess of electrons). When a P-N junction is formed, **electrons from the N-type** region (which has extra electrons) **diffuse** into the **P-type region** (which has **holes** or positive charge carriers), and holes from the P-type **diffuse** into the N-type. This diffusion of charge carriers results in a region around the **junction** where the **free electrons and holes have combined**, leaving behind a zone **depleted** of mobile charge carriers. The **depletion zone** acts as a **barrier** to the flow of charge carriers due to its lack of **free electrons and holes**. This **lack of mobile charge carriers** results in **high resistance** in the **depletion zone**. This **high resistance** **inhibits** the flow of **current** across the junction under **normal conditions** (when the diode is **not forward-biased**).

<figure>
<p align="center">
<img width="497" alt="semiconductor diode np junction" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/b876a1d1-5981-4367-a300-8c09eb07969f">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

When the diode is **forward-biased**, which occurs when the **anode** is connected to the **positive** terminal of a battery and the **cathode** to the **negative terminal**, the external electric field (created by the battery) supports the **movement of charge carriers** across the P-N junction. This results in **reducing the width** of the **depletion zone**. Electrons in the N-type region are pushed towards the P-type region, and holes in the P-type region are pushed towards the N-type region, effectively **narrowing the region** where they recombine. As the **depletion zone narrows**, its **resistance decreases** significantly. This **lowered resistance** allows **charge carriers** (**electrons and holes**) to cross the junction more easily. Once the **applied voltage exceeds a certain level**, known as the **forward threshold voltage**, **significant current** starts flowing through the diode. This **threshold voltage** (to **overcome the depletion zone** and to **let the charge carriers flow**) typically ranges from about **0.6 V to 0.7 V for silicon diodes**. In **forward bias**, the diode **conducts electricity**, allowing a **substantial flow of current**. This **current** consists mainly of **electrons** moving from the N-type region to the P-type region and **holes** moving in the opposite direction. 

<figure>
<p align="center">
<img width="596" alt="how diode works forward biased" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/1c47d78f-6f8c-4b41-b3eb-52e00fbfec65">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

When a diode is **forward-biased** (turned on), it exhibits a characteristic known as the **forward voltage drop**. This forward voltage drop is relatively **constant** for a **given type of diode under normal operating conditions**. For a **silicon diode**, this drop is typically around **0.6 to 0.7 volts**, while for **LEDs**, it can range from about **1.8 to 3.2 volts** depending on the **color** and **type**. When the diode is connected to a **voltage source greater than its forward voltage drop**, the **diode drops its forward bias voltage** and "turns on" and **conducts current**. This current can be high enough to **burn the diode**. It's crucial to use a **current-limiting resistor** in series with the diode, especially for LEDs, to **prevent excessive current** that can **damage the diode**. The **resistor value** is chosen based on the **desired current** and the **voltage drop** across the diode. 

In **reverse bias**, it **blocks current**. When the **cathode** of a diode is connected to the **positive** terminal of a battery and the **anode** to the **negative** terminal, this configuration is known as **reverse biasing** the diode. In this scenario, the **depletion zone widens** because the external voltage pulls the electrons in the N-type region and the holes in the P-type region **further away from the junction**. This action **increases the region** that is **depleted of free charge carriers**. As the **depletion zone widens**, its **resistance increases**. This **higher resistance** effectively **blocks the flow of current** through the diode.

<figure>
<p align="center">
<img width="596" alt="diode reverse biased" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/0333ad9b-c6f2-49f3-bbb1-9005b24ccf25">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The **current-voltage (I-V) characteristic curve** of a diode is a **graphical** representation of the relationship between the **current** flowing through the diode and the **voltage** across it. For a typical semiconductor diode, the curve shows that the diode conducts electricity primarily in **one direction**. At **low forward voltages**, there is **a small leakage current**. As the **forward voltage increases** beyond a certain **threshold** (known as the **forward voltage drop**, typically **around 0.7 V for silicon diodes and 0.3 V for germanium diodes**), the **current increases** rapidly, indicating the diode is in its **conductive state**. In the **reverse direction**, the diode exhibits **very high resistance** with a **tiny leakage current**, until a point where the **reverse breakdown voltage** is reached, beyond which there is a **sharp increase in reverse current**. 

<figure>
<p align="center">
<img width="552" alt="diode i-v characteristic" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/bf616e10-aeec-4862-a223-6e66abdfb602">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The diode in your kit is the **rectifier diode 1N4007** depicted in the figure below. Also, in this figure, you can see the **schematic drawing** for a diode. 

<figure>
<p align="center">
<img width="221" alt="rectifier diode anode cathode" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/40b15bf2-e896-4c35-87d8-ff2e7d635676">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### Zener Diodes

Zener diodes are a type of semiconductor diode that allow current to flow **not only** from its **anode to its cathode**, like **a typical diode**, but also in **the reverse direction** when the voltage across its terminals exceeds a certain value known as the "**Zener breakdown voltage**." This unique feature makes Zener diodes particularly useful for **voltage regulation purposes**. Therefore, Zener diodes are specifically designed to work in **reverse bias**. They are also designed to break down at the same voltage every time. 

When the voltage across a Zener diode is below the breakdown voltage, it behaves like a normal diode – it blocks reverse current. However, once the voltage exceeds this threshold, the Zener diode allows a significant reverse current to flow. This ability to maintain a relatively constant voltage across its terminals (even with changes in load current or supply voltage) makes it an invaluable component in circuits that require stable voltage supply, such as **power supplies** and voltage reference circuits.

The schematic drawing of a zener diode is like as follows:

<figure>
<p align="center">
<img width="191" alt="zener diode schematic" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/9c9c48b6-1ae5-4aff-9e31-5962a8a272ad">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

### Light Emitting Diodes (LEDs)

Light Emiting Diodes (LEDs) are a type of **semiconductor** device that **emits light** when an **electric current passes** through them. They are a **special class of diodes**, which harness the movement of electrons in a semiconductor material to create light. Like other diodes, LEDs are made from a **semiconductor material**, typically formed from a P-N junction. When **forward-biased**, electrons cross from the N-type material and recombine with holes in the P-type material, **releasing energy in the form of photons**, which is **visible light**.

<figure>
<p align="center">
<img width="600" alt="led semiconductor pn junction" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/b0c6cfdb-5b09-49f1-8d95-615cc9e6f70a">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>


Different **semiconductor materials** produce different **colors of light**. For example, gallium arsenide (GaAs) emits **infrared light**, indium gallium nitride (InGaN) can produce **blue light**, and a combination of materials can produce **white light**. LEDs have a **characteristic forward voltage drop** that **varies based on the color** (wavelength) of the LED. For example, red LEDs may have a forward voltage of about 1.8 volts, while blue LEDs might be around 3.0 to 3.5 volts. LEDs require a **certain range of current** to operate effectively. **Too little current** and the LED will be **dim**; **too much current** can **damage** the LED. This **current** is typically in the range of **10 to 20 milliamperes** for standard indicator LEDs.

A **resistor** is often used in series with an LED to limit the current to a safe value. The **resistor value** is calculated based on the **supply voltage**, the **LED's forward voltage**, and the desired **current**.

### How to choose the current-limiting resistor

Choosing the correct **current-limiting resistor** for an LED is crucial to ensure it operates safely and efficiently. Here are the steps to choose a current-limiting resistor for an LED: 

- Identify the LED's **forward voltage** ($`V_f`$) by measuring it using the **diode test** of the multimeter or alternatively read it from its information **sheet**. 
- Determine the LED's **forward current** ($`I_f`$) which is the **recommended operating current** ($`I_f`$) from the LED's **datasheet**. Standard small LEDs usually operate **around 10 to 20 mA**. This is the **absolute maximum current**. You should **avoid the max current** but also the current should be **high enough** to make the LED bright.
- Now use Ohm’s Law to calculate the **resistance**. The voltage drop across the resistor ($`V_r`$) is the supply voltage minus the LED's forward voltage: $`V_r = V_s - V_f`$.
- The **current through the resistor** will be the **same** as the **current through the LED** ($`I_f`$). 
- Use the **Ohm's law** to calculate the resistor needed: $`R = \frac{V_r}{I_f}`$. 
- Now **choose a resistor** with an **appropriate power rating**. Calculate the **power dissipated by the resistor** using $`P = RI^2 \text{ or } P = VI`$. Select a resistor with a power rating **greater** than the calculated value for safety. Commonly, a 1/4 watt resistor is sufficient for most small LED applications. Note that you might not find a resistor with the exact calculated value. Choose the nearest higher standard value.

**Class Activity: Designing a current-limiting resistor for an LED (make sure to follow steps above and provide pictures to get the full points)**

Suppose the following circuit:

<figure>
<p align="center">
<img width="436" alt="designing current limiting resistor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/069b045f-1afb-4c83-9b6e-2efa711bef1f">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

You have an LED and you want to design an **appropriate resistor** to protect it. Suppose that the supply voltage is 5 V.

- Design a resistor that you can safely turn on the LED using the 5 V power source. You may get different amounts for the resistor using different colors for the LED (**15 points**). 

- Implement the circuit on your breadboard, and measure the voltage across the LED. Is it the same as the forward bias voltage of the LED that you measured using the diode test (**15 points**)? 

- Measure the current and verify that it is less than the LED's forward current. **Make sure that you choose the right range for current on your multimeter. You should plug the red probe into the correct port based on the current range or else you will burn the fuse** (**15 points**). 

### Passive vs. Active Components

**Passive components**, like **resistors and capacitors**, have **fixed values**. For example, a 10k resistor will consistently present 10k resistance, irrespective of the applied voltage. 

<figure>
<p align="center">
<img width="436" alt="resistor passive element" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/324cee95-e4d0-442e-a465-97bad51915cb">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

In contrast, a **diode**, an **active component**, exhibits **dynamic behavior**. Its **resistance** **changes** significantly with the **applied voltage**. When **reverse-biased**, its resistance is **effectively infinite**, **blocking current flow**. This is depicted in the following diagram.

<figure>
<p align="center">
<img width="436" alt="diode reverse biased diagram" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/612ea10a-0521-4cf5-96ea-05c0036d5b6b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Upon **forward-biasing**, the diode initially shows **infinite resistance**, which drastically **drops to almost zero** ohms once the forward voltage is **sufficient** to '**turn on**' the diode. These transitions from **high to low resistance** demonstrate the diode's **active** **response** to voltage changes, as shown in the diagram below.

<figure>
<p align="center">
<img width="472" alt="forward-biasing diode diagram" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/5ac94490-2615-4e80-9069-997702f01a84">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Hence, a **diode's resistance varies dynamically** with the **applied voltage**, categorizing it as an **active component** that actively modulates its resistance based on external conditions.

## Guidelines for the labsson 3 report

- The grading criteria are as follows:
  - each activity and question according to respective points mentioned throughout the text (total = 90 points)
  - a conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
  - references (disclose the use of AI) (5 points)

- Some useful notes:
  - Keep it concise and to the point.
  - You can use any text editing software that you are comfortable with, like **Google Docs** or **Latex**. 
  - make sure to provide photos of the activities done when needed
  - include the labsson title and your name in the report 

Good luck