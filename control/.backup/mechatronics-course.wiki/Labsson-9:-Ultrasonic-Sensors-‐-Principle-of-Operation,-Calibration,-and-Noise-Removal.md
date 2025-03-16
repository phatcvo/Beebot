## Introduction 

In this labsson, we will become familiar with a very common type of sensors named **ultrasonic sensors**, their **principle of operation**, a general **calibration** method and **a general noise removal method** that are applicable to many sensors. A separate labsson is dedicated to these sensors because of their importance but we will become familiar with more sensors in the **mobile robots module** and we can incorporate different sensors in our projects based on the **requirements and needs** of the project. 

Note: In nature, bats and whales understand distance with similar procedure and there are some man-made submarines that also have this feature. 

## Ultrasonic Sensors - Principle of Operation

The **HC-SR04 ultrasonic sensor** is a widely used sensor for **distance measurement**, and obstacle detection in various electronic projects. It works based on the principle of **ultrasonic sound waves** and their **reflection**. The components of an HC-SR04 Ultrasonic Sensor are: 

<figure>
<p align="center">
<img width="261" alt="HC_SR04_ultrasonic_sensor" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e65c51b6-f5ed-45b4-845e-6237b18da9b1">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

- **Transmitter/Receiver (T/R) Pair.** The sensor has a pair of transducers, one for **transmitting ultrasonic pulses (transmitter)** and the other for **receiving the reflected waves (receiver)**. If you look at your sensor, they are identified with the letters T and R. 

- **Control Circuit.** The sensor includes a control circuit responsible for **triggering the transmitter** and **measuring the time delay for the received echoes**.
- **Trigger (Trig) Pin.** The Trigger pin (Trig) is used to **initiate the ultrasonic pulse**. When a **pulse** is sent to this pin, the sensor **transmits an ultrasonic wave**.
- **Echo Pin.** The Echo pin (Echo) is used to **measure the time** it takes for the **ultrasonic pulse to travel to the target and back**. It **receives the echo signal**.
- **VCC and GND Pins.**  Standard power pins for connecting the sensor to a power source.

To **measure distance**, the HC-SR04 sends a short ultrasonic pulse (see video above). **The pulse is generated when a `HIGH` level is applied to the Trig pin for at least 10 microseconds**. When the **trigger pin** is set `HIGH` for at least **10 microseconds**, the sensor emits an 8 cycle sonic burst (8 cycle burst of ultrasound) at 40 kHz:

<figure>
<p align="center">
<img width="738" alt="how_an_ultrasonic_sensor_works" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/0e48fb8a-92bd-4a82-ba62-f4a45b4370f2">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The ultrasonic pulse travel through the air at the speed of sound until it encounters an object in its path. When the **pulse hits an object**, **it reflects back to the sensor**. The **Echo pin then receives the reflected wave**. The sensor measures the time it takes for the ultrasonic pulse to travel to the target and back. This time is known as the "**time of flight**." The **output** of the HC-SR04 ultrasonic sensor is **the duration of time that the Echo pin remains high after a trigger pulse is sent**. This duration as mentioned is referred to as the "time of flight."

<figure>
<p align="center">
<img width="738" alt="ultrasonic_sensor_operation_diagram" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/7f4c3c67-16d7-4fa4-878a-6bc349cfadb6">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The sensor calculates the distance using the following equation (we know from physics that the distance is equal to speed multiplied by the time):

$`\text{Distance} = \text{Speed of Sound} \times \frac{\text{Time of Flight}}{2}`$

Note that the division by 2 accounts for the **round trip of the ultrasonic pulse**.

## (14 points) Class Activity: Wiring Instructions and Arduino Code Instructions for Running the Ultrasonic Sensor

In this class activity, you will be guided through the process of wiring your ultrasonic sensor to an Arduino and are given instructions for creating the Arduino code necessary to operate the sensor.

***

### Wiring Instructions

**Connecting the HC-SR04** ultrasonic sensor to Arduino is pretty straightforward:
- Connect the VCC and GND pins to the 5V power source, and the ground (GND) of Arduino, respectively.
- Trig and Echo pins can be connected to any **digital pins** on Arduino. Just pay attention to which pin should be an input and which should be an output (if you are writing to a pin, it is an output and if you are reading from it, it is an input). 

***

### Arduino Code Instructions

- Open the Arduino IDE on your computer and create a new sketch.
- As always, at the beginning of the code define digital pins for Trig and Echo pins of the sensor. 
- Inside the `setup()` function, **begin serial communication** and **set the pin modes** for the trigger and echo pins (pay attention to which pin should be input and which one should be the output). 
- In the `loop()` function:
  - start by **triggering the ultrasonic pulse** (as we said before, the Trig pin should be high at least for 10 microseconds to trigger the ultrasonic pulse). You can do this by setting the Trig pin to `LOW` briefly (use `delayMicroseconds()` function for delays in the order of microseconds), then `HIGH` for 10 microseconds, and back to `LOW`. What you did here, you exactly created the pulse depicted above and you wrote that pulse to the Trig pin. 
  - Now that you created the ultrasonic pulse, you should read from the Echo pin (note that the Echo pin will stay `HIGH` as long as the Trig pin is high and goes back to `LOW` when the Trig pin goes to `LOW`). What you need to do is, you should **measure the the duration of the incoming echo signal**. For this, you can use `pulseIn(pin, state)` function. This function is used to measure the duration of a pulse on a pin. For example, in this case, you should measure how long the Echo pin stays in the `HIGH` state. Store this in a variable for duration (do not forget to declare and note that the measured time is in **microseconds**). Print out this number on Serial Monitor, place an object in front of the sensor and try to move it closer and farther and **explain** how this number changes with respect to the object's distance to the sensor. 
  - (**14 points**) Now, **create a table** in Excel to record the time (as measured in microseconds by the `pulseIn` function) and the corresponding distance (better to use the **metric system** so that we all get the same answer but not mandatory) to an object (measured **manually** with a **ruler**). Move the object farther from the sensor and repeat the process until you have some data points (make sure to divide the time by 2 as it is the time that the ultrasonic pulse travels to the object and then goes back). Use Excel or any similar software to **visualize data** (distance-time) and then fit a line to the data points. Calculate the **slope** of the line (pay attention to the unit). What does it represent? Compare it to a known value in the literature, are they the same?     
  - Next, calculate the distance and put the equation in your code. 
  - Finally, print the distance to the Serial Monitor, and do not forget to add a brief delay before the next measurement.
- Verify and upload your sketch to Arduino. 
- (**14 points**) Open the Serial Monitor, place objects at different distances in front of the ultrasonic sensor to see the distance measurements in the Serial Monitor. Get a ruler and measure to see if the real measurement matches the one that you see in Serial Monitor. In case they don't explain why? Do you see a lot of fluctuations in data? Explain what this fluctuation is. 

***

### (14 points) Visualizing Sensor Data

Let's now **visualize data** (measured **distance** by the sensor **vs. time**) and figure out the **nature of the noise**. Depending on the nature of the noise, you should decide whether you should do the noise removal first or calibration first. If the noise is a lot such that it is obscuring the actual data, then you need to remove noise first but if it is just some random noise, then do the calibration first. 

To visualize the data that you see on Serial monitor, you will need to save the data to a .csv file, but to do this we need some additional steps as the plotter in Arduino is not user-friendly and you have no control over it. There are different methods to save the data that you see in Arduino's Serial monitor (like downloading different software), but I decided to write a **Python code** that **reads the Serial port** of your computer and saves the data as a .csv file. Here are the steps:

**Step 1.** First you need to **print the measured distance by the sensor** and you will also need **time stamps**. In order to add time stamps to data, add the following code to the `loop()` function to capture the current time:

```C++
unsigned long currentTime = micros();
```

And make sure to print it along with the distance (add comma between the time and the measured distance and add a small delay to capture as many data as possible):

```C++
Serial.print(currentTime);
Serial.print(","); 
Serial.println(measured_distance);

// Add a delay before the next measurement
delay(100); 
```
**Step 2. Install Visual Studio Code (VS Code)**:
- Go to the [Visual Studio Code website](https://code.visualstudio.com/) and download the installer for your operating system (Windows, macOS, Linux).
- Follow the installation instructions to install VS Code on your computer.

**Step 3. Install Python:** Python does not come pre-installed on most operating systems, with the exception of some Linux distributions and macOS. However, the pre-installed version on macOS might not be the latest, and you might still want to install a newer version or manage multiple versions of Python. To check if you have Python installed, and to see which version it is, open a terminal (on Linux or macOS) or Command Prompt (on Windows) and type: `python --version` or `python3 --version`. If Python is installed, this command will display the version number. If not, or if you need to install a different version, download Python from the [official Python website](https://www.python.org/downloads/). During installation on Windows, make sure to tick the box labeled "Add Python to PATH" to ensure that the Python interpreter can be easily run from any command prompt or terminal window.

Also in order to **reading data from a serial port** and writing it to a file, you'll need the `pyserial` library installed on your computer. This library provides the **necessary functionalities to interact with the serial port**. To install `pyserial`, you can use `pip3`, the Python package installer. Follow these steps:
- Open your terminal (or Command Prompt).
- Type the following command and press Enter: `pip3 install pyserial`

This command downloads and installs the `pyserial` package and makes the `serial` module available for your script to import and use.

**Step 4. Install the Python extension for VS Code**:
- Open VS Code, go to the **Extensions** view by clicking on the square icon on the sidebar.
- Search for "Python" and install the extension provided by Microsoft.

**Step 5. Create a Python Script**:
- Create a new file in VS Code with the extension .py, for example, read_sensor_data.py.
- Copy the Python code below into this file:

```Python
import serial
import time

# Configuration
serial_port = '/dev/cu.usbmodem11101'  # Change this to your Arduino's serial port
baud_rate = 9600  # Change this to the baud rate of your Arduino
output_file_path = 'sensor_data.csv'

# Opening the serial port and the output file
with serial.Serial(serial_port, baud_rate) as ser, open(output_file_path, 'w') as outfile:
    print(f"Reading from serial port {serial_port} and writing to {output_file_path}...")

    try:

        while True:
            # Read one line from the serial port
            line = ser.readline().decode('utf-8').strip()

            # Combine the timestamp and data
            csv_line = f"{line}\n"
            
            # Write the line to the output file
            outfile.write(csv_line)

            # Optionally, print to console for monitoring
            print(csv_line, end='')

    except KeyboardInterrupt:
        # User interrupted the process, exit gracefully
        print("Data collection stopped by user.")
```

Don't forget to replace the `serial_port` variable's value in the Python script with the correct port for your Arduino (you know the drill on how to find it). 

**Step 6: Collect and Visualize Sensor Data**:
- With your Arduino connected to your computer, upload the code you've prepared for capturing sensor readings. Make sure this code prints the raw sensor data and time to the Serial Monitor in a format compatible with your Python script (usually **comma-separated** as I gave you above).
- Before running your Python script, **ensure the Arduino IDE's Serial Monitor is closed** to avoid conflicts over serial port access.
- You should see the timestamp and unfiltered data being printed to the terminal, indicating that data collection has begun.
- Place an object in front of your sensor. A thin, rectangular object often works well for consistent readings. Maintain each position briefly to collect a range of data points. Gradually move the object away from the sensor to gather more varied data. This process helps in capturing a range of distance measurements for analysis.
- When you have collected enough data, **interrupt** the data collection by pressing `Ctrl+C` in the terminal. This **stops** the Python script and completes the data logging process.
- Locate the `.csv` file generated by your Python script. You have several options to visualize this data, and feel free to do it your own way. What I did is that I used Excel, and here is how to do it with Excel: Open the CSV file in Excel, add titles to your columns, and use Excel's chart and graph features to visualize the data.
- Look at your graph to **see the nature of noise** on your data and **decide if you want to do calibration first or noise removal first**:
  - **Random fluctuations** that appear as sharp, erratic deviations from the expected measurement. This is typically indicative of **random noise**, which can be caused by electronic or environmental interference.
  - **Systematic deviations** that show a **consistent bias** or **trend** away from the expected measurement. This could suggest systematic noise, possibly from calibration errors, sensor drift, or consistent environmental effects.
  - **Periodic variations** that repeat at regular intervals. This could indicate noise from cyclic sources, such as mechanical vibrations or electromagnetic interference from nearby equipment.

**Note:** The above analysis of the noise is the **visual inspection** but for comprehensive analysis you can perform **statistical analysis**, and **frequency analysis** (performing Fourier Transform (FFT) on your data to convert it from the time domain to the frequency domain to see specific frequencies that dominate the noise) to further identify the noise that is out of the scope of this labsson but feel free to explore them as I know that you have encountered the theory in **Engineering Experimentation**. 

## (16 points) Sensor Data Calibration and Applying it to the Ultrasonic Sensor Data

**Device calibration** is the process of adjusting and fine-tuning the performance of a measuring or sensing device to ensure its **accuracy** and **reliability**. The primary **goal of calibration** is to **bring the measurements made by a device in line with known standards or reference values**. This ensures that the device provides **accurate and consistent results**, making it trustworthy for various applications.

Key aspects of device calibration include:
- Calibration helps **correct any systematic errors** or inaccuracies in the measurements made by a device. By **comparing the device's readings** with **a standard or reference**, adjustments can be made to improve its **accuracy**.
- Calibration ensures that a device **consistently** produces reliable results over time. It helps maintain the reliability of measurements and prevents variations that may occur due to wear and tear, environmental changes, or other factors.
- Calibrations are often **traceable** to national or international standards that provide a clear and documented chain of measurement accuracy. This traceability is crucial for quality assurance and compliance with standards in various industries.
- In regulated industries, certain devices must meet specific standards and guidelines. Calibration helps ensure that devices **comply** with regulatory requirements and quality standards.
- During the calibration process, adjustments are made to the device to align its measurements with the reference values. These adjustments may involve tweaking settings, applying correction factors, or other means.

***

We will **calibrate our ultrasonic sensor** using a **best linear fit method**. Now in order to follow this, first use the code that you have written above and record the sensor readings for the distance at 10 different distances accompanying with the exact ground truth distance using ruler measurement. By the end of this step, you should have **10 sensor readings** ($`x_i,`$$` i=1,...,10`$) and **10 ground truth readings** ($`y_i,`$$` i=1,...,10`$).

<figure>
<p align="center">
<img width="369" alt="faulty_meaurements_actual_measurements" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/7b16ee1c-7c02-4b21-b5e6-abd3c39b165b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

***

### Optimizing Sensor Accuracy: Linear Fitting

Linear fitting, also known as **linear regression**, is a statistical method that its goal is to find the best-fitting straight line through the data points that minimizes the sum of the **squared differences** between the ground truth values ($`y_i`$) and the values predicted by the line ($`\hat{y}_i`$). The sum of squared differences is also called Sum of Squared **Errors** (SSE):

$`SSE = \sum\limits_{i=1}^{n} (y_i - \hat{y}_i)^2`$  

<figure>
<p align="center">
<img width="369" alt="linear regression" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/0615e6d4-e6c7-4a44-8273-af0b7de76de4">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

By inserting the predicted value $`\hat{y}_i = mx_i + b`$ in the above equation, we can write the Sum of Squared Errors (SSE) as:

$`SSE = \sum\limits_{i=1}^{n} (y_i - (mx_i + b))^2`$

So, the **goal** is to find the values of m and b such that the SSE is minimized (and thus the error is minimized). This means we want to adjust the slope m and the intercept b of our **linear model** so that the sum of the squared differences between the actual observed values ($`y_i`$) and the predicted values ($`\hat{y}_i = mx_i + b`$) is as small as possible. Note that $`x_i, i=1,...,n`$ are the **faulty measured values by the sensor**. This **optimization problem** leads us to the best-fitting line through our data points, according to the least squares criterion. 

To achieve this goal, we solve for m and b that make the partial derivatives of SSE with respect to m and b equal to zero. These conditions are necessary for a minimum in the SSE surface. 

**Partial derivative of SSE with respect to m:**

$`\frac{\partial SSE}{\partial m} = \frac{\partial}{\partial m} \sum\limits_{i=1}^{n} (y_i - (mx_i + b))^2 = -2 \sum\limits_{i=1}^{n} x_i(y_i - mx_i - b)`$

Setting the above equation to zero, we can solve for m:

$`m = \frac{n\sum{x_iy_i} - \sum{x_i}\sum{y_i}}{n\sum{x_i^2} - (\sum{x_i})^2}`$

**Partial derivative of SSE with respect to b:**

$`\frac{\partial SSE}{\partial b} = \frac{\partial}{\partial b} \sum\limits_{i=1}^{n} (y_i - (mx_i + b))^2 = -2 \sum\limits_{i=1}^{n} (y_i - mx_i - b)`$

Again, setting this equation to zero, we can solve for b:

$`b = \frac{\sum{y_i} - m\sum{x_i}}{n}`$

Therefore the equation of the line that can predict the data can be written as: $`\hat{y} = mx+b`$. Note that $`n`$ is the number of data points, $`\Sigma`$ denotes summation, $`x`$ and $`y`$ are values measured by sensor, and the ground truth values, respectively, and $`xy`$ is their product.
 
So the procedure to find a line that can predict the data points can be summarized as follows:
- Step 1: Gather Data. By now you have gathered your data in the previous part.
- Step 2: Calculate Linear Regression Parameters. Use a **linear regression algorithm** or method to calculate the slope $`m`$ and intercept $`b`$. Common methods include the **least squares method** that we talked above. 
- Step 3: Use the calculated slope and intercept to find the equation to predict values: $`\hat{y} = mx+b`$. 
- Step 4: Evaluate the Fit. Compare the **predicted values** with the **actual values** to assess how well the linear model fits the data. Common evaluation metrics include the **coefficient of determination** ($`R^2`$): 

$`R^2 = 1 - \frac{\sum\limits_{i=1}^{n} (y_i - \hat{y}_i)^2}{\sum\limits_{i=1}^{n} (y_i - \bar{y})^2}`$

Where, $`y_i`$ represents the **actual values**, $`\hat{y}_i`$ denotes the **predicted values** based on the regression line, $`\bar{y}`$ is the mean of the actual values, and $`n`$ is the number of observations. $`R^2`$ ranges from 0 to 1, where **1 indicates a perfect fit**.

- Step 5: **Update your code**. Apply the linear fit equation to the code to **change the uncalibrated reading to a calibrated reading**. Print and check to make sure that your sensor is acceptably calibrated. Show a screenshot as proof. 

## (13 points) Noise Removal for Sensors and Applying it to the Ultrasonic Sensor Data

When working with ultrasonic sensor, for distance measurement, you may notice that the readings **fluctuate** unexpectedly, even if the object's distance from the sensor hasn't changed. This **fluctuation** is what we refer to as **noise**. Noise can come from various sources, such as **electrical interference**, **air currents**, or even **slight changes in the angle of the reflected ultrasonic wave**.

These **variations**, or "**noise**," can make your **sensor's data** seem **erratic**, making it challenging to make accurate measurements or decisions based on this data. To get more reliable readings from the sensor, we use **noise filtering techniques**.

### Moving Average Filter

One effective way to reduce noise in your sensor readings is to use a **Moving Average Filter**. This technique smoothens the data by averaging a set number of readings. The **Simple Moving Average (SMA)** is a method used to smooth out data points to help reveal the underlying trend or pattern. By **averaging a series of data points over a specific window**, the SMA reduces the impact of **random fluctuations** or **noise**, making it easier to analyze trends. Here's how it works:

- You select a number of consecutive data points, known as the **window size**. This determines how many of the most recent readings will be averaged to calculate the SMA.
- For each new data point, you calculate the average of the values within the current window. This involves adding up all the data points in the window and dividing by the window size.
- As new data points are collected, the window moves forward, dropping the oldest data point and including the newest one in the calculation.

The result is a series of averages that smooths out short-term fluctuations and highlights longer-term trends or cycles in the data. This technique is particularly useful in situations where data is noisy or prone to sudden spikes, as it helps to provide a clearer view of the data's movement over time.

**Mathematically** SMA can be expressed as:

$`{SMA}_i = \frac{1}{M} \sum\limits_{j=0}^{M-1}d_{i+j}`$

Where, $`{SMA}_i`$ is the **simple moving average** at the $`i^{th}`$ position in the smoothed series, M is the total number of data points in the window used to calculate the average, including the current point $`d_i`$ and the next M-1 points, and $`d_{i+j}`$ represents each data point in the window, starting from the current data point $`d_i`$ and including the next M-1 points in the series. 

***

Example: Suppose that we are measuring the distance to a stationary object using our ultrasonic sensor, but due to factors like sensor noise and environmental conditions, the readings vary around the true distance. The true distance is 100 cm but the ten successive measurements that sensor shows is as follows that show random fluctuations to the true distance:

$`[100,105,101,99,98,106,110,99,102,100]`$

Let's now demonstrate how a moving average filter can smooth these measurements to provide a more consistent estimate of the true distance, reducing the impact of noise. Consider that the window size is 3, so applying the SMA will give us the following values:

$`[102.0,101.67,99.33,101.0,104.67,105.0,103.67,100.33]`$

Here is how we can calculate these values (here the window size is 3):

<figure>
<p align="center">
<img width="443" alt="SMA_example" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/42ec87ec-ac12-4a1a-8441-0a97b1508d31">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

Now let's plot the data (both the original measurements and the smoothed measurements) to see the smoothing effect of the Simple Moving Filter:

<figure>
<p align="center">
<img width="473" alt="SMA_smoothing_effect" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/1f8e1948-a041-46fe-b884-e95663282d1a">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

As you see from the figure above, random fluctuation are smoothed out. Note that **applying a larger window size will further smooth out the data**. 

Figure below shows applying this filter to real sensor data and the smoothing effect it has to mitigate the effect of the noise:

<figure>
<p align="center">
<img width="749" alt="SMA_sensor_noisy_smoothed_data" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/e0b923cf-3b90-46ee-a3a6-a6457bc2c7b8">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

To learn more about Moving Average Filters, you can refer to the following link:

https://www.analog.com/media/en/technical-documentation/dsp-book/dsp_book_ch15.pdf

***

Now, apply this filter to the ultrasonic sensor's calibrated data (that you have done in the previous step) to smooth out the readings. Here’s a simplified algorithm to guide you through the process:

- **Initialize variables.** Declare an **array to store recent distance readings** and **variables for the array index**, **the number of readings (Window Size)**, and for **calculating the sum and average of these readings**. Here is a guide on how to do this:

```C++
//Note that const is needed because in C and C++, the size of an array must be constant and not variable
const int windowSize = 5; // The number of readings to smooth. This is your "window size."
int distanceReadings[windowSize]; // Array to store recent distance readings.
int readIndex = 0; // Index of the current reading in the array.
long total = 0; // For calculating the sum of the window's readings.
float averageDist = 0; // For storing the calculated average distance.
```

Note that `distanceReadings[]` is an array that holds the last windowSize distance readings. The idea behind this is to keep track of a fixed number of recent readings (determined by windowSize) to calculate a moving average. Let's say you've chosen a windowSize of 5. This means your distanceReadings array will hold the last 5 distance measurements. In C++, when you see a declaration like `int distanceReadings[windowSize];`, it's declaring the entire array itself, with a size specified by windowSize. Therefore, `int distanceReadings[windowSize];` declares an array named `distanceReadings` that can hold windowSize number of int (integer) values.

- **Initialize the array** with zeros to prepare it for storing distance readings. The code below helps you do this (put this in the `setup()` function as you want it to be executed only once):

```C++
// Initialize all elements of the distanceReadings array to 0
for (int i = 0; i < windowSize; i++) {
    distanceReadings[i] = 0;
}
```
- In the Main Loop, and after obtaining a new calibrated distance reading, apply the moving average filter as follows:
  - **Store new reading.** Insert the calibrated distance into the array at the current index.
  - **Calculate average.** Sum all readings in the array and divide by window size to get the average distance.
  - **Update index.** Increment the array index for the next reading. Ensure it wraps around when it reaches the end of the array (use modulus operation).
  - Print the moving average distance to the Serial Monitor for observation.

Code below can help you start with this:

```C++
void loop() {
    float calibDistance = /* The calibrated distance from the sensor */;

    // Subtract the oldest reading from total before adding the new one
    total = total - distanceReadings[readIndex];
    // Store the new reading
    distanceReadings[readIndex] = calibDistance;
    // Add the new reading to the total
    total = total + distanceReadings[readIndex];

    // Move to the next position in the array, wrapping around if necessary. 
    // Explaining this line of code: 
    // Assume windowSize is 5, meaning the array can hold 5 measurements. Here’s what happens as readIndex increments:
    // Initially, readIndex = 0.
    // After one loop, readIndex = (0 + 1) % 5 = 1.
    // This continues until readIndex = 4 (the last valid index for the array).
    // On the next increment, readIndex = (4 + 1) % 5 = 5 % 5 = 0. Thus, readIndex loops back to the start of the array.
    readIndex = (readIndex + 1) % windowSize;

    // Calculate the average
    averageDist = total / windowSize;
    
    // Print the moving average distance
    Serial.println(averageDist);

    // The rest of your loop code...
}
```

- **Collect and visualize your data using the method explained above** (**averaged** and **calibrated data** vs time) and see if the filter is actually working.  

## Guidelines for the labsson 9 report

**The grading criteria are as follows:**
- Abstract (5 points)
- Each activity and question according to respective points mentioned throughout the text (total = 85 points)
- A conclusion paragraph that talks about what challenges you had and how you solved those (5 points) 
- References (disclose the use of AI) (5 points)

**What to submit:**

Your report should include these besides the written text:

- a photo of your completed circuit. 
- Answers to questions with screenshots and codes if applicable. 
- All calculations, data tables, and calibration procedures for each part.
- Data visualizations

Good luck!