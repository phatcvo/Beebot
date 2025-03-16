## Objectives of lab 4

- Understanding the concept of DOFs and different joint types using the PincherX 100 Robot Arm
- Being able to calculate the DOFs for different types of robots and mechanisms

Here are some implementations by students in the previous semesters in case you are interested:

https://youtu.be/RYROSBkbkp8

## PincherX 100 Robot Arm DOFs and Joint Types

**Equipment:**

- PincherX 100 Robot Arm
- Computer with PincherX control software installed

**Step 1.** Identify the joint types in the PincherX 100 robot arm and discuss how they constrain the movement of the links (20 points).

**Step 2.** Using Grübler’s formula that we learned in lesson 2, calculate the total DOFs of the PincherX 100 Robot Arm (do not take into account the opening and closing of the end-effector). Note that a joint connects two links and traditionally ground is considered a link (because the first joint connects the ground to the first link of the robot) (20 points). 

**Step 3.** Using the joint control code in Lab 3, experimentally show that the robot actually has that number of DOFs. Write the code and first show it in simulation and then try it on the robot arm. Submit a video for this part (20 points).  

## Practice Exercises

1. Three identical SRS open-chain arms are grasping a common object, as shown in the figure below:

<figure>
<p align="center">
<img width="465" alt="SRS arms grasping a common object" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7462e269-1945-4462-bf23-85136c41bb0c">
<figcaption> <p align="center">Three cooperating SRS arms grasping a common object. Exercise is from the Modern Robotics book by Frank Park and Kevin Lynch. </figcaption></p>
</p>
</figure>


Find the number of degrees of freedom of this system using Grubler's formula. Verify that (just for yourself) it actually has that many DOFs (20 points). 

2. Write a Python program that gets the number of degrees of freedom of a single body (m), number of bodies (N), number of joints (J), and number of joints with 1, 2, and 3 DOFs as inputs from the user and prints out the number of degrees of freedom of that mechanism. Numerically, verify that your program works using the above examples. **Useful hint:** If you needed to change a string to an integer, do so using the int() function (20 points). 

## Guidelines for Lab 4 report

- Submit one report per person (disclose all your collaborators including the AI ones) through Canvas. The report will be graded based on clarity, correctness of calculations, completeness of the experimental results (including the code, simulation, and video demonstration), and the overall presentation of the report. Please see above for points for each part. 
- The codes can be submitted through a GitHub repo (with link provided) or alternatively be uploaded to the submission.  
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 4! You've done a fantastic job.