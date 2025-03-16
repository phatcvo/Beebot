## Objectives of lab 7

- Find the Pincherx 100 Robot Arm's Pose in Its Zero Position

- Calculate the Transformation Matrices for Kinova's Gen3 Robot Arm

## Required hardware and software

- PincherX 100 Robot Arm

- Computer running Ubuntu 22.04

- ROS2 Humble

- RViz

## Part 1: Pincherx 100 Robot Arm's Pose in Its Zero Position

One of the first steps to calculate the **forward kinematics** of an open-chain robot arm using **screw theory** is to find the pose in the zero position of the robot. We will calculate the full forward kinematics when studying screw theory but for now, let's complete one of the first strides. 

**Step 1.** Launch the robot simulation in RViz as we learned in the previous labs. 

- Put the robot in its zero position. 

- Draw the schematic of the robot's base frame and end-effector frame or alternatively hide the robot and all other frames but the base and the end-effector frame from the left-hand side in RViz and take a screenshot to start with (10 points).

- Find the **orientation** of the end-effector frame w.r.t. the base frame in the robot's **zero pose**. This will give you the orientation part of the pose transformation matrix (10 points). 

- Now use the robot's technical drawing (make sure you also verify that the measurements are correct on the physical robot) and find the **position** of the end-effector frame w.r.t the base frame. That will give you the position part of the transformation matrix (10 points). 

<figure>
<p align="center">
  <img width="750" height="187.5*2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c5472f00-bb40-4895-8583-7597b68a2acb">
<figcaption> <p align="center">This technical drawing is from Trossen Robotics Website. </figcaption> </p>
</p>
</figure>


- Complete the **transformation matrix** and call this matrix M which will give you the pose of the end-effector frame w.r.t the base frame in the robot's zero position (10 points). 

**Step 2.** Now put the physical robot in the home position and show that the matrix M makes sense. Show the measurements on the board that the robot is attached to (show that the pose is actually what you calculated) (10 points). 

## Part 2: Transformation Matrices for Kinova's Gen3 Robot Arm

[Kinova's Gen3 robot arm](https://www.kinovarobotics.com/product/gen3-robots#ProductGallery) is an ultra-lightweight, modular, and adaptable robotic arm designed for research, education, and industrial applications. This robot arm has 7 degrees of freedom. The standard base and tool frames in this robot are depicted below:

<figure>
<p align="center">
<img width="191" alt="kinova's gen3 robot arm 7 dof" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7710f815-3e72-43d0-85af-09bf696ca4f3">
<figcaption> <p align="center">The standard and tool frames of the Kinova's Gen3 robot arm.</figcaption> </p>
</p>
</figure>

Now for more practice, we want to find transformations between successive reference frames in this robot in the robot's zero pose. The successive frames in the 7-dof  version of this robot with a spherical wrist can be depicted as below:

<figure>
<p align="center">
<img width="375" alt="successive coordinate frames for kinova's gen3 robot arm " src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b9cf0a87-7c35-47d4-a157-5311fa12ebe2">
<figcaption> <p align="center">Successive reference frames in the Kinova's Gen3 robot arm.</figcaption> </p>
</p>
</figure>

Find the successive Transformation matrices ($`T_{s1}, T_{12}, ..., T_{7e}`$), where {s} is the base frame and {e} is the interface module frame (5.5 points for each of the matrices - 8 in total). 

- What is the transformation matrix describing the pose of the end-effector frame in the base frame? What is the easiest way to calculate this? Is this matrix what you expected (6 points)? 

## Guidelines for Lab 7 report

- Submit one report per person (disclose all your collaborators, including the AI ones) through Canvas. **It will be graded based on the above rubric. Please see above for points for each part.** 
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 7! You've done a fantastic job.