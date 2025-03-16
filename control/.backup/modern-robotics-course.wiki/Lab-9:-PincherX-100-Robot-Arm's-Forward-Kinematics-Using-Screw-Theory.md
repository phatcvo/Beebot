## Objective of lab 9

- To understand and calculate the forward kinematics of the PincherX 100 robot arm using the Product of Exponentials (PoE) formula based on screw theory and validating it using the physical robot arm. 

## Required hardware and software

- PincherX 100 robot arm
- Computer running Ubuntu 22.04 and RViz where control software for the robot are installed
- A ruler or any measurement tool
- Pen and paper for calculations and notes

## Part 1: Python Version of the MATLAB Code for POE Formula for Forward Kinematics

Study the MATLAB code that is given to you in Lesson 7: Forward Kinematics of Robot Arms Using Screw Theory, and develop the Python version of this code that calculates the position and orientation of the end-effector frame in the base frame using the PoE formula (12.5 points).

## Part 2: Pincherx 100 Robot Arm's Forward Kinematics Using Screw Theory 

- Run the robot in simulation, put the robot in zero configuration, and hide the robot arm and all other frames but the base frame and the end-effector frame (you may also leave other links in case you need to use one axis later for the rotation axis. This is not required but it is nice if everyone gets the same transformation at the end) from the left-hand side of the RViz (12.5 points).

- Find M which is the pose of the end-effector of the robot w.r.t the base frame in the robot's zero configuration. Use the robot's technical drawing for the measurements or simply measure the link lengths etc. using a ruler/measurement tape (12.5 points).

- Determine, draw, and calculate all the screw axes (the direction of the screw axis should be in the positive movement of the joint based on the RHR) (12.5 points).

- Now use the Product of Exponentials (PoE) formula to find the pose of the end-effector with respect to the base frame. You do not need to do the matrix multiplication (12.5 points). 

-  Use the code that you wrote in part 1 and find the transformation matrix representing the pose of the end-effector in the base frame for $`\theta_1 = 0^{0}, \theta_2 = 0^{0}, \theta_3 = -90^{0}, \theta_4 = 90^{0}`$ (12.5 points). 

- Now, attach the physical robot, first go to the home pose, and from there put the robot in the above configuration (set the joint values to the above given values). Physically show that the transformation matrix that you calculated is the same as the physical robot's end effector's position and orientation (12.5 points).   

- Choose another set of angles (pay attention to the joint limits of the robot) and verify that your calculation is the same as the physical position and orientation of the robot (12.5 points). 

## Guidelines for Lab 9 report

- Submit one report per person (disclose all your collaborators, including the AI ones) through Canvas. **It will be graded based on the above rubric. Please see above for points for each part.** For every part, photo and/or video proofs are required. 
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 9! You've done a fantastic job. 