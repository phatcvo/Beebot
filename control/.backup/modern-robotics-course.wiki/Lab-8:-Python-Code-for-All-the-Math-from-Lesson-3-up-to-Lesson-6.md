## Objective of lab 8

In this lab, we will develop functions in Python that can compute all the math that we learned from lesson 3 up to lesson 6. We will use these functions when studying the next parts of the course. 

## Required Software

- Visual Studio Code or any IDE for Python

## Instructions

Based on the lessons that we have learned starting from Lesson 3: Orientation in Robotics (Rotation Matrices) to Lesson 6: Pose in Robotics (Exponential Coordinates of Robot Motions), develop functions in Python that can calculate the following. One function

- gets the rotation matrix R and returns its inverse (6 points). 

- gets a 3-vector $`\hat{\omega}`$ and gives back the $`3 \times 3`$ skew-symmetric matrix representation of it (7 points). 

- gets the angle $`\theta`$ and the unit axis of rotation $`\hat{\omega}`$ and returns the rotation matrix representing the rotation about $`\hat{\omega}`$ by $`\theta`$. This function should be able to handle the special cases of rotation operators about $`\hat{x}`$, $`\hat{y}`$, and $`\hat{z}`$. Use Rodrigues' formula to write this function (7 points). 

- gets the $`3 \times 3`$ skew-symmetric matrix representation of $`\hat{\omega}`$ and returns the 3-verctor $`\hat{\omega}`$ (7 points). 

- gest the 3-vector exponential coordinates for rotation $`\hat{\omega}\theta`$ and extracts the rotation axis $`\hat{\omega}`$ and the rotation amount $`\theta`$ (7 points).

- gets a $`3 \times 3`$ rotation matrix R and finds the axis $`\hat{\omega}`$, and angle $`\theta`$ representation of it (7 points). 

- gets the rotation matrix $`R \in SO(3)`$ and a position vector $`p \in \mathbb{R}^3`$ and returns the $`4 \times 4`$ homogenous matrix T coressponding to them (7 points). 

- gets the $`4 \times 4`$ homogenous transformation matrix T and extract the $`3 \times 3`$ rotation matrix and $`3 \times 1`$  position vector from it (7 points).

- gets the $`4 \times 4`$ homogenous transformation matrix T and computes the inverse of it (7 points). 

- gets a 3-vector and returns its homogenous coordinates (6 points).

- gets the $`4 \times 4`$ homogenous transformation matrix T and computes its $`6 \times 6`$ adjoint representation $`[Ad_{T}]`$ (7 points). 

- gets the 6-vector exponential coordinates of motion $`\mathcal{S}q`$ and extracts the normalized screw axis $`\mathcal{S}`$ and the distance traveled along the screw q (7 points). 

- gets the 6-vector screw axis and computes the matrix representation of it (6 points). 

- gets the screw axis $`\mathcal{S}`$ and q and calculates the corresponding homogenous transformation matrix $`T \in SE(3)`$ (6 points). 

- gets the $`4 \times 4`$ homogenous transformation matrix and computes the screw axis and q (6 points). 

## Guidelines for Lab 8 report

- Submit one report per person (disclose all your collaborators, including the AI ones) through Canvas. **It will be graded based on the above rubric. Please see above for points for each part.** 
- **The report should include an example for each function that receives pertinent input and produces accurate output. Provide this in written format**. 
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 8! You've done a fantastic job.