## Objectives of lab 6

- This lab will reiterate the concepts learned in Lab 5 regarding the orientation of the robot arm and will show you alternative methods for calculating the orientation of the PincherX100 robot arm's end effector.
- You will also solve a problem to find the Euler angles that represent a robotic wrist's desired orientation

## Required hardware and software

- Computer running Ubuntu 22.04
- PincherX 100 robot arm along with its RViz simulation environment
- Python programming environment (VS code)
- Pen and paper 

## Part 1: Tool Orientation of the Pincherx 100 Robot Arm Using Exponential Coordinates

Similar to what we did in lab 5, we want to find the orientation of the tool frame w.r.t the base frame but this time using exponential coordinates of rotation. 

**Step 1.** Using the diagram that you drew in lab 5, find the orientation of the tool frame w.r.t the base frame for the set of joint angles $`\theta_1 = 90^{0}, \theta_2 = -45^{o}, \theta_3 = 0, \theta_4 = 45^{o}`$ using exponential coordinates. Complete the following equation (10 points):

$`R = Ie^{[\hat{z}]\theta_1}e^{[\hat{y}]\theta_2}--`$.

Note here that since all rotations starting from the first joint happen w.r.t the current (body) frame (you can verify this with your joint control code of the robot), we **post-multiply** all the rotations. 

Now write a Python program that can calculate the rotation matrix R. Follow these steps: 

- Calculate the skew-symmetric matrices $`[\hat{z}]`$, and $`[\hat{y}]`$ (10 points).

- Use Rodrigue's formula to find each exponential function (10 points). 

- Find the final result by multiplying the successive rotations (10 points).

You can start with the following code:

``` Python
import numpy as np

np.set_printoptions(suppress=True)

# Angles in radians
theta_1 = 
theta_2 = 
theta_3 = 
theta_4 = 

# Skew-symmetric matrices
z_hat_bracket = 
y_hat_bracket = 

# Calculate the rotation matrices using Rodrigue's formula
e_z_hat_bracket_theta_1 = 
e_y_hat_bracket_theta_2 = 
e_y_hat_bracket_theta_3 = 
e_y_hat_bracket_theta_4 = 

# Calculate the final rotation matrix product of exponentials 
R = 

# Print the final rotation matrix
print(R)

# Question: Is this familiar to you?
```
**Is this rotation matrix familiar to you?** (10 points) 

**Step 2.** Can the tool of the Pincherx 100 robot arm reach the following orientation? In other words, what is the set of joint angles that can give the following orientation of the tool frame? 

$`R_{desired} = \begin{pmatrix}
-1 & 0 & 0\\
0 & 0 & -1\\
0 & -1 & 0
\end{pmatrix}`$.

- For this, solve the final tool orientation symbolically from the multiplication of the exponential functions. You can complete the code below (10 points):

``` Python
import sympy as sp

# Define symbolic variables
t1, t2, t3, t4 = sp.symbols('t1 t2 t3 t4')

# Define the skew-symmetric matrices
z_hat_bracket = sp.Matrix()
y_hat_bracket = 

# Calculate e_bracket_z_t1
e_bracket_z_t1 = sp.eye(3) + ...

# Calculate e_bracket_y_t2
e_bracket_y_t2 = 

# Calculate e_bracket_y_t3
e_bracket_y_t3 = 

# Calculate e_bracket_y_t4
e_bracket_y_t4 = 

# Calculate the final result
result = e_bracket_z_t1 @ e_bracket_y_t2 @ e_bracket_y_t3 @ e_bracket_y_t4

# Simplify the result
simplified_result = sp.simplify(result)

# Display the simplified result
print(simplified_result)
```
- Now equate this orientation to the desired orientation. What do you see? Can the tool frame of the Pincherx 100 reach the given orientation? Can you find $`\theta_1`$, $`\theta_2`$, $`\theta_3`$, and $`\theta_4`$ such that the desired orientation can be achieved (10 points)? 

## Part 2: Determining the ZYX Euler Angles for the Robotic Wrist's Desired Orientation

Suppose we have a robotic wrist mechanism that has 3 DOF, and the axes of three joints intersect at a point. Initially, the wrist mechanism is in its zero position, meaning that all three joint angles are set to zero. 

<img width="441" alt="three-dof-orienting-mechnism-robotic-wrist-3" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d741952e-1b8d-465f-963f-2788c15a85cd">

Given the rotation matrix representing the wrist's orientation (w.r.t the world frame) as:

$`\begin{pmatrix}
0.866 & -0.5 & 0 \\
0.5 & 0.866 & 0 \\
0 & 0 & 1 \\
\end{pmatrix}`$

- Draw the coordinate frame representing this orientation using the code that you wrote in the previous lab (10 points). 
- Determine the ZYX Euler angles $`(\alpha,\beta,\gamma)`$ that represent this orientation of the robotic wrist (10 points). 
- Download and install [RoboDK](https://robodk.com/), add a coordinate frame, then choose ABB/KUKA/Nachi from the drop-down menu and apply these sets of angles. Do you get the same orientation as your drawn coordinate frame? Do the two sets of Euler angles that you computed give the same orientation (10 points)?

## Guidelines for Lab 6 report

- Submit one report per person (disclose all your collaborators, including the AI ones) through Canvas. **It will be graded based on the above rubric. Please see above for points for each part.** 
- The codes can be submitted through a GitHub repo (with a link provided) or alternatively be uploaded to the submission.  
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 6! You've done a fantastic job.