This homework assignment is designed to test your understanding of the exponential coordinates of rotation and Euler angles (note that some of the questions are adapted from our textbook, but some are designed by myself). 

**Question 1:**

In terms of the $`\hat{x}_s`$, $`\hat{y}_s`$, $`\hat{z}_s`$ coordinates of a fixed space frame {s}, the frame {a} has its $`\hat{x}_a`$ axis pointing in the direction (0,0,1) and its $`\hat{y}_a`$ axis pointing in the direction (-1,0,0), and the frame {b} has its $`\hat{x}_b`$ axis pointing in the direction (1,0,0) and its $`\hat{y}_b`$ axis pointing in the direction (0,0,-1).

(a) Calculate the matrix logarithm $`[\hat{\omega}]\theta`$ of $`R_{sa}`$. Do this first by hand (5 points), then write a Python/MATLAB program that gets a rotation matrix and gives its axis-angle representation (5 points). Also, the program should be able to draw the axis in the {s} frame (5 points). Verify that what you have calculated by hand is the same as the axis and angle that your program provided (5 points). Note that your program should be able to handle all cases of rotation matrices.

(b) Calculate the matrix exponential corresponding to the exponential coordinates of rotation $`\hat{\omega}\theta = (1,2,0)`$ (5 points). Draw the corresponding frame relative to {s}, as well as the rotation axis $`\hat{\omega}`$ (5 points). Write a Python/MATLAB program that verifies this for you (5 points). 

**Question 2:**

(a) Given the rotation matrix

$`R = \begin{pmatrix}
0 & 0 & 1\\
0 & -1 & 0\\
1 & 0 & 0
\end{pmatrix}`$

find all possible values for $`\hat{\omega} \in \mathbb{R}^3, ||\hat{\omega}|| = 1`$, and $`\theta \in [0,2\pi)`$ such that $`e^{[\hat{\omega}]\theta} = R`$ (7 points). Verify this using the code that you wrote in the previous question (5 points). 

(b) The two vectors $`v_1,v_2 \in \mathbb{R}^3`$ are related by 

$`v_2 = Rv_1 = e^{[\hat{\omega}]\theta}v_1`$

where $`\hat{\omega} \in \mathbb{R}^3`$ has length 1, and $`\theta \in [-\pi,\pi]`$. Given $`\hat{\omega} = (\frac{2}{3},\frac{2}{3},\frac{1}{3}), v_1 = (1,0,1), v_2 = (0,1,1)`$, find all the angles $`\theta`$ that satisfy the above equation (8 points). 

**Question 3:** XYZ Roll-Pitch-Yaw angles

Another type of Euler Angles that is very common in aerospace applications is called XYZ Roll-Pitch-Yaw angles. Roll-pitch-yaw angles are a sequence of rotations about axes of the **space frame**. Visualize Roll-Pitch-Yaw angles as a sequence of rotations relative to the space frame as the following:

<figure>
<p align="center">
<img width="582" alt="roll-pitch-yaw-angles-visualization-four-frames-2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/2d50710d-c766-4952-b4ee-0f2c74cf2a05">
<figcaption> <p align="center">Visualization of the XYZ roll-pitch-yaw angles. The body frame is initially coincident with the space frame and then goes through a rotation about the space frame’s x-axis followed by a rotation about the space frame’s y-axis and finally a rotation about the space frame’s z-axis.</figcaption> </p>
</p>
</figure>

The terms roll, pitch, and yaw are traditionally used to describe the rotational motion of a ship or an aircraft where the roll is considered to be in the forward motion direction, the pitch is in the direction of the wing, and the yaw is towards the ground (or up):

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/37e44da9-fd7b-4ec1-95bf-d855789315ee
<figcaption> <p align="center">Demonstration for roll-pitch-yaw angles. This simulation is from the website www.wolfram.com.</figcaption> </p>

(a) Find the rotation matrix that represents XYZ roll-pitch-yaw angles depicted above. Note here that all the rotations happen w.r.t the space frame (successive rotations about the fixed frame axes) (6 points). 

(b) The result that you get from (a) should be the same as the ZYX Euler angles that we studied in the lesson (verify this, 5 points). Though the orientations are the same at the end, does this mean that the physical interpretations are the same as well (5 points)? Think about this in terms of the order in which the rotations are written and how they are happening. 

(c) **Inverse problem:** Suppose the representation of an orientation is given by a rotation matrix R as:

$`R = \begin{pmatrix}
0.6 & 0.79 & -0.01\\
0.47 & -0.34 & 0.81\\
0.64 & -0.5 & -0.58\
\end{pmatrix}`$

Find the equivalent sets of roll-pitch-yaw angles that can represent this orientation (7 points). You should find two sets of roll-pitch-yaw angles that give the same orientation. Show this in simulation using RoboDk that these two sets of Euler angles will result in the same orientation (in the drop-down menu use a convention that uses XYZ roll-pitch-yaw angles for the rotation) (7 points). 

**Question 4:** Converting the Orientation given by roll-pitch-yaw angles to the Equivalent Euler Angles

Suppose that we have a remote controller with a known orientation given by the roll-pitch-yaw angles $`(\gamma_{rpy},\beta_{rpy},\alpha_{rpy})`$:

<figure>
<p align="center">
<img width="447" alt="remote-control-roll-pitch-yaw-angles" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/a532b5a8-d42c-41c4-90b2-376695f68851">
<figcaption> <p align="center">The orientation of the remote control is given by the set of roll-pitch-yaw angles. In practical applications, the orientation of an object can be measured using sensors such as a gyroscope.</figcaption> </p>
</p>
</figure>

(a) We want to find a transformation that transforms the orientation of the remote control given by roll-pitch-yaw angles $`(\gamma_{rpy},\beta_{rpy},\alpha_{rpy})`$ to the ZYZ Euler angles (successive rotations in the body frame) $`(\alpha_{zyz},\beta_{zyz},\gamma_{zyz})`$ representation of the orientation of the robotic hand.

<figure>
<p align="center">
<img width="430" alt="ZYZ euler angles" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/2dd8be4d-a411-42bd-8850-1a210f4f8122">
</p>
</figure>

- Note: think about the orientation of the remote control as the desired (known) orientation that the robot hand wants to achieve. What sets of ZYZ Euler angles does it need? You will get two sets of such angles (5 points). 

- You can always use MATLAB/Python's symbolic computing features for ease of calculations. For example, in MATLAB you can define theta as a symbol like: `syms theta` and from that time you can use theta and it will do all calculations symbolically (use `simplify` function to make the answer simpler if possible). For Python, we learned how to do that in this lesson's lab. 

(b) As a numerical example, if the sensor attached to the remote control measures the orientation of the remote control approximately as the Roll-Pitch-Yaw angles of $`(30^{o},60^{o},45^{o})`$, then in order for the robotic hand to follow this orientation, find the ZYZ Euler angles using the equations that you symbolically calculated above (5 points). 

(c) Use RoboDK and show that these sets of Euler angles will give the same orientation as the set of the roll-pitch-yaw angles representing the orientation of the remote. Record a video of your demonstration (5 points). 

**Submission:**

- Submit your answers via Canvas in a PDF and video format. If the video is large, you can include a link to a Drive, and I can download it from there. 
- For the coding section, provide a link to your GitHub page or submit the code directly through Canvas. 

Good luck!