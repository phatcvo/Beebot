## Objectives of lab 5

- Calculate and visualize the orientation of the end effector of the PincherX100 robot arm with respect to its base frame using rotation matrices

## Required hardware and software
- Computer running Ubuntu 22.04
- PincherX 100 robot arm along with its RViz simulation environment
- Python programming environment (VS code) with matplotlib 
- Pen and paper for manual drawing

## Calculating the Orientation of the PincherX 100 Robot Arm Using Rotation Matrices

**Step 1.** Put the PincherX100 Robot Arm in the Home Pose

Note: The home pose is where all servos are centered, and the joint angles are set to zero.

![pix100 in home pose](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/af2d6ff2-800b-4d33-a313-b99abe6e55c6)

**Step 2.** Draw a Diagram of Coordinate Frames (10 points)

- Based on the above figure, draw a diagram illustrating the coordinate frames, including the base frame and end-effector frame, along with all the frames in between.
- Note that the first coordinate frame coincides with the {s} frame (the world frame of the pix100 is on the robot's base).

**Step 3.** Identify Rotation Axes and Joint Angles (10 points):

- Identify the **rotation axes** of each joint and define the **rotation angles** $`\theta_1`$ to $`\theta_4`$ based on RHR.

**Step 4.** Calculate Rotation Matrices (15 points) 

- Begin from the base frame and work your way up to find the rotation matrix representing the orientation of one frame with respect to the previous frame. For example, $`R_{01} = IRot(\hat{z},\theta_1)`$.
- Use the subscript cancellation rule to express all the rotation matrices w.r.t the base frame. For instance, $`R_{02} = R_{01}R_{12} = IRot(\hat{z},\theta_1)Rot(\hat{y},\theta_2)`$. Do this until you get the $`R_{05}`$ which represents the orientation of the tool frame w.r.t the base frame. **Note: Do not do the matrix multiplication by hand**. 

**Note:** Here you can also visualize it as successive rotations from the base frame to the tool frame. Each rotation happens w.r.t the current frame and you can post-multiply the rotation operators starting from joint 1.  

**Step 5.** Write a Python Code to Multiply Rotation Matrices (10 points)
- Write a Python code to multiply the rotation matrices as calculated in Step 4. Complete the following code:
``` Python
import numpy as np 

# angles in radians
theta_1 = 
theta_2 = 
theta_3 = 
theta_4 = 

rot_z_theta_1 = np.array([[ , , ],
                        [ , , ],
                        [ , , ]]) 

rot_y_theta_2 = 
rot_y_theta_3 = 
rot_y_theta_4 = 
eye = 

# tool frame w.r.t the base frame

rot_0_5 = 

# print the final rotation matrix

```
- Calculate the rotation matrix representing the orientation of the tool frame w.r.t the base frame when $`\theta_1 = 90^{0}, \theta_2 = -45^{o}, \theta_3 = 0, \theta_4 = 45^{o}`$. Note that if you are seeing the result in scientific notation, and want to make it more readable, add `np.set_printoptions(suppress=True)` to the above code after importing numpy (10 points). 
- Draw the calculated orientation by hand on a piece of paper (10 points).

**Step 6.** Visualize the final rotation matrix using matplotlib (10 points)
- Open a command line and install matplotlib if it is not already installed: `pip install matplotlib`
- Create a 3D visualization of the calculated matrix in Step 5. Complete the following code and combine it with the code from the previous step:
``` Python
import matplotlib.pyplot as plt

# Create vectors for the coordinate frame
origin = np.zeros(3)
x_axis = 
y_axis = 
z_axis = 

# Visualize the coordinate frame
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.quiver(*origin, *x_axis, color='r', label='X-axis')
ax.quiver(*origin, *y_axis, color='g', label='Y-axis')
ax.quiver(*origin, *z_axis, color='b', label='Z-axis')

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()
```
- Verify that the visualization is the same as what you have drawn by hand. Show a side-by-side comparison (10 points). 

**Step 7.** Verify Orientation with thet pix100's joint position control

- Use the joint position control that we learned from previous labs
- Input the above given joint angles
- Record the orientation of the pix100's tool frame w.r.t the world frame (this frame is stationary and on the robot's base in the simulation). 
- Verify that the orientation achieved by physically rotating the joints to those angles is the same as the orientation you draw by hand, and visualized using the matplotlib. Show a side-by-side comparison (15 points). Note: If necessary, you can selectively hide frames in RViz from the left-hand side panel to focus on specific frames or components during visualization. 

## Guidelines for Lab 5 report

- Submit one report per person (disclose all your collaborators including the AI ones) through Canvas. It will be graded based on the above rubric. Please see above for points for each part. 
- The codes can be submitted through a GitHub repo (with link provided) or alternatively be uploaded to the submission.  
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 5! You've done a fantastic job.