## Introduction

In project 2, you will design a scenario in which you will calculate the **inverse kinematics** of the robot arm and use it to implement a task such as pick and place. This task will be both done **with and without camera feedback**. In the first part of project 2, you are going to implement inverse kinematics using the two approaches that we discussed in the lecture.

For implementation of this part in ROS2, you will start with updating the configuration yaml files to set the robot motion mode to 'position.' Following that, you will create the helper functions that will do all the calculations and algorithm implementations for you. Finally, you will create your custom APIs.

The guides below will help you implement this part of the project faster. Remember that this is just a guide for you, and at the end, you should be able to design **your own experiment**. 

And, here are some cool implementations by me and students in the previous semesters in case you are interested:

https://youtube.com/shorts/J8nSxMFDyu0

https://youtube.com/shorts/v8eUALYdJ18

https://youtube.com/shorts/aYxGKKSEHH0

https://youtu.be/-67sP6ab2h4

https://youtu.be/xOmVYJphdRo

## Setup and helper functions

**Guide 1**. Change the yaml file content to 'position' mode.

After that, create a **new script** called px100_IK_ex.py (or any appropriate name you choose). You can put that in the same package that we have created for the velocity kinematics before. In this script, develop a **main function** and a **special class** for **inverse kinematics**. I called this class as ourAPI. The code below gives you a template to start this:

```python
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from math import atan2, sqrt, pi, acos, sin, cos, asin
from scipy.linalg import logm, expm
import numpy as np

class ourAPI:
    def __init__(self):
        # Robot parameters
        self.L1 = 
        self.L2 = 
        self.Lm =  
        self.L3 = 
        self.L4 = 
        self.S = np.array([[0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0]]) # Screw axes
        self.M = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0]]) # End-effector M matrix
```

The schematic of the robot arm in the home position is depicted in the figure below:

<img width="622" alt="pix100 robot arm in home position" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/58866ed1-363c-484e-b6d8-5b20ef336612">

**Guide 2.** You will need to start writing the **class helper functions** and adding them to the class methods one by one. Remember the numerical inverse kinematics algorithm:

(a) Initialization: Given $`T_{sd}`$ (transformation of the desired frame with respect to space frame) and an initial guess $`q_o \in R^n`$, set $`i = 0`$.

(b) Set $`[{\mathcal{V}}_b] = log(T_{sb}^{-1} (q_i) T_{sd})`$. While the algorithm is not converged:

* Set $`q_{i+1} = q_i + J^\dagger_b(q_i){\mathcal{V}}_b`$.
* Increment $`i`$.

The numerical method relies on the **forward kinematics** (homogeneous transformation of the end-effector with respect to the base frame $`T_{sb}`$). This requires screw axes assignments and computing the exponential form of the transformation matrix **that you have done before**. Therefore you need to create the function that will convert a combination of a screw axis and joint angle to a homogeneous transformation matrix (exponentiation), as follows (**fill in the missing spaces**):

```python
def screw_axis_to_transformation_matrix(self, screw_axis, angle):
    """
    Convert a screw axis and angle to a homogeneous transformation matrix.

    Parameters:
    - screw_axis: A 6D screw axis [Sw, Sv], where Sw is the rotational component
                  and Sv is the translational component.
    - angle: The angle of rotation in radians.

    Returns:
    - transformation_matrix: The 4x4 homogeneous transformation matrix
                            corresponding to the input screw axis and angle.
    """
    assert len(screw_axis) == 6, "Input screw axis must have six components"

    # Extract rotational and translational components from the screw axis
    Sw = 
    Sv = 

    # Matrix form of the screw axis
    screw_matrix = np.zeros((4, 4))
    screw_matrix[:3, :3] = np.array([[ , , ],
                                    [ , , ],
                                    [ , , ]])
    screw_matrix[:3, 3] = Sv

    # Exponential map to get the transformation matrix
    exponential_map = expm(angle * screw_matrix)
    
    return exponential_map
``` 

**Guide 3**. In the numerical method, you will need to get the **twist vector** from the **matrix form of the twist vector**. Fill the missing lines in the following helper function:

```python
def twist_vector_from_twist_matrix(self, twist_matrix):
    """
    Compute the original 6D twist vector from a 4x4 twist matrix.

    Parameters:
    - twist_matrix: A 4x4 matrix representing the matrix form of the twist 

    Returns:
    - twist_vector: The 6D twist vector [w, v] corresponding to the input
                    twist matrix.
    """
    assert twist_matrix.shape == (4, 4), "Input matrix must be 4x4"

    w = 
    v = 

    return np.concatenate((w, v))
```

**Guide 4**. In the **numerical algorithm**, you need to compute the **body jacobian** of the robot iteratively. Let's create a jacobian function that takes in the arm angles and gives out the body jacobian. You can also refer to the robot documentation to get the parameters you need (https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/px100.html). Use your knowledge of screw theory to write down the expression for the Jacobian (you already **have the space Jacobian**, use that to compute the body version):

```python
def body_jacobian(self, angles):
    # Calculate the body jacobian
    J = np.array([[, , , ],
                    [, , , ],
                    [, , , ],
                    [, , , ],
                    [, , , ],
                    [, , , ]])
    return J
```

## Algorithm implementation

**Guide 1.** You will start by implementing the **geometric method**. You need to refer to the following link: https://github.com/cychitivav/px100_ikine to understand and write down the missing equations in the following function code (for the geometric method part):

```python
def geom_IK(self, Td):
    """
    Gives joint angles using the geometric method.
    """ 
    # Get the end-effector coordinates
    Xt = Td[0,3]; Yt = Td[1,3]; Zt = Td[2,3]

    # Get the end-effector approach vector
    ax = Td[0,0]; ay = Td[1,0]; az = Td[2,0]

    # Get the wrist vector
    wx = 
    wy = 
    wz = 
    
    # Calculate some intermediate variables
    r = 
    c = 
    sai = 
    phi = 
    gamma = 
    alpha = 
    theta_a = 

    # Get corresponding joint angles using geometry (elbow-up solution)
    q1 =  # Waist angle
    q2 =  # Shoulder angle
    q3 =  # Elbow angle
    q4 =  # Wrist angle

    # Return angles
    return [q1, q2, q3, q4]
```

**Guide 2**. You also need to implement the **numerical solution** using the Newton-Raphson algorithm given. Fill the missing lines in the following code and refer to the previously mentioned algorithm:

```python
def num_IK(self, Tsd, InitGuess):
    """
    Gives joint angles using numerical method.
    """
    for i in range(100):
        # Calculate the end-effector transform (Tsb) evaluated at the InitGuess using the helper functions that you wrote at the beginning.
        Tsb = 

        # Compute the body twist
        matrix_Vb = logm( ); Vb = # use the helper function at the beginning to extract the vector

        # Compute new angles
        NewGuess = 
        print(f"Iteration number: {i} \n")

        # Check if you're done and update initial guess
        if(np.linalg.norm(abs(NewGuess-InitGuess)) <= 0.001):
            return [NewGuess[0], NewGuess[1], NewGuess[2], NewGuess[3]] 
        else:
            InitGuess = NewGuess
    print('Numerical solution failed!!')
```

## Main creation

**Guide 1.** Now that we're done with our class, we need to create our **main function**. I just created a sample experiment where I gave different end-effector poses for different phases of the cube alignment task below (home-grasp-release). Note that this is just an example and you need to develop your own experiment. 

```python
# sample code 

def main():
    # Determine the desired end-effector transform
    Td_grasp = np.array([[,  ,  ,  ],
                    [,  ,  ,  ],
                    [,  ,  , ],
                    [,  ,  ,  ]]) # Gripping location
    Td_release = np.array([[,  ,  ,  ],
                    [,  ,  ,  ],
                    [,  ,  ,  ],
                    [,  ,  ,  ]]) # Throwing location

    # Create experiment objects (use robot API + our custom API)
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    my_api = ourAPI()

    # Start with home position
    bot.arm.go_to_home_pose()

    # toggle between the geometric method and the numerical method below
    # record the answers that you get in both cases. report your observations. 

    # Go to gripping position and grip
    joint_positions = my_api.geom_IK(Td_grasp) # Geometric inverse kinematics
    #joint_positions = my_api.num_IK(Td_grasp, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.grasp(2.0) # Grip
    
    # Go to throwing position and throw
    joint_positions = my_api.geom_IK(Td_release) # Geometric inverse kinematics
    #joint_positions = my_api.num_IK(Td_release, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.release(2.0) # Release
    
    # End mission
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    bot.shutdown()


if __name__ == '__main__':
    main()
```

## Testing the code

**Guide 1**. Launch the Rviz simulation. 

**Guide 2**. Run the new .py file from a new terminal tab or from your IDE.

## Guidelines for Project 2 - Part 1

- Submit one code and video per group. 
- Each group will give a **live demonstration** of their designed task. For part 1, you will design the experiment with **no camera feedback**.
- Be **prepared** for questions (both theoretical and implementation) during the presentation. I will give new desired positions and ask you to change the code on the spot for the robot to reach it. 
- **Important note:** In order for the inverse kinematics problem to have a solution and for the robot to operate safely, it is recommended that you design the location of objects within 70% of the reach/span (**recommend workspace**). See the link below for more information about the robot's workspace:

https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications.html#workspace 

Good luck!
