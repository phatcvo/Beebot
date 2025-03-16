Up to now, we have understood how to find the tool orientation and position (pose) of the open chain robots from joint positions that we call **forward kinematics**. In this lesson, we are going to learn how to compute the **twist** (angular and linear velocities) of the end-effector of an open chain, **given joint velocities**. But before diving into mathematical details, why do we need the Jacobian matrix to relate the end-effector velocities to the joint velocities? What are the possible applications of understanding the end-effector twist given the joint velocities? 

There are numerous applications in robotics where we want to control the velocity of the end-effector. Common applications are in industrial robotic arms used for manufacturing processes. For example, in the automotive industry, precise velocity control of robotic arms is crucial in tasks such as [painting](https://youtu.be/nee3vYTJSr4). Controlling the end-effector's velocity is essential to ensure a smooth and even application of paint, thereby guaranteeing a high-quality finish on vehicles. This means that in order to get a smooth finish, the robot arms cannot move too fast. 

<figure>
<p align="center">
<img width="655" alt="two-robot-arms-are-painting-a-car-chassis" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e0fbb151-ab61-440b-af55-215be558c9ef">
</p>
</figure>

Note: The above figures are created by DALL·E 3.

What other applications can you think of? 

Let's first start with a quick example to provide an intuitive insight into the given problem.

## Example: Jacobian Matrix for a 2 DOF Planar Open Chain Robot Arm

Consider a two-do planar open-chain robot arm modeled using STEM building blocks. 

<figure>
<p align="center">
<img width="453" alt="2dof-planar-no-background with coordinates" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d5bad6bf-b1a9-4cdd-ac64-16169a718182">
</p>
</figure>

Let's first find the forward kinematics of this simple planar robot using screw theory. It should be a piece of cake for you now, if not, review [the forward kinematics lesson](https://github.com/madibabaiasl/modern-robotics-I-course-private/wiki/Lesson-7:-Forward-Kinematics-of-Robot-Arms-Using-Screw-Theory). The vector of joint angles can be written as:

$`\begin{pmatrix}
\theta_1\\
\theta_2
\end{pmatrix}`$

The screw axes of the joints can be written as (note that screw axes are written in the robot's zero pose):

$`\mathcal{S}_1 = \begin{pmatrix}
0\\
0\\
1\\
0\\
0\\
0
\end{pmatrix}, \mathcal{S}_2 = \begin{pmatrix}
0\\
0\\
1\\
0\\
-L_1\\
0
\end{pmatrix}`$

And the pose of the end-effector w.r.t the base frame in the robot's zero position is:

$`M = \begin{pmatrix}
1 & 0 & 0 & L_1 + L_2 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}`$

Therefore, the forward kinematics of this robot relating the joint positions to the end-effector pose can be found as the following matrix:

$`T(\theta) = e^{[\mathcal{S}_1]\theta_1}e^{[\mathcal{S}_2]\theta_2} M = \begin{pmatrix}
c(\theta_1 + \theta_2) & -s(\theta_1 + \theta_2) & 0 & L_2c(\theta_1 + \theta_2) + L_1 c(\theta_1)\\
s(\theta_1 + \theta_2) & c(\theta_1 + \theta_2) & 0 & L_2 s(\theta_1 + \theta_2) + L_1 s(\theta_1)\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
\end{pmatrix}`$

Therefore, the position of the end-effector frame w.r.t the base frame can be written as:

$`\displaylines{x = L_1 c(\theta_1) + L_2c(\theta_1 + \theta_2)\\
y = L_1 s(\theta_1) + L_2 s(\theta_1 + \theta_2)}`$

Taking the time derivative of these equations yields:

$`\displaylines{\dot{x} = -L_1 \dot{\theta_1} s(\theta_1) - L_2 (\dot{\theta_1} + \dot{\theta_2}) s(\theta_1 + \theta_2)\\
\dot{y} = L_1 \dot{\theta_1} c(\theta_1) + L_2 (\dot{\theta_1} + \dot{\theta_2}) c(\theta_1 + \theta_2) \quad}`$

In robotics, we prefer to represent all our equations in matrix form; therefore, re-writing this in matrix form, we get:

$`\begin{pmatrix}
\dot{x}\\
\dot{y}
\end{pmatrix} = \begin{pmatrix}
-L_1 s(\theta_1) - L_2 s(\theta_1 + \theta_2) & -L_2 s(\theta_1 + \theta_2)\\
L_1 c(\theta_1) + L_2 c(\theta_1 + \theta_2) & L_2 c(\theta_1 + \theta_2) 
\end{pmatrix}\begin{pmatrix}
\dot{\theta_1}\\
\dot{\theta_2}
\end{pmatrix} = \begin{pmatrix}
J_1(\theta) & J_2(\theta)
\end{pmatrix}\begin{pmatrix}
\dot{\theta_1}\\
\dot{\theta_2}
\end{pmatrix} = J(\theta)\\
\dot{\theta}`$

The term $`J(\theta)`$ is called the Jacobian, and it represents the linear sensitivity of the end-effector velocity to the joint angle velocities. It shows how small changes in the joint angles/positions affect the velocity of the end-effector (note here that the velocity of the end-effector is the linear velocity of the end-effector frame in the base frame). It is actually a map that maps joint velocities to the end-effector velocities (see simulation below):  

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/5c2eadf7-6f52-4223-80ac-a4e8baf635a7

In the simulation above, you see how **changing joint velocities** that are bound to a **unit circle** are mapped to the **end-effector velocity vector** that is bound to an **ellipsoid**. This ellipsoid is called the **manipulability ellipsoid**, and it shows what **instantaneous velocity** the end-effector can achieve if you keep the joint velocities inside that unit circle. 

**Important note:** Here, we found the Jacobian that relates the **linear velocity of the end-effector frame in the space frame** to the joint velocities. 

Let's now study the columns of the Jacobian matrix. Let's re-write the above equation in terms of the linear velocity of the tool and joint velocities as follows:

$`v_{tip} = J_1(\theta) \dot{\theta_1} + J_2(\theta) \dot{\theta_2}`$ 

This velocity vector is shown in the following figure:

<figure>
<p align="center">
<img width="622" alt="tip velocity for the 2 dof planar robot_2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8b00f6d3-a4a7-4f65-8060-a00717735885">
</p>
</figure>

Note that $`J_1(\theta)`$ is dependent on joint positions and it is the end-effector velocity when joint 1 rotates at unit speed while joint 2 is kept constant (it does not move):

<figure>
<p align="center">
<img width="631" alt="first column of jacobian for 2 dof planar robot_2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/5671852d-952a-4204-b598-c412bee7535a">
</p>
</figure>


and $`J_2(\theta)`$ is also dependent on joint positions and it is the end-effector velocity when joint 2 rotates at unit speed while joint 1 does not move and is kept constant:

<figure>
<p align="center">
<img width="682" alt="second column of jacobian for 2 dof planar robot_2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7845c99e-33a6-4685-a350-67c81b3409db">
</p>
</figure>

If we draw $`J_1(\theta)`$ and $`J_2(\theta)`$ together, then the end-effector velocity will be the linear combination of these two:

<figure>
<p align="center">
<img width="292" alt="tool velocity as the linear combination of the jacobian columns" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/db22ea3c-81c5-4f80-93e2-7d478dc8b0b9">
</p>
</figure>

Now, suppose we want to determine which **joint velocities** are required to generate **desired tip velocities**. For example, for the configuration shown in the video below, you see that by **changing the tip velocity**, the required **joint velocities to generate that desired tip velocity** are changing (note the joint velocity arrows that are changing). You also see that in this configuration, the end-effector can only generate velocities in the directions bound by the **manipulability ellipsoid**:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/0447d055-2a12-4aeb-b622-7d3afb0a504a

If the configuration changes, so does the Jacobian matrix and the manipulability ellipsoid. Also, note in the video below that in certain configurations that we call them **singularities**, the manipulator's manipulability ellipsoid becomes tall and thin until, at the singular configuration, it becomes a **straight line**. In these singular configurations, the manipulator loses its degree of freedom and can only move on a straight line:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/1e09cca7-6c17-4c3d-92d3-a6107c1913c9

Mathematically, and according to the laws of linear algebra, it is possible to determine which joint velocities $`\dot{\theta_1}`$ or $`\dot{\theta_2}`$ are required to generate a desired tip velocity as long as J is **invertible** (i.e. $`J_1(\theta)`$  and $`J_2(\theta)`$ are not collinear). So, as long as the column vectors of the Jacobian matrix are not collinear (meaning that the matrix J is not singular), we can generate a tip velocity in any arbitrary direction in the x-y plane by choosing appropriate joint velocities. Note that it is important in control that we have control over the end-effector's velocity in different directions. If $`J_1(\theta)`$  and $`J_2(\theta)`$ are collinear, this means that we can only control the end-effector velocity along one specific direction. The tool will be unable to generate motion in other directions in the x-y plane independently. 

In case the **Jacobian matrix** columns are **linearly dependent**, the associated configuration is called **singular** or represents a **singularity**. This means that in certain configurations, it is difficult or impossible to control the end-effector in a certain direction. The **achievable velocity space dimension** in a singularity is **less than full rank**. 

As we saw from the above equations and from the simulations $`J_1(\theta)`$ and $`J_2(\theta)`$ are dependent on joint values $`\theta_1`$ and $`\theta_2`$. We also saw the singular configurations of this robot arm, but now let's mathematically find the configs at which $`J_1(\theta)`$ and $`J_2(\theta)`$ become collinear (the matrix J becomes singular). 

If the matrix J is singular, it means that its **determinant** is zero (this way it cannot be inverted):

$`det J = 0 \rightarrow L_1 L_2 sin(\theta_2) = 0 \rightarrow \theta_2 = 0 \text{ or } \theta_2 = \pi`$

Therefore, for $`\theta_2 = 0`$ or $`\theta_2 = \pi`$, the Jacobian is singular, and its columns are collinear, and these configurations are called **singularities** of this robot. Singularities are situations where the robot tip is unable to generate velocities in certain directions. For example, if $`\theta_2 = 0`$, the configuration of the end-effector will be like the following figure:

<figure>
<p align="center">
<img width="325" alt="2 dof planar robot in singular pose" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/1631ab05-1b78-4cc4-93f6-fac5d63c7929">
</p>
</figure>

As the figure above shows, you can only generate end-effector velocity along this line. In the simulation below, you see that at the singular configurations $`J_1(\theta)`$ and $`J_2(\theta)`$ (Jacobian matrix columns) become collinear and the manipulator can only generate velocities in certain directions:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/42f138ba-95b1-46d2-a429-dabd803ee58a

After this intuitive example of the concept of velocity kinematics, Jacobian matrix, manipulability ellipsoid, and singularities, let's see how we can calculate space and body Jacobians using screw theory. 

## Velocity Kinematics: Space Jacobian and Body Jacobian

Jacobian matrix can have different forms and interpretations, but there are two standard types of Jacobians: the space Jacobian $`J_s(q)`$ and the body Jacobian $`J_b(q)`$. The space Jacobian screw axis associated with column $`J_{si}(q)`$ is expressed in the fixed frame {s}, whereas the body Jacobian screw axis associated with column $`J_{bi}(q)`$ is expressed in the body/end-effector frame {b}.

**Space Jacobian**

Considering the forward kinematics of an n-link open chain, which can be represented as a product of exponential terms, as follows:

$`T(q_1,...,q_n) = e^{[\mathcal{S}_1]q_1}e^{[\mathcal{S}_2]q_2}...  e^{[\mathcal{S}_n]q_n}M`$

As we have seen before in previous lessons, the **spatial twist** can be represented by the following product: $`[\mathcal{V}_s] = \dot{T}T^{-1}`$ (1). $`\dot{T}`$ can be computed by taking the derivative of T using the chain rule, as follows:

$`\displaylines{\dot{T} = (\frac{d}{dt}e^{[\mathcal{S}_1]q_1})...e^{[\mathcal{S}_n]q_n} M + e^{[\mathcal{S}_1]q_1} (\frac{d}{dt}e^{[\mathcal{S}_2]q_2})...e^{[\mathcal{S}_n]q_n} M + ... \quad \quad \quad \,\\
= [\mathcal{S}_1]\dot{q_1}(e^{[\mathcal{S}_1]q_1})...e^{[\mathcal{S}_n]q_n}M + e^{[\mathcal{S}_1]q_1} [\mathcal{S}_2]\dot{q_2}(e^{[\mathcal{S}_2]q_2})...e^{[\mathcal{S}_n]q_n} M + ...}`$ (2)

Also, the transformation inverse can be computed as follows:

$`T^{-1}(q_1,...,q_n) = M^{-1} e^{-[\mathcal{S}_n]q_n} e^{-[\mathcal{S}_{n-1}]q_{n-1}} ... e^{-[\mathcal{S}_1]q_1}`$ (3)

Substituting (2) and (3) in (1), yields:

$`[\mathcal{V}_s] = [\mathcal{S}_1]\dot{q_1} + e^{[\mathcal{S}_1]q_1}[\mathcal{S}_2]e^{-[\mathcal{S}_1]q_1}\dot{q_2}+e^{[\mathcal{S}_1]q_1}e^{[\mathcal{S}_2]q_2}[\mathcal{S}_2]e^{-[\mathcal{S}_2]q_2}e^{-[\mathcal{S}_1]q_1}\dot{q}_3+...`$

We can simplify the expression using the **adjoint representation**, remember that: $`[Ad_T]\mathcal{V} = T[\mathcal{V}]T^{-1}`$. Which means:

$`\mathcal{V}_s = \mathcal{S}_1 \dot{q}_1 + [Ad_{e^{[\mathcal{S}_1]q_1}}]\mathcal{S}_2\dot{q}_2 + [Ad_{e^{[\mathcal{S}_1]q_1}e^{[\mathcal{S}_2]q_2}}]\mathcal{S}_3\dot{q}_3 + ...`$

This is a sum of n spatial twists, which can be represented in the following form:

$`\mathcal{V}_s = J_{s1}(q)\dot{q}_1 + J_{s2}(q)\dot{q}_2 + ... +  J_{sn}(q)\dot{q}_n = \begin{bmatrix}
J_{s1} & J_{s2} & ... & J_{sn}
\end{bmatrix}\begin{bmatrix}
\dot{q}_1\\
\dot{q}_2\\
.\\
.\\
.\\
\dot{q}_n
\end{bmatrix} = J_s(q)\dot{q}`$

The matrix $`J_s(q)`$ is called the **Space Jacobian**. Note that by viewing the Jacobian expression and recalling that $`\mathcal{S}_i`$ is an expression for the screw axis describing the ith joint axis in terms of the fixed frame with the robot in its **zero position**, one can see that $`[Ad_{T_{i-1}}]\mathcal{S}_i`$ is the screw axis describing the ith joint axis, but after it undergoes the rigid body displacement $`T_i`$ instead of being at zero position. 

In summary, in order to find the **space Jacobian**, we should follow these steps:

- Find the **screw axes of the joints** when the robot is in **zero position**
- Find the **adjoint map** of those screw axes for when the robot is no longer in zero position
- Find the columns of the Jacobian $`J_{s1}`$, $`J_{s2}`$, ... using this formulae: 

$`J_{s1} = \mathcal{S}_1, J_{s2} = [Ad_{e^{[\mathcal{S}_1]q_1}}]\mathcal{S}_2, J_{s3} = [Ad_{e^{[\mathcal{S}_1]q_1}e^{[\mathcal{S}_2]q_2}}]\mathcal{S}_3, ...`$

- Construct the Jacobian matrix using its columns. The Jacobian will be a $`6 \times n`$ matrix, where n is the number of joints.

**Note:** Each column of the space Jacobian represents the **spatial twist** resulting from the movement of an individual joint, with all other joint speeds set to zero except for the joint in consideration, which has a speed of one. For example, $`J_{s2}`$ is the twist of the end-effector when the speed of joint 2 is one and the speed of all other joints are zero. 

**Note:** Note that the movement of joint i is only affected by the movement of joints i-1,i-2,...,1 and not the joints i and after the i, and that's why for the Jacobian matrix column associated with that joint, we only take into consideration the adjoint map of the screw motions of the joints until joint i. For example, the movement of joint three is only affected by the movement of joints 1 and 2 (See the illustration below).

![space jacobian illustrations](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8b71a1df-ef20-4612-87cd-fe79f9a1347f)

**In summary:**

The space Jacobian $`J_s(q)`$ relates the joint velocities to the twist of the end-effector in {s} frame:

$`\mathcal{V}_s = J_s(q) \dot{q}`$

We can define the Jacobian by its columns as:

$`J_s(q) = \begin{bmatrix}
J_{s1} & J_{s2}(q) & ... & J_{sn}(q)
\end{bmatrix} \in \mathbb{R}^{6 \times n}`$

Where $`J_{s1} = \mathcal{S}_1`$, and $`J_{si}(q) = [Ad_{e^{[\mathcal{S}_1]q_1}...e^{[\mathcal{S}_{i-1}]q_{i-1}}}]\mathcal{S}_i \text{ for } i = 2,...,n`$. 

The initial column of the space Jacobian corresponds to the screw axis $`\mathcal{S}_1`$ when the robot is at its **zero configuration**. Since there are no joints between joint 1 and the {s} frame, this column is not dependent on the joint positions. Any other column i in the space Jacobian is obtained by multiplying the screw axis $`\mathcal{S}_i`$ with the transformation that represents the screw axis in the {s} frame for arbitrary joint positions.

In summary, for the space Jacobian, we take the joint screw axes $`\mathcal{S}_1`$ to $`\mathcal{S}_n`$, defined in the space frame {s} when the robot is at the zero configuration, and transform them to the n columns of the space Jacobian at **any arbitrary joint configuration**. 

**Class Activity**

Write a MATLAB code and calculate the twist of the end-effector for the 2-dof planar robot arm at the start of this lesson using both the step-by-step method for the space Jacobian (outlined above) and using the equation:

$`[\mathcal{V}_s] = \dot{T}T^{-1} = \begin{pmatrix}
[\omega_s] & v_s\\
o & 0
\end{pmatrix}`$

Show that they are the same. 

**Question 1:** Why is this Jacobian different than the **Coordinate Jacobian** that we calculated at the start of this lecture? Using the concepts that we learned in this lesson and the velocities in the robotics lesson, write a Matlab code that can convert the linear part of the spatial twist to the velocity of the end-effector frame in the space frame. 

**Question 2:** Does the 2R robot arm have singularities when considering the “output” to be the full 6-dof twist (as opposed to just the 2-dof linear velocity ($`\dot{x}`$, $`\dot{y}`$))?

**Body Jacobian**

For body Jacobian, the screw axes are defined in the body frame. Body Jacobian can be expressed as the following equations:

$`\mathcal{V}_b = J_b(q)\dot{q}, \text{ where } J_b(q) \in \mathbb{R}^{6 \times n}`$

The **body Jacobian** transforms **joint velocities into the body twist**. Since we did not go into details about how to find the screw axes in the body frame back in the forward kinematics lesson (which in my mind was not necessary), to find the body Jacobian, we only learn the mapping that can take the space Jacobian to the body Jacobian. The **adjoint transformation** can be used to map the space Jacobian to the body Jacobian:

$`J_b(q) = [Ad_{T_{bs}}]J_s(q)`$

$`J_s(q) = [Ad_{T_{sb}}]J_b(q)`$

Therefore, $`J_b`$ is obtained from $`J_s`$ by the matrix adjoint of $`T_{bs}`$, and $`J_s`$ is obtained from $`J_b`$ by the matrix adjoint of $`T_{sb}`$.

## A Discussion about Rank of the Jacobian Matrix

We saw that the Jacobian matrix can relate the joint velocities to the end-effector twist (spatial twist or body twist):

$`\mathcal{V}_{6 \times 1} = J_{6 \times n}(q)\dot{q}_{n \times 1}`$

The Jacobian is a $`6\times n`$ matrix, where n is the number of joints. This means that the rank of the Jacobian can be no greater than the minimum of 6 and n: $`\text{rank }J(q) \leq min(6,n)`$

- We know from linear algebra that the Jacobian matrix is called **full rank** at a configuration q if the rank is equal to the minimum of 6 and n: $`\text{J is full rank if rank }J(q) = min(6,n)`$. 

- The Jacobian is **singular** at a configuration $`q^{*}`$ if the rank of the Jacobian at $`q^{*}`$ is less than the maximum rank the Jacobian can achieve at some configuration: $`\text{singular at } q^{*} \text{ if rank }J(q^{*}) < \text{max rank }J(q) \text{(at some q)}`$

As we saw before, in a singular configuration, the robot loses the ability to move in one or more directions. 

## Types of Robots based on their Jacobians

- **Kinematically Dificient Arms**: If the number of joints of the arms n is less than 6, then the $`6 \times n`$ Jacobian matrix will have more rows than columns, and it is "tall." In this case, the **set of reachable configurations** for the end-effector is **less than 6-dimensional**, so we call such robots kinematically deficient. Our Pincherx 100 robot arm has 4 joints so it is in this category. Being kinematically deficient is not a bad thing; it just means that the robot is not capable of general motion at the end-effector. 

- **General-purpose manipulators**: If the number of robot joints is 6, then the Jacobian matrix is $`6 \times 6`$, and these robots are capable of **general 6-dimensional rigid-body motion** at their end-effectors, and that's why they are called general-purpose manipulators. The Kinova's 6R robot arm that you saw in the last homework is a general-purpose manipulator. 

- **Redundant manipulators**: If the robot has more joints than 6, then the Jacobian matrix has more columns than rows, and it is "fat." These types of robots are called **redundant** since they can achieve the same end-effector twist with different joint velocities. Can you show redundancy in your own arm?

<!-- ## Class Activity 

Write a MATLAB/Python function that gets the screw axes expressed in the fixed frame and a list of joint angles and computes the Space Jacobian. It will also outputs body Jacobian using the adjoint transformation. --> 

## References 

- Kevin, M.L. and Frank, C.P., 2017. Modern robotics: mechanics, planning, and control
- Murray, R.M., Li, Z. and Sastry, S.S., 2017. A mathematical introduction to robotic manipulation. CRC press.
- Corke, P., 2023. Robotics, Vision and Control: Fundamental Algorithms in Python (Vol. 146). Springer Nature
- https://www.wolfram.com/mathematica/
