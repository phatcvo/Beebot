In the previous lesson, we became familiar with **rotation matrices**, and we saw that the **nine-dimensional** space of rotation matrices subject to six constraints (three unit norm constraints and three orthogonality constraints) could be used to implicitly represent the **three-dimensional** space of orientations.

There are also other methods to express the orientation with a **minimum number of parameters**. **Exponential coordinates** that define an axis of rotation and the angle rotated about that axis (it is also sometimes called axis-angle representation), the three-parameter **Euler angles**, the three-parameter **roll-pitch-yaw angles**, and the **unit quaternions** (use four variables subject to one constraint) are some of the methods to express the orientations with a fewer number of parameters. We will discuss the exponential coordinates and one type of Euler angles (due to their importance) in this lesson and the rest will be left for your own study (they are not required in our class).

## Exponential Coordinate (axis-angle) Representation of Orientation

Exponential coordinates of orientation are a three-parameter representation of orientation in which a rotation matrix R can be parameterized in terms of a rotation axis $`\hat{\omega}`$ (a unit vector: $`\hat{\omega} \in  \mathbb{R}^3, ||\hat{\omega}|| = 1`$) and an angle of rotation $`\theta`$ about that axis:

<figure>
<p align="center">
<img width="388" alt="rotation-about-arbitrary-axis" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/a7270fdd-5c31-4664-bd2c-fb164e759d4e">
<figcaption> <p align="center">The Exponential Coordinate Representation of Orientation</figcaption> </p>
</p>
</figure>

Here, note that if a frame initially coincident with the space frame {s} were rotated by $`\theta`$ about $`\hat{\omega}`$, then its final orientation with respect to {s} could be represented by R:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7bc95113-63b9-40ee-b63a-16ce54e83fa0

If we combine these two parameters, we get the exponential coordinate representation of orientation as: $`\hat{\omega}\theta \in  \mathbb{R}^3`$. If we write $`\hat{\omega}`$ and $`\theta`$ individually, we get the axis-angle representation of rotation.

## Relationship Between Exponential Coordinates and a Rotation Matrix

The relationship between a rotation matrix and exponential coordinates of orientation can be explained using **Rodrigues’s formula** as:

$`Rot(\hat{\omega}, \theta) = e^{[\hat{\omega}]\theta} = I + sin\theta[\hat{\omega}] + (1-cos\theta)[\hat{\omega}]^2 \in SO(3)`$

Here, $`[\hat{\omega}]`$ is the **skew-symmetric** matrix representation of the vector $`\hat{\omega}`$. This is the same formula that we had in the Rotation Matrices lesson for the general case of rotation about an arbitrary axis. 

**Start of Math Note**

By definition, if we have a vector defined as: $`a = \begin{bmatrix}
a_1 & a_2 & a_3\\
\end{bmatrix}^T \in  \mathbb{R}^3`$, then the 3×3 skew-symmetric matrix representation of $`a`$ can be defined as:

$`[a] = \begin{bmatrix}
0 & -a_3 & a_2\\
a_3 & 0 & -a_1\\
-a_2 & a_1 & 0
\end{bmatrix}`$

Note that since [a] is a skew-symmetric matrix, then: $`[a] = -[a]^T`$. 

**End of Math Note**

If you want to know the physics and mathematics behind Rodrigue's formula and why it can represent a rotation matrix, you can watch the following video (the proof is not required in our class):

https://youtu.be/I2p1lCcmOsM?list=PLlqdnFs9xNwpD9zJr8BgAbfHH3AyixTqt

Rodrigue's formula uses the **matrix exponential** to construct a rotation matrix from a rotation axis $`\hat{\omega}`$ and an angle $`\theta`$. Formally, we can define this as:

Given a vector $`\hat{\omega} \theta \in  \mathbb{R}^3`$ in which $`\theta`$ is any scalar and $`\hat{\omega} \in  \mathbb{R}^3`$ is a unit vector, then the matrix exponential of $`[\hat{\omega}]\theta = [\hat{\omega}\theta] \in so(3)`$ is $`R = Rot(\hat{\omega}, \theta) = e^{[\hat{\omega}]\theta} = I + sin\theta[\hat{\omega}] + (1-cos\theta)[\hat{\omega}]^2 \in SO(3)`$. In short, according to Rodrigue's formula, the matrix exponential takes the skew-symmetric representation of exponential coordinates $`\hat{\omega}\theta`$ and calculates the corresponding rotation matrix that is achieved by rotating about $`\hat{\omega}`$ by $`\theta`$ from an initial orientation I:

**Exponentiation** (maps Lie algebra to Lie group): $`[\hat{\omega}]\theta \in so(3) \rightarrow R \in SO(3)`$. The set of all 3×3 real skew-symmetric matrices is called so(3), which is called the Lie algebra of the Lie group SO(3). In general, the space of n×n skew-symmetric matrices is called so(n) and can be defined as: $`so(n) = \{S \in  \mathbb{R}^{n \times n}: S^T = -S\}`$.

We can also conclude that the matrix exponential from Rodrigue's formula can act as an operator and rotate a frame or a vector:

- $`e^{[\hat{\omega}]\theta} p`$: here the matrix exponential can rotate the vector $`p \in \mathbb{R}^3`$ about the fixed frame axis $`\hat{\omega}`$ by an angle $`\theta`$. 

- if R is a rotation matrix with 3 column vectors:
  - then $`R' = e^{[\hat{\omega}]\theta} R = Rot(\hat{\omega}, \theta) R`$ is the orientation achieved by rotating R by $`\theta`$ about the axis $`\hat{\omega}`$ in the **fixed** frame. 
  - and $`R'' = R e^{[\hat{\omega}]\theta} = R Rot(\hat{\omega}, \theta)`$ is the orientation achieved by rotating R by $`\theta`$ about the axis $`\hat{\omega}`$ in the **body** frame.  

Now let's solve the reverse problem. What if we have a rotation matrix $`R \in SO(3)`$ and we want to find the unit axis and angle that can represent that? For any rotation matrix $`R \in SO(3)`$, we can always find a unit vector $`\hat{\omega}`$ and scalar $`\theta`$ such that $`R = e^{[\hat{\omega}]\theta}`$. This brings up a new discussion about the **matrix logarithm** of rotations. The **matrix logarithm** is the inverse of the **matrix exponentiation** discussed above:

$`\begin{cases}
    \exp : [\hat{\omega}]{\theta} \in so(3) \rightarrow \mathbb{R} \in SO(3) \\
    \log : \mathbb{R} \in \text{SO}(3) \rightarrow [\hat{\omega}]{\theta} \in so(3) 
\end{cases}`$. 

So now the problem is we have the rotation matrix $`R \in SO(3)`$ as:

$`R = 
\begin{pmatrix}
    r_{11} & r_{12} & r_{13} \\
    r_{21} & r_{22} & r_{33} \\
    r_{31} & r_{32} & r_{33}
\end{pmatrix}`$

and we want to find the corresponding $`\hat{\omega} = (\hat{\omega}_1, \hat{\omega}_2, \hat{\omega}_3)`$ and $`\theta`$. In order to find these, we need to equate the matrix exponential formula to the rotation matrix given:

$`\begin{pmatrix}
c_{\theta} + {\hat{\omega}_1}^2 (1-c_{\theta}) & \hat{\omega}_1\hat{\omega}_2(1-c_{\theta})-\hat{\omega}_3 s_{\theta} & \hat{\omega}_1 \hat{\omega}_3 (1-c_{\theta}) + \hat{\omega}_2 s_{\theta} \\
\hat{\omega}_1\hat{\omega}_2(1-c_{\theta}) + \hat{\omega}_3 s_{\theta} & c_{\theta} + {\hat{\omega}_2}^2(1-c_{\theta}) & \hat{\omega}_2\hat{\omega}_3(1-c_{\theta})-\hat{\omega}_1 s_{\theta}\\
\hat{\omega}_1\hat{\omega}_3(1-c_{\theta})-\hat{\omega}_2 s_{\theta} & \hat{\omega}_2\hat{\omega}_3(1-c_{\theta}) + \hat{\omega}_1 s_{\theta} & c_{\theta} + {\hat{\omega}_3}^2(1-c_{\theta})
\end{pmatrix} = \begin{pmatrix}
    r_{11} & r_{12} & r_{13} \\
    r_{21} & r_{22} & r_{33} \\
    r_{31} & r_{32} & r_{33}
\end{pmatrix}`$

Where $`s_{\theta} = sin\theta`$, and $`c_{\theta} = cos\theta`$. 

After doing some math that can be found in the above given video, we can get the skew-symmetric matrix form of the axis of rotation for the given rotation matrix R as:

$`[\hat{\omega}] = \frac{1}{2sin\theta}(R-R^T)`$, and $`1 + 2cos\theta = r_{11} + r_{22} + r_{33}`$.

Note here that $`[\hat{\omega}]`$ is not well-defined if $`\theta`$ is an integer multiple of $`\pi`$ and those angles are called the singularities of the representation. Note that singularities are unavoidable for any 3-parameter representation of orientation. Euler angles and roll-pitch-yaw angles have the same singularities. Therefore, in order to find $`\theta`$, we can have two cases:

- $`sin\theta \ne 0`$ ($`\theta`$ is not an integer multiple of $`\pi`$): With $`[\hat{\omega}]`$ found above and $`\theta`$ from this equation, R can be expressed as the exponential matrix $`R = e^{[\hat{\omega}]\theta}`$. 
- $`sin\theta = 0`$ ($`\theta = k\pi`$ and k is some integer): 
  - Case 1: k is an even integer, then no matter what the axis of rotation $`\hat{\omega}`$ is, we have rotated back to R = I, so the axis of rotation $`\hat{\omega}`$ is undefined. 
  - Case 2: k is an odd integer ($`\theta = (\pm\pi, \pm 3\pi, ...)`$), and then $`R = I + 2[\hat{\omega}]^2`$. For this case, we can find the rotation axis by equating this rotation matrix to the given rotation matrix. 

In summary: if we are given $`R \in SO(3)`$, using the approach above, we can find a $`\theta`$ and a unit rotation axis $`\hat{\omega} \in  \mathbb{R}^3, ||\hat{\omega}|| = 1`$ such that $`e^{[\hat{\omega}]\theta} = R`$. The vector $`\hat{\omega} \theta \in  \mathbb{R}^3`$ is the exponential coordinate of R and the skew-symmetric matrix $`[\hat{\omega}]\theta \in so(3)`$ is the matrix logarithm of R. 

**Example.** If the rotation matrix R is defined by successive rotations about the z-axis of the space frame by 180 deg followed by a rotation about the x-axis of the space frame by 90 deg as (notice the order in which the rotations are written):

$`R = Rot(\hat{x}, 90^\circ) Rot(\hat{z}, 180^\circ) =
\begin{pmatrix}
-1 & 0 & 0 \\
0 & 0 & -1 \\
0 & -1 & 0 \\
\end{pmatrix}`$

The rotation operators about the x and z axes used in this calculation can be found in the rotation matrices lesson. If you like to visualize this rotation, you can watch the following demonstration and verify that the orientation of the final frame with respect to the base frame is indeed the rotation matrix above:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d577edf1-e3cd-4647-afd2-d1b0ce82e1d8

We want to find a unit vector $`\hat{\omega}`$ and an angle $`\theta`$ such that $`e^{[\hat{\omega}]\theta} = R`$.

**Solution:** Let's first calculate $`\theta`$:

$`1 + 2c_{\theta} = r_{11} + r_{22} + r_{33} \rightarrow \theta = \pi`$.

Therefore, the angle of rotation is an odd integer multiple of $`\pi`$ and we have:

$`R = I + 2[\hat{\omega}]^2 \rightarrow R = \begin{pmatrix}
2{\hat{\omega}_1}^2 - 1 & 2{\hat{\omega}_1}{\hat{\omega}_2} & 2{\hat{\omega}_1}{\hat{\omega}_3}\\
2{\hat{\omega}_1}{\hat{\omega}_2} & 2{\hat{\omega}_2}^2 - 1 & 2{\hat{\omega}_2}{\hat{\omega}_3}\\
2{\hat{\omega}_1}{\hat{\omega}_3} & 2{\hat{\omega}_2}{\hat{\omega}_3} & 2{\hat{\omega}_3}^2 - 1
\end{pmatrix}`$

Now equate this to the given orientation:

$`\begin{pmatrix}
2{\hat{\omega}_1}^2 - 1 & 2{\hat{\omega}_1}{\hat{\omega}_2} & 2{\hat{\omega}_1}{\hat{\omega}_3}\\
2{\hat{\omega}_1}{\hat{\omega}_2} & 2{\hat{\omega}_2}^2 - 1 & 2{\hat{\omega}_2}{\hat{\omega}_3}\\
2{\hat{\omega}_1}{\hat{\omega}_3} & 2{\hat{\omega}_2}{\hat{\omega}_3} & 2{\hat{\omega}_3}^2 - 1
\end{pmatrix} = \begin{pmatrix}
-1 & 0 & 0\\
0 & 0 & -1\\
0 & -1 & 0
\end{pmatrix}`$

Note here that since $`\hat{\omega}`$ is a unit vector, we have: $`{\hat{\omega}_1}^2 + {\hat{\omega}_2}^2 + {\hat{\omega}_3}^2 = 1`$. 

From the equation above, we can have:

$`\begin{cases}
  2{\hat{\omega}_1}^2 - 1 = -1 \\
  2{\hat{\omega}_2}^2 - 1 = 0 \\
 2{\hat{\omega}_3}^2 - 1 = 0 
\end{cases} \quad \begin{cases}
  \hat{\omega}_1 \hat{\omega}_2 = 0 \\
  \hat{\omega}_1 \hat{\omega}_3 = 0\\
 \hat{\omega}_2 \hat{\omega}_3 = -\frac{1}{2} 
\end{cases}`$

Solving these equations:

$`\hat{\omega} = (0, \frac{1}{\sqrt{2}}, -\frac{1}{\sqrt{2}}) \quad \text{or} \quad \hat{\omega} = (0, -\frac{1}{\sqrt{2}}, \frac{1}{\sqrt{2}})`$.

Here, note that when the angle of rotation is an odd integer multiple of $`\pi`$, then rotations about $`\hat{\omega}`$ and $`-\hat{\omega}`$ will yield the same orientation. 

So, the orientation of the frame achieved by a rotation from the initial orientation about $`\hat{\omega}`$ calculated above by an angle 180 deg gives the same orientation as the frame if it was rotated first about the z-axis of the space frame by 180 deg followed by a rotation about the x-axis of the space frame by 90 deg.

## Euler Angles 

In the lesson about the degrees of freedom of a robot, we learned that there are at least three independent parameters needed to express the orientation of a rigid body. At the start of this lesson, we learned about the exponential coordinate representation for the orientation which is a three-parameter representation for a rotation matrix R, and parameterizes the rotation matrix using a unit axis of rotation and the angle of rotation about this axis.

There are also other explicit representations that are useful in different applications when dealing with orientations. In this part, we will talk about Euler Angles and will see how we can use this representation to parameterize an orientation. Euler angles are commonly used when controlling the 3 degrees of freedom (DOF) of the wrist in robotic inverse kinematics due to their effectiveness in representing and controlling orientation. 

Euler angles are a set of **three angles** that describe the orientation of a rigid body or object in three-dimensional space by specifying a **sequence of rotations**. There are different sequences or conventions for defining Euler angles, and each sequence represents a unique way of describing the orientation. Here are some commonly used types of Euler angles: XYZ Euler Angles (Roll-Pitch-Yaw), ZYX Euler Angles, ZYZ Euler Angles, etc. Here, we will discuss **ZYX Euler Angles** and the rest can be easily deduced from this.

### ZYX Euler Angles

Consider the body frame {b} is instantaneously attached to a rigid body and was initially aligned with the space frame {s}:

<figure>
<p align="center">
<img width="388" alt="rigid-body-zyx-Euler-engles-1" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/f257ef18-17d5-4357-9fee-07a6ee051193">
</p>
</figure>

The ZYX Euler angles $`(\alpha,\beta,\gamma)`$ are specified by a rotation of the body by $`\alpha`$ about the body z-axis, then by $`\beta`$ about the body y-axis, and finally by $`\gamma`$ about the body x-axis. Then the final orientation of the body can be found by the multiplication of the rotation operators about the z, y, and x-axes:

$`\begin{split}
R(\alpha,\beta,\gamma) &= I \text{Rot}(\hat{z},\alpha) \text{Rot}(\hat{y},\beta) \text{Rot}(\hat{x},\gamma) \\
&= \begin{pmatrix}
c_{\alpha}c_{\beta} & c_{\alpha}s_{\beta}s_{\gamma} - s_{\alpha} c_{\gamma} & c_{\alpha}s_{\beta}c_{\gamma} + s_{\alpha} s_{\gamma}\\
s_{\alpha}c_{\beta} & s_{\alpha}s_{\beta}s_{\gamma} + c_{\alpha} c_{\gamma} & s_{\alpha}s_{\beta}c_{\gamma}-c_{\alpha}s_{\gamma}\\
-s_{\beta} & c_{\beta}s_{\gamma} & c_{\beta}c_{\gamma}
\end{pmatrix}
\end{split}`$

These rotation operators are found in the Rotation Matrices lesson, and the above equation for the final orientation of the body can be interpreted as starting from the initial orientation when the body frame is coincident with the space frame and thus R = I, then rotate it by $`\alpha`$ about the body frame z-axis, then by $`\beta`$ about the body frame y-axis and finally by $`\gamma`$ about the body frame x-axis to reach the final orientation (note the order in which the rotation operators are written). Visualize this as the following consecutive rotations:

![ZYX-euler-angles-demonstration](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c35c0175-bdb0-418a-8d23-25c0e8234211)

Video below shows a demonstration of the ZYX Euler angles to represent the orientation:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/2d24047f-9119-449e-aeb2-ab73a10fef6e
<figcaption> <p align="center">Demonstration of the ZYX Euler Angle Representation. Shout out to Husam Aldahiyat for writing the MATLAB code for this simulation.</figcaption> </p>

The opposite problem, which is so common in finding the inverse kinematics of a robotic arm wrist, is determining the Euler angles $`(\alpha,\beta,\gamma)`$ that correspond to a given desired orientation matrix R, allowing the wrist mechanism to achieve a specific end-effector pose in three-dimensional space. For this, we will be given an arbitrary rotation matrix $`R \in SO(3)`$ as:

$`R =
\begin{pmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33} \\
\end{pmatrix}`$

does there exist $`(\alpha,\beta,\gamma)`$ that satisfies $`R(\alpha,\beta,\gamma)`$ found by rotation operators above? Solving this equation is a fundamental step in addressing the inverse kinematics problem. 

Since we are dealing with cosines and sines here, Euler angles like any other 3-parameter representation of orientation suffer from singularities. When cosines or sines approach zero or values close to zero, it can lead to singularities in the Euler angle representations, resulting in situations where multiple sets of Euler angles can represent the same orientation or where certain orientations cannot be effectively represented. This is problematic in practical applications where the robot’s controller will be confused at those configurations and can generate solutions that can cause problems.

There are also other ways to represent orientation like **unit quaternions** and the **Cayley-Rodrigues** parameters that you can refer to the videos below if you are interested in learning them (they are not required in our class):

- https://youtu.be/Ek9fySVzuq4
- https://youtu.be/gfqYTZ9s26o

## References

**Important Note:** If you read 100 robotics books/articles, you will encounter 99 different notations. I read at least 5 different robotics books (Craig, Siciliano, Asada, Peter Corke, and our main textbooks (Lynch and MLS)), and the Modern Robotics book by Kevin Lynch offers a better notation. I found it easier for learning and research. 

- Kevin, M.L. and Frank, C.P., 2017. Modern robotics: mechanics, planning, and control
- Murray, R.M., Li, Z. and Sastry, S.S., 2017. A mathematical introduction to robotic manipulation. CRC press.
- Corke, P., 2023. Robotics, Vision and Control: Fundamental Algorithms in Python (Vol. 146). Springer Nature.
