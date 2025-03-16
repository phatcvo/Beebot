Up to this point, we have discussed **orientations in robotics**, and we have become familiarized with different representations to express orientations in robotics. In this lesson, we will start with configurations, and we will learn about homogeneous transformation matrices which are great tools for expressing **pose** (both **position and orientation**) in a compact matrix form.

In the lesson about the **degrees of freedom of a robot**, we saw that in 3D physical space, we need **six parameters** to explicitly represent the position and orientation of a rigid body (three parameters for the position and three parameters for the orientation).

In order to represent the pose of a rigid body, we adopt a **sixteen-dimensional** $`4 \times 4`$ matrix subject to **ten constraints** ($`16 - 10 = 6`$). This matrix can be used to express the configuration of the body frame relative to the fixed frame.

## Robot's Pose in Space 

In the [rotation matrices lesson](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-3:-Orientation-in-Robotics-(Rotation-Matrices)), we saw that the **body frame** is a fixed frame that is instantaneously attached to the moving body and the **space frame** is the fixed frame that is fixed somewhere in the space. Then the configuration of the body can be expressed by the pair $`(R,p)`$ in which $`R \in SO(3)`$ is the $`3 \times 3`$ rotation matrix representing the orientation of the {b} frame relative to the {s} frame (which we discussed thoroughly in previous lessons) and $`p \in \mathbb{R}^3`$ is a 3-vector that is the position of the origin of the {b} frame relative to the {s} frame:

<figure>
<p align="center">
<img width="514" alt="represenation-robot-motion-space" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b4458c98-759f-471e-8dc5-6c666640be80">
<figcaption> <p align="center">The configuration of a rigid body can be expressed using the pair (R,p) in which R and p are the orientation and the position of the origin of the body frame {b} relative to the space frame {s}, respectively.</figcaption> </p>
</p>
</figure>

If we package both $`R`$ and $`p`$ into a single $`4 \times 4`$ matrix form, we get the homogenous transformation matrix representation of the robot’s configuration T as:

$`T = \begin{pmatrix}
R_{3 \times 3} & p_{3 \times 1} \\
o_{1 \times 3} & 1 
\end{pmatrix} = \begin{pmatrix}
r_{11} & r_{12} & r_{13} & p_1 \\
r_{21} & r_{22} & r_{23} & p_2 \\
r_{31} & r_{32} & r_{33} & p_3 \\
0 & 0 & 0 & 1
\end{pmatrix}`$

where $`R`$ is a **rotation matrix** in the Special Orthogonal Group SO(3) and $`p \in \mathbb{R}^3`$ is a column vector. Set of all $`4 \times 4`$ real matrices, T described above is called the **Special Euclidean group** SE(3), which is the group of rigid body motions. An element $`T \in SE(3)`$ can also be expressed as the pair $`(R,p)`$.

The advantage of adopting the $`4 \times 4`$ matrices to implicitly represent the configuration of a robot is the simple algebraic calculations that can be used to work with these matrices.

## Robot's Pose on a Plane

As many robotic mechanisms are **planar**:

<figure>
<p align="center">
<img width="680" alt="robot-motion-on-plane-toy-car" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/5dec57dc-fd3a-4997-a05b-f31f8e72e295">
<figcaption> <p align="center">The configuration of a toy car on a plane has a rather simpler form in terms of homogenous transformation matrices.</figcaption> </p>
</p>
</figure>

We also define another **Special Euclidean Group** SE(2) for **planar motions**. The Special Euclidean Group SE(2) is defined as the set of all $`3 \times 3`$ real matrices T of the form:

$`T = \begin{pmatrix}
R_{2 \times 2} & p_{2 \times 1} \\
o_{1 \times 2} & 1 
\end{pmatrix}`$

Where $`R \in SO(2)`$, and $`p \in \mathbb{R}^2`$. A matrix $`T ∈ SE(2`$) always has the following form:

$`T = \begin{pmatrix}
r_{11} & r_{12} & p_1 \\
r_{21} & r_{22} & p_2\\
0 & 0 & 1
\end{pmatrix} = \begin{pmatrix}
cos\theta & -sin\theta & p_1 \\
sin\theta & cos\theta & p_2\\
0 & 0 & 1
\end{pmatrix}`$

where $`\theta \in [0,2\pi)`$. We saw how to derive the orientation part in the rotation matrices lesson and the position part is just mapping the origin of the body frame to the base frame which is $`p_1`$ units in the base frame's x-axis direction and $`p_2`$ units in the base frame's y-axis direction. Let’s see an example for the 2D case.


### Example: Pose in 2D Plane

Suppose that frame {a} is instantaneously attached to a small mobile robot and it is initially coincident with the space frame {s}. The robot starts to move and first rotates $`90^{o}`$ and then travels forward for five units to reach configuration {b}, it then rotates $`-90^{o}`$, and moves forward for five units, and reaches the configuration {c} and finally, it rotates $`-90^{o}`$ and moves forward for three units to reach the configuration {d}:

![homogenous-transformations-2d-case-mobile-robot](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/ce28f16d-2566-460b-b084-c7b7dab056e0)

The homogenous transformation matrices representing the configuration of the robot at each stage can be calculated as the following:

$`T_{ab} = \begin{pmatrix}
0 & -1 & 0 \\
1 & 0 & 5\\
0 & 0 & 1
\end{pmatrix}, \, T_{bc} = \begin{pmatrix}
0 & 1 & 0 \\
-1 & 0 & -5\\
0 & 0 & 1
\end{pmatrix}, \,
T_{cd} = \begin{pmatrix}
0 & 1 & 0 \\
-1 & 0 & -3\\
0 & 0 & 1
\end{pmatrix}, \, T_{ad} = \begin{pmatrix}
0 & 1 & 5 \\
-1 & 0 & 2\\
0 & 0 & 1
\end{pmatrix}`$

Now let’s discuss the properties of transformation matrices.

## Properties of Homogeneous Transformation Matrices to Express Pose in Robotics

As we also studied with [the rotation matrices](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-3:-Orientation-in-Robotics-(Rotation-Matrices)), **homogenous transformation matrices** have a bunch of properties that are unique to them:

- Identity matrix I is a trivial form of a transformation matrix and it means that the pose of the body frame {b} is the same as the space frame {s}.

- The Special Euclidean Group SE(3) is a group because:

  1. The inverse of a transformation matrix $`T \in SE(3)`$ is also a transformation matrix and can be computed as:

     $`T^{-1} = {\begin{pmatrix}
     R & p\\
     o & 1
     \end{pmatrix}}^{-1} = \begin{pmatrix}
     R^T & -R^{T}p\\
     o & 1
     \end{pmatrix} \in SE(3)`$

     Prove this using the fact that the multiplication of a matrix with its inverse is the identity matrix and use a simple matrix 
     multiplication to find the elements of the inverse of the transformation matrix.

  2. The product of two transformation matrices is also a transformation matrix: $`T_1 T_2 \in SE(3)`$.
  
  3. The multiplication of transformation matrices is associative: $`(T_{1} T_{2})T_{3} = T_{1}(T_{2} T_{3})`$. But not generally commutative: $`T_{1} T_{2} \neq T_{2} T_{1}`$. 

- If we have $`T = (R,p) \in SE(3))`$ and $`x,y \in \mathbb{R}^3`$ then:
  
  1. The transformation matrix T preserves distances between the points in $`\mathbb{R}^3`$, which means that the distance between these points after the transformation is the same as the original distance: $`||Tx - Ty|| = ||x - y||`$. In which, ||.|| is the standard Euclidean norm in $`\mathbb{R}^3`$ and is defined as: $`||x|| = \sqrt{x^T x}`$. 

  2. The transformation matrix T preserves angles: $`(Tx - Tz).(Ty - Tz) = (x-z).(y-z) \, \text{for all } z \in \mathbb{R}^3`$. In which $`.`$ is the standard Euclidean inner (dot) product in $`\mathbb{R}^3`$ that can be defined as: $`x.y = x^T y`$. 

**Note 1.** $`T \in SE(3)`$ as the **transformation on points** in $`\mathbb{R}^3`$, transforms the point x to Tx and this means that the point x is rotated by R and translated by p as Rx + p; therefore Tx means the representation of x in homogenous coordinates as: 

$`T \begin{pmatrix}
x\\
1
\end{pmatrix} = \begin{pmatrix}
R & p\\
o & 1
\end{pmatrix} \begin{pmatrix}
x\\
1
\end{pmatrix} = \begin{pmatrix}
Rx + p\\
1
\end{pmatrix}`$

**Note 2**. If {x,y,z} are points on the rigid body, then {Tx,Ty,Tz} are displaced versions of the points on the rigid body.

**Note 3.** If $`x,y,z \in \mathbb{R}^3`$ are three vertices of a triangle, then the triangle formed by the transformed vertices {Tx,Ty,Tz} has the same set of lengths and angles as those of the triangle {x,y,z}. These two triangles are called isometric.


## Uses of Homogeneous Transformation Matrices

Similar to what we studied for the rotation matrix, the homogenous transformation matrix $`T \in SE(3)`$ can have three different applications.

1. It can be used to express the **configuration (position and orientation)** of a frame relative to a fixed frame. If we consider the figure of the robot with body frame {b} and the space frame {s} that we saw at the beginning of this lesson, then the configuration of the body frame relative to the space frame can be defined as: $`T_{sb} = \begin{pmatrix}
R_{sb} & p_{sb}\\
o & 1
\end{pmatrix}`$. Where $`R_{sb}`$ is the **rotation matrix** representing the **orientation** of the frame {b} relative to the frame {s} and by now you definitely feel conformable to easily calculate it, and p is the **position** of the body frame {b} origin in the space frame’s coordinates. Let's see an example of this application. 

**Example.** As an illustration, suppose that three coordinate frames {a}, {b}, and {c} are positioned in space as the following figure:

![homogenous-transformation-matrices-three-coordinate-frames](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8816aef5-6c23-409a-8f90-6206cb4daf41)
<figcaption> <p align="center">This figure shows the position and orientation of the frames {a}, {b}, and {c} in space.</figcaption> </p>

{a} is initially coincident with the space frame {s}. Thus, the pose of the frames {a}, {b}, and {c} relative to {s} can be calculated as:

$`T_{sa} = (R_{sa}, p_{sa}) = \begin{pmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}`$

$`T_{sb} = (R_{sb}, p_{sb}) = \begin{pmatrix}
0 & 0 & 1 & 0\\
0 & -1 & 0 & -2\\
1 & 0 & 0 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}`$

$`T_{sc} = (R_{sc}, p_{sc}) = \begin{pmatrix}
-1 & 0 & 0 & -1\\
0 & 0 & 1 & 1\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}`$. 

2. It can act as an **operator** and change the representation of a **frame or vector** from one coordinate frame to another coordinate frame.

Another **application** for the **homogenous transformation matrix** is that it can act as an **operator** and **change the reference frame** of **a vector or a frame**. For any three reference frames {a}, {b}, and {c} and any free vector v expressed in for example {b} frame as $`v_b`$ then using the **subscript cancellation rule** that we learned before, we can write:

$`T_{ab} T_{bc} = T_{ac}, \, T_{ab} \begin{pmatrix}
v_b \\
1
\end{pmatrix} = \begin{pmatrix}
v_a \\
1
\end{pmatrix}`$

Here, the transformation matrix T acts as an operator and changes the reference frame of the vector or a frame. In the second equation, $`T_{ab}`$ acts on the $`v_b`$ and changes its reference frame from {b} to {a}. We **append** 1 to the end of each vector to get its **homogenous coordinate representation** (the **3-vector** is changed into a **4-vector**) and avoid **dimension mismatch** when applying $`T_{ab}`$. 

As an instance, in the example that we saw above, we can find the homogenous transformation matrix representing the pose of any frame relative to another frame. For example, in order to calculate the homogenous transformation matrix $`T_{bc}`$, which represents the position and orientation of the frame {c} relative to the frame {b}, we can write: $`T_{bc} = T_{bs} T_{sc} = {T^{-1}_{sb}} T_{sc}`$. 

Note that for any two frames {d} and {e}, $`T_{de} = T^{-1}_{ed}`$. Using the expression for the inverse of the homogenous transformation matrix, we can calculate: 

$`{T^{-1}_{sb}} = \begin{pmatrix}
{R^T_{sb}} & -{R^T_{sb}}p_{sb} \\
o & 1
\end{pmatrix} = \begin{pmatrix}
0 & 0 & 1 & 0 \\
0 & -1 & 0 & -2 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}`$

And thus:

$`T_{bc} = \begin{pmatrix}
0 & 0 & 1 & 0 \\
0 & -1 & 0 & -2 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix} \begin{pmatrix}
-1 & 0 & 0 & -1 \\
0 & 0 & 1 & 1 \\
0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix} = \begin{pmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & -1 & -3 \\
-1 & 0 & 0 & -1 \\
0 & 0 & 0 & 1
\end{pmatrix}`$.

Let’s now see an example that uses these two applications to find the relative position and orientation of different coordinate frames.

**Example.** Suppose an arm-mounted mobile robot [X-Terrabot ](https://robots.ros.org/x-terrabot/)is moving in a room and wants to pick up an object with body frame {e} using its end-effector with the attached frame {c}:

![homogenous-transformations-arm-mounted-mobile-robot-camera-object](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/62218710-0bdd-4f25-8392-05ad8e2ee10f)

A camera is fixed to the ceiling, and based on its measurements, the configuration of the frames attached to the wheeled platform {b} and the object frame {e} relative to the camera’s frame {d} are known:

$`T_{db} = \begin{pmatrix}
0 & 0 & -1 & 250 \\
0 & -1 & 0 & -150 \\
-1 & 0 & 0 & 200 \\
0 & 0 & 0 & 1
\end{pmatrix}, \, T_{de} = \begin{pmatrix}
0 & 0 & -1 & 300 \\
0 & -1 & 0 & 100 \\
-1 & 0 & 0 & 120 \\
0 & 0 & 0 & 1
\end{pmatrix}`$

Also, using the arm’s joint angle measurements, $`T_{bc}`$ is also known:

$`T_{bc} = \begin{pmatrix}
0 & -\frac{1}{\sqrt{2}} & -\frac{1}{\sqrt{2}} & 30 \\
0 & \frac{1}{\sqrt{2}} & -\frac{1}{\sqrt{2}} & -40 \\
1 & 0 & 0 & 25 \\
0 & 0 & 0 & 1
\end{pmatrix}`$

The configuration of the camera frame {d} relative to the fixed-frame {a} is known in advance:

$`T_{ad} = \begin{pmatrix}
0 & 0 & -1 & 400 \\
0 & -1 & 0 & 50 \\
-1 & 0 & 0 & 300 \\
0 & 0 & 0 & 1
\end{pmatrix}`$

In order to calculate how to move the robot arm so as to pick up the object, the configuration of the object relative to the robot hand, $`T_{ce}`$, should be determined. Using the things that we’ve learned thus far, we can write:

$`\begin{cases}
    T_{cb} T_{bd} T_{da} T_{ae} = T_{ce} \\
    T_{ae} = T_{ad} T_{de}\\
    \rightarrow {T^{-1}_{bc}} {T^{-1}_{db}} \underbrace{{{T^{-1}_{ad}} T_{ad}}}_{=I} T_{de} = T_{ce}
\end{cases}`$

Therefore, the configuration of the object relative to the robot’s end-effector can be calculated as:

$`T_{ce} = \begin{pmatrix}
0 & 0 & 1 & -75 \\
-0.7071 & 0.7071 & 0 & -183.85 \\
-0.7071 & -0.7071 & 0 & 113.14 \\
0 & 0 & 0 & 1
\end{pmatrix}`$

Now, let’s see another example. In this example, we go back to an example from [the lesson about rotation matrices](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-3:-Orientation-in-Robotics-(Rotation-Matrices)), and this time we are not solely interested in the orientation, but we want to calculate configurations.

**Example.** Suppose that a camera and a gripper are attached to the end-effector of the industrial arm. The camera is used to observe the workpiece and position the end-effector in the right position, and the gripper is used to grip the workpiece. The overall system can be depicted in the figure below:

![homogenous-transformation-matrices-example-configuration](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/638f0d5c-2f9c-48cd-9d90-2e6f780f6108)
<figcaption> <p align="center">A camera is attached to the robot end-effector to observe the object and position the end-effector in the right position. Four reference frames are attached to different elements in the robot’s workspace, as shown in the figure. The configuration of one frame relative to the other can be represented by a homogenous transformation matrix.</figcaption> </p>

Four frames are attached to different elements in the robot’s workspace, as shown above. {a} is the frame coincident with the space frame {s}, {b} is the gripper frame, {c} is the camera frame, and {d} is the workpiece frame.

First, we want to calculate the configuration of the workpiece frame {d} relative to the frame {a} and the camera frame {c}. To this end, we just need to calculate the homogenous transformation matrices, $`T_{ad}`$ and $`T_{cd}`$. In the lesson about the rotation matrices, we calculated the orientation part of the transformation matrix. Now, we just build up to the configuration matrix with the position data as:

$`T_{ad} = \begin{pmatrix}
1 & 0 & 0 & -1 \\
0 & 1 & 0 & 1 \\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}, \, T_{cd} = \begin{pmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & -1 & 2\\
0 & 0 & 0 & 1
\end{pmatrix}`$

Now, suppose that the configuration of the camera frame {c} relative to the end-effector frame {b} is given as:

$`T_{bc} = \begin{pmatrix}
1 & 0 & 0 & 4 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}`$

And we’d like to calculate the configuration of the end-effector frame {b} relative to the frame {a}. To this end, we use the **subscript cancellation rule**, and we can write:

$`\begin{split}
T_{ab} &= T_{ad}T_{db} \\
&= T_{ad}T_{dc}T_{cb} \\
&= T_{ad}{T^{-1}_{cd}}{T^{-1}_{bc}} \\
&= \begin{pmatrix}
0 & 1 & 0 & -1 \\
1 & 0 & 0 & -3 \\
0 & 0 & -1 & 2 \\
0 & 0 & 0 & 1
\end{pmatrix}
\end{split}`$.

3. It can act as an **operator** and can be used to **translate** and **rotate** (**displace**) a frame or a vector.

The homogenous transformation matrix can act on a vector or a frame and displace (rotate and translate) it. Then $`T = (R,p)`$ can be expressed as $`T = (Rot(\hat{\omega},\theta),p)`$ and can act on a vector or a frame and rotate it by $`\theta`$ about an axis $`\hat{\omega}`$ and translate it by p.

By abuse of notation, we can define the $`3 \times 3`$ **rotation operator** $`R = Rot(\hat{\omega},\theta)`$ as a $`4 \times 4`$ transformation matrix that rotates but not translates as:

$`Rot(\hat{\omega},\theta) = \begin{pmatrix}
R = e^{[\hat{\omega}]\theta} & o\\
o & 1\\
\end{pmatrix}_{4 \times 4}`$

And the $`4 \times 4`$ transformation matrix that can act as the **translation operator** and can only **translate** but cannot **rotate** can be defined as:

$`Trans(p) = Tran(\hat{p}, ||p||) = \begin{pmatrix}
1 & 0 & 0 & p_x \\
0 & 1 & 0 & p_y \\
0 & 0 & 1 & p_z \\
0 & 0 & 0 & 1
\end{pmatrix}`$

Where $`p = \hat{p} ||p||`$. 

This **translation operator** causes a **translation** along the unit direction $`\hat{p}`$ by a distance $`||p||`$.

Now let’s see the effect of **pre-multiplying** or **post-multiplying** a **transformation matrix** by the **operator** $`T = (R,p)`$. Suppose that $`T_{sb}`$ is the configuration of the body frame {b} relative to the space frame {s} and $`T = (Rot(\hat{\omega},\theta),Trans(p))`$ is the rotation and translation operator. Then we will have two cases:

- **Fixed-frame transformation:** If we **pre-multiply** $`T_{sb}`$ by T, then $`\hat{\omega}`$, and p are interpreted in the space frame {s}:

$`T_{sb'} = TT_{sb} = \text{Trans}(p) \text{Rot}(\hat{\omega}, \theta) T_{sb} = \begin{pmatrix} R & p \\ 0 & 1 \end{pmatrix} \begin{pmatrix} R_{sb} & p_{sb} \\ 0 & 1 \end{pmatrix} = \begin{pmatrix} R R_{sb} & R p_{sb} + p \\ 0 & 1 \end{pmatrix}`$.

In this case, the operator $`T = (\text{Rot}(\hat{\omega},\theta), \text{Trans}(p))`$ first rotates the body frame {b} by $`\theta`$ about an axis $`\hat{\omega}`$ in the space frame {s} and then translates it by p in the space frame {s} to get {b’}. If the origin of the body frame {b} is not coincident with the origin of the space frame {s}, then the rotation moves the origin of the body frame {b}.

- **Body-frame transformation:** if we **post-multiply** $`T_{sb}`$ by the operator T, then the rotation axis $`\hat{\omega}`$ and the position vector p are both interpreted in the body frame {b}:

$`T_{sb"} = T_{sb}T = T_{sb} \text{Trans}(p) \text{Rot}(\hat{\omega}, \theta)  = \begin{pmatrix} R_{sb} & p_{sb} \\ 0 & 1 \end{pmatrix} \begin{pmatrix} R & p \\ 0 & 1 \end{pmatrix} = \begin{pmatrix} R_{sb}R &  R_{sb}p + p_{sb} \\ 0 & 1 \end{pmatrix}`$.

In this case, the operator $`T = (\text{Rot}(\hat{\omega},\theta), \text{Trans}(p))`$ first translates the body frame {b} by p considered to be in the body frame {b}, then rotates about $`\hat{\omega}`$ in the new body frame by $`\theta`$ to get the new frame {b”}. This rotation does not move the origin of the frame.

Let’s see all these with examples.

**Example:** Suppose that the configuration of the body frame {b} relative to the space frame {s} is as the following figure:

<figure>
<p align="center">
<img width="340" alt="homogenous-transformations-displacement-operator-example" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/203bd6e4-ec5c-4f41-a8d8-8502546de6fb">
</p>
</figure>

Where the configuration of the body frame relative to the space frame, as we saw earlier in the lesson, can be represented by a 4×4 transformation matrix as:

$`T_{sb} = \begin{pmatrix} R_{sb} & p_{sb} \\ o & 1 \end{pmatrix} = \begin{pmatrix} 0 &  0 & 1 & 0 \\ 0 & -1 & 0 & -2 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}`$. 

Now suppose that displacement (rotation and translation) operator $`T = (\text{Rot}(\hat{\omega},\theta), \text{Trans}(p))`$ corresponds to a rotation about the unit axis $`\hat{\omega} = (0,0,1)`$ by $`\theta = 90^{o}`$ and a translation along the vector $`p = (0,2,0)`$. Let’s find the final pose (position and orientation) of the body frame relative to the space frame after going through a transformation of pre-multiplying or post-multiplying of the transformation matrix representing the configuration of the body frame relative to the space frame by the displacement operator defined.

- **Case (1):** $`T_{sb}`$ is pre-multiplied by T, then the rotation axis $`\hat{\omega}`$ is interpreted in the space frame {s} and is equal to the axis $`\hat{z}_s`$. p is also interpreted in the {s} frame and represents a translation of two units along the $`\hat{y}_s`$ axis. Then the final pose of the body frame relative to the space frame can be visualized as:

<figure>
<p align="center">
<img width="514" alt="homogenous-transformation-fixed-frame-operator-without-logo" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/fd3d15a4-8876-4442-8942-efb07ceec69c">
<figcaption> <p align="center">The final pose of the body frame after going through a rotation about the space frame’s z-axis by 90 deg, and then translating along the space frame’s y-axis by 2 units.
</figcaption> </p>
</p>
</figure>

The frame {b} first goes through a rotation by 90 deg about the z-axis of the {s} frame, and because the origins of the frames {b} and {s} are not initially coincident, this rotation displaces the origin of the {b} frame. Then it goes through a translation by two units along the y-axis of the space frame {s} to reach the frame {b’}. The simulation below shows a demonstration of this transformation:

![simulation-1-fixed-frame-transformation](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/fe9f9165-70b9-4db5-9e4a-36fc96e2dbc9)

Mathematically, we can say that the transformation matrix representing the pose of the frame {b’} relative to the space frame {s} can be calculated as:

$`\begin{split}
T_{sb'} &= T T_{sb} = Trans(p)Rot(\hat{\omega}, \theta)T_{sb} \\
&= \begin{pmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}
\begin{pmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}
\begin{pmatrix}
0 & 0 & 1 & 0 \\
0 & -1 & 0 & -2 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix} \\
&= \begin{pmatrix}
0 & 1 & 0 & 2 \\
0 & 0 & 1 & 2 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}
\end{split}`$

where $`Rot(\hat{\omega},\theta)`$ can be calculated using [the Rodrigues’ formula that we learned in the lesson about the exponential coordinate representation of orientation](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-4:-Orientation-in-Robotics-(Exponential-Coordinates,-and-Euler-Angles)) as $`Rot(\hat{\omega},\theta) = I + sin\theta[\hat{\omega}] + (1 - cos\theta) [\hat{\omega}]^2`$. And then, we wrote it as a 4×4 rotation operator. The above transformation matrix can also be verified from the figure.

- **Case (2):** $`T_{sb}`$ is **post-multiplied** by T (**body-frame transformation**), then the rotation axis $`\hat{\omega}`$ and the position vector p are both interpreted in the **body frame** {b}. In this case, first, a translation along the {b} frame’s y-axis, $`\hat{y}_b`$, is done by two units, and then a rotation about the new body frame’s z-axis is done by 90 deg. This rotation **does not change** the origin of the new body frame. The final pose of the body frame relative to the space frame can be visualized as:

<figure>
<p align="center">
<img width="514" alt="homogenous-transformation-matrices-body-frame-transformation" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/12e81b0a-8731-401e-a37e-03be3fbe5299">
<figcaption> <p align="center">The final pose of the body frame after going through a translation about the {b} frame's y-axis followed by a rotation about the new body frame’s z-axis by 90 deg.</figcaption> </p>
</p>
</figure>

The simulation below shows a demonstration of this transformation:

![body-frame-transformation-simulation](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/f5057f17-71d2-4d62-9185-41cbb577eba8)

Mathematically, we can also prove this. The transformation matrix representing the pose of the frame {b”} relative to the space frame {s} can be calculated as:

$`\begin{split}
T_{sb"} &= T_{sb}T = T_{sb} Trans(p)Rot(\hat{\omega}, \theta) \\
&= \begin{pmatrix}
0 & 0 & 1 & 0 \\
0 & -1 & 0 & -2 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}
\begin{pmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}
\begin{pmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix} \\
&= \begin{pmatrix}
0 & 0 & 1 & 0 \\
-1 & 0 & 0 & -4 \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{pmatrix}
\end{split}`$

That verifies the result that we got from the visualization.

## References

- Kevin, M.L. and Frank, C.P., 2017. Modern robotics: mechanics, planning, and control

- Murray, R.M., Li, Z. and Sastry, S.S., 2017. A mathematical introduction to robotic manipulation. CRC press.

- Corke, P., 2023. Robotics, Vision and Control: Fundamental Algorithms in Python (Vol. 146). Springer Nature.

- Cao, C.T., Do, V.P. and Lee, B.R., 2019. A novel indirect calibration approach for robot positioning error compensation based on neural network and hand-eye vision. Applied Sciences, 9(9), p.1940.