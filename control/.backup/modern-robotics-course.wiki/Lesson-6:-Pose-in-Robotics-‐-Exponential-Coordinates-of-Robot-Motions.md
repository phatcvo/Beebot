In this lesson, we will see an introduction to screw motion in robotics, and we will also see how we can define exponential coordinates for robot motions. 

## Introduction to Screw Theory

Screw theory in robotics provides a foundation to express robot motions and states that **any robot configuration** can be achieved by starting from the home (fixed) reference frame and then simultaneous rotation about and translation along the screw axis:

<figure>
<p align="center">
<img width="514" alt="screw-motion-exp-coordinates" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/501991ff-39e5-4b9e-91da-fdc6b28dc873">
</p>
</figure>

We can say that all configurations can be achieved by a **screw motion** (spatial rigid body displacements) that comes from the **Chasles-Mozzi** theorem in kinematics, which states that every **displacement** (rotation and translation at the same time) of the rigid body can be obtained by a finite rotation about and translation along a fixed screw axis $`\mathcal{S}`$:

<figure>
<p align="center">
<img width="257" alt="exponential-coordinates-robot-motions-chasles-mozzi-theorem" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/420652d1-e812-4da9-a7ff-4c87b311440b">
<figcaption> <p align="center">Chassles-Mozzi theorem says that every rigid body motion can be described as a screw displacement (rotation and translation) along some screw axis 𝘚.</figcaption> </p>
</p>
</figure>

This motion is like the motion of a screw that simultaneously rotates about and translates along the same fixed axis:

<figure>
<p align="center">
<img width="385.5" alt="screw-motion" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/11bbb064-f8c9-43ce-b41d-8e86e4bd0ba2">
<figcaption> <p align="center">Any robot’s configuration can be achieved by starting from the home reference frame and then rotating about and translating along a screw axis. This act is like a screw motion.</figcaption> </p>
</p>
</figure>

The screw theory provides a geometric description of robot motions that makes the kinematic analysis very simple in comparison to other methods like **Denavit-Hartenberg**. We will study this in the forward kinemtatics lesson, but for now, just know that the pose of the end-effector relative to the base frame (depicted in the figure below) can simply be found by the product of the exponential formula and the screw theory: $`T(q) = e^{[\mathcal{S}_1]q_1} ... e^{[\mathcal{S}_{n-1}]q_{n-1}}e^{[\mathcal{S}_{n}]q_{n}}M`$.

<figure>
<p align="center">
<img width="578.25" alt="screw-motion-exp-coordinates-1" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/2c4011c4-07de-478e-80b6-40d50fe000c4">
<figcaption> <p align="center">The end-effector’s configuration with respect to the world frame can be easily calculated using the product of the exponentials formula and the screw theory.</figcaption> </p>
</p>
</figure>

$`\mathcal{S}_1 ... \mathcal{S}_n`$ are **screw axes** expressed in the fixed base frame when the robot is at its home position. $`q_1 ... q_n`$ are **joint variables**, and M is the end-effector configuration when the robot is at **zero position**. Unlike conventional methods like Denavit-Hartenberg, this method **does not require** link reference frames to be defined.

[Prof. Daniel Martins ](https://scholar.google.com.br/citations?user=AlzmDlIAAAAJ)from the Federal University of Santa Catarina made a great presentation on the importance of screw theory that puts all of this more articulately:

https://youtu.be/vJPlSfPkScU?list=PLlqdnFs9xNwpD9zJr8BgAbfHH3AyixTqt

As an example in my previous research, I used screw theory to describe the motion of continuum compliant robots like steerable needles. Steerable needles are compliant needles that can be steered to a target location by rotation of the needle about its axis and translation of the needle:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/04961735-a1b3-48a2-bdd7-318ddc97cdc7
<figcaption> <p align="center">Steerable needles are compliant needles that can be steered to a target location by rotating the needle about its axis and translating the needle. Screw theory can be used to describe the motion of continuum soft robots like steerable needles.</figcaption> </p>

As an example, consider fracture-directed steerable needles. Fracture-directed steerable needles are a type of steerable needles in which the direction of the tissue fracture is controlled by either the tip of the needle or the waterjet, and then the steerable needle follows the path:

https://youtu.be/g72HpFUgyKY

https://youtu.be/ldkkqJhiHrU

Screw theory can be used to describe the kinematics of these needles accurately:

<figure>
<p align="center">
<img width="410" alt="screw-theory-fracture-directed-steerable-needles" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7a068d02-1a06-42b0-a068-7a19208ccfc8">
<figcaption> <p align="center">Screw theory can be used to describe the kinematics of fracture-directed steerable needles accurately.</figcaption> </p>
</p>
</figure>

<figure>
<p align="center">
<img width="410" alt="screw-theory-waterjet-steerable-needles" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/5682c4d0-cb2a-4f25-b048-a806652d38bb"><figcaption> <p align="center">Screw theory can be used to describe the kinematics of waterjet steerable needles accurately.</figcaption> </p>
</p>
</figure>

Now let's see how we can define the screw axis mathematically. 

## Mathematical Interpretation of the Screw Axis

Screw axis can be mathematically defined as follows. First, we choose a reference frame and then define the screw axis 𝘚 as the 6-vector in that frame’s coordinates as:

$`\mathcal{S} = \begin{pmatrix}
\mathcal{S}_{\omega}\\
\mathcal{S}_v
\end{pmatrix}`$

$`\mathcal{S}_{\omega}`$ is the angular part of the screw axis (the one related to rotation), and $`\mathcal{S}_v`$ is the linear part of the screw axis (the one related to translation). Here, we can have two cases:

- Case one is when there is a **rotational component**. The angular component $`\mathcal{S}_{\omega}`$ of the screw axis is nonzero, and the screw axis can be found as: $`\mathcal{S}_{\omega} = \hat{\omega}`$ (the axis of rotation), and $`\mathcal{S}_v = a \times \mathcal{S}_{\omega} + h\mathcal{S}_{\omega}`$. a is a point (any point) on the screw axis, and h is called the pitch of the screw axis and it is the ratio of translation to the rotation and can be calculated as: $`h = \mathcal{S}^T_{\omega} \mathcal{S}_v`$ .

- Case two is when there is **no rotational motion**. In this case, the motion is a **purely linear motion** with no rotation. The angular component is zero, and the linear part is a unit vector that shows the direction of linear motion: $`\mathcal{S}_{\omega} = 0`$, and $`\mathcal{S}_v`$ will be a unit vector in the direction of linear motion. 

Note here that six numbers are needed to represent the screw axis $`\mathcal{S} = (\mathcal{S}_{\omega}, \mathcal{S}_v)`$, but the space of all screws is five-dimensional (5D), and this is because either $`\mathcal{S}_{\omega}`$ or $`\mathcal{S}_v`$ has a unit length.

Now let’s see how we can define the matrix representation of the screw axis 𝘚.

## Matrix Representation of the Screw Axis 𝘚

The matrix representation of the screw axis $`\mathcal{S} = (\mathcal{S}_{\omega}, \mathcal{S}_v)`$ can be defined as: 

$`[\mathcal{S}] = \begin{pmatrix}
[\mathcal{S}_{\omega}] & \mathcal{S}_v\\
o & 0
\end{pmatrix} \in se(3)`$

Here, the first bracket notation does not imply a skew-symmetric matrix, but it only wants to show the 4×4 matrix representation of the screw axis. The proof of this comes from the concept of twists in robotics that we will study in the future. 

Now let’s see how we can change the frame of reference in which a screw axis 𝘚 is defined.

## Change of Frame of Reference of a Screw Axis 𝘚

We can use the **adjoint transformation** to change the frame of reference of the screw axis: $`\mathcal{S}_a = [Ad_{T_{ab}}] \mathcal{S}_b, \, \mathcal{S}_b = [Ad_{T_{ba}}] \mathcal{S}_a`$. 

$`\mathcal{S}_a`$ is the screw axis representation in the frame {a}, and $`\mathcal{S}_b`$ is the screw axis representation in the frame {b}. The Adjoint representation of a transformation matrix $`T_{ab}`$, and $`T_{ba}`$ are defined as:

$`[Ad_{T_{ab}}] = \begin{pmatrix}
R & o\\
[p]R & R
\end{pmatrix}`$

and 

$`[Ad_{T_{ba}}] = \begin{pmatrix}
R^T & o\\
-R^T[p] & R^T
\end{pmatrix}`$

We will get back to this matrix when we study twists in robotics in future lessons. Now, based on our knowledge about the screw axis, let’s talk about the exponential coordinates of robot motions.

## Exponential Coordinates of Robot Motions

By now, we are familiar with [the exponential coordinate representation for orientations](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-4:-Orientation-in-Robotics-(Exponential-Coordinates,-and-Euler-Angles)), and we know that it is a three-parameter representation of orientation and parameterizes the orientation using a unit axis of rotation $`\hat{\omega}`$ and the angle of rotation $`\theta`$ about that axis:

<figure>
<p align="center">
<img width="257" alt="exponential-coordinates-axis-angles-SO3" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c474e6c5-dcf9-401d-b80e-679dca2e3520">
<figcaption> <p align="center">Exponential coordinate representation of orientation parameterizes the orientation using a unit axis of rotation and the angle of rotation about that axis.</figcaption> </p>
</p>
</figure>

$`\hat{\omega} \theta \in \mathbb{R}^3`$ is then the exponential coordinate representation of the orientation. We also learned about homogenous transformation matrices that are great tools for expressing rotations and translations in a compact 4×4 matrix:

$`T = \begin{pmatrix}
R & p\\
o & 1
\end{pmatrix}`$

Another possible representation of displacement (rotation and translation) is a **6-parameter** representation called exponential coordinates of motion. The 6D exponential coordinates of a homogenous transformation T can be defined as: $`\mathcal{S} q \in \mathbb{R}^6`$. Where 𝘚 is the screw axis and q is the distance that must be traveled along the screw axis to take the frame from the initial configuration I to T.

In the exponential coordinate representation, $`\mathcal{S} q \in \mathbb{R}^6`$:

- if we have rotational motion, then $`q = \theta`$ is the angle of rotation about the screw axis

- if the motion is pure translation with no rotational motion, then q is the linear distance traveled along the screw axis.

As we saw in the lesson about the exponential coordinates of rotation, the matrix exponential $`e^{[\hat{\omega}]\theta}`$ is equal to the rotation matrix that can act on a vector or a frame and can rotate it from the initial orientation to the final orientation. Similarly, the matrix representation of the screw axis can be used in the matrix exponential $`e^{[\mathcal{S}]q}`$ for robot motions. Thus, the matrix exponential for robot motions can map the elements of the Lie algebra se(3) to the elements of the Lie group SE(3): $`exp: [\mathcal{S}] q \in se(3) \rightarrow T \in SE(3)`$. 

And this means that exponentiation takes the initial configuration of the frame to the final configuration of the frame by following along and about a screw axis 𝘚 by q.

<figure>
<p align="center">
<img width="514" alt="screw-therory-final-config" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/f450c64f-be27-49e2-b2ee-11486a32d1bb">
</p>
</figure>

As we found a **closed-form** solution for the matrix exponential $`e^{[\hat{\omega}]\theta}`$ for orientations before using the Rodrigues formula, let’s examine if we can do the same for the matrix exponential $`e^{[\mathcal{S}]q}`$ for robot motions. Let the screw axis be $`\mathcal{S} = (\mathcal{S}_{\omega}, \mathcal{S}_v)`$ then we will have two cases:

- If we have **rotational motion**, then for any distance $`\theta \in \mathbb{R}`$ traveled about the axis, the matrix exponential for rigid body motions can be written as:

$`e^{[\mathcal{S}]\theta} = \begin{pmatrix}
e^{[\mathcal{S}_{\omega}]\theta} & (I\theta + (1-cos\theta)[\mathcal{S}_\omega] + (\theta - sin\theta)[\mathcal{S}_{\omega}]^2)\mathcal{S}_v\\
o & 1
\end{pmatrix}`$

- And if the **rotational part is zero** and the screw axis is **pure translation** with no rotation, then:

$`e^{[\mathcal{S}]d} = \begin{pmatrix}
I & \mathcal{S}_v d\\
o & 1
\end{pmatrix}`$

Here, d is the **linear distance traveled**. The **matrix I** in the upper left of the matrix shows that the **orientation does not change**, and the motion is **pure translation**. 

The proof is similar to the approach to the exponential coordinates of rotation for the matrix exponential and is left for your own practice.

**Inverse problem.** And the **matrix logarithm** is the **invert** of the **matrix exponential** and finds the matrix representation of the exponential coordinates 𝘚q:

$`\text{log: }T \in SE(3) \rightarrow [\mathcal{S}]q \in se(3)`$. 

And this means that if we have a **given configuration**, we want to find the **screw axis** and q such that if followed about and along this screw axis by that amount gives the same configuration. The unit screw axis for full spatial motions is similar to the unit rotation axis for pure rotations that we saw in the orientation lesson.

The inverse problem says that given an **arbitrary configuration** $`(R,p) \in SE(3)`$, we can always find a screw axis $`\mathcal{S} = (\mathcal{S}_{\omega},\mathcal{S}_v)`$ and a scalar q such that:

$`e^{[\mathcal{S}]q} = \begin{pmatrix}
R & p\\
o & 1
\end{pmatrix}`$

And as we saw before, the matrix:

$`[\mathcal{S}]q = \begin{pmatrix}
[\mathcal{S}_{\omega}]q & \mathcal{S}_v q\\
o & 0
\end{pmatrix} \in se(3)`$

is called the matrix logarithm of T = (R,p).

To solve the **inverse problem**, we follow the following **algorithm**:

Given (R,p) written as $`T \in SE(3)`$, find a q and a screw axis $`\mathcal{S} = (\mathcal{S}_{\omega},\mathcal{S}_v) \in \mathbb{R}^6`$ such that $`e^{[\mathcal{S}]q} = T`$. The vector $`\mathcal{S}q \in \mathbb{R}^6`$ comprises the **exponential coordinates** for T, and the matrix $`[\mathcal{S}]q \in se(3)`$ is the **matrix logarithm** of T.

- If R = I then set $`\mathcal{S}_{\omega} = 0`$ and $`\mathcal{S}_v`$ will be a unit vector in the direction of the translation.

- Otherwise, use the matrix logarithm on SO(3) that we learned in the exponential coordinates for rotations lesson to determine $`\mathcal{S}_\omega = \hat{\omega}`$ and θ for R. The $`\mathcal{S}_v`$ is calculated as:

$`\underbrace{(I\theta + (1-cos\theta)[\mathcal{S}_\omega] + (\theta - sin\theta)[\mathcal{S}_{\omega}]^2)}_{G(\theta)}\mathcal{S}_v = p \rightarrow \mathcal{S}_v =  G^{-1}(\theta) p = (\frac{1}{\theta}I - \frac{1}{2}[\mathcal{S}_{\omega}] + (\frac{1}{\theta} - \frac{1}{2}cot{\frac{\theta}{2}})[\mathcal{S}_{\omega}]^2)p`$.

Note that every **single-degree-of-freedom joint** (revolute joint, a prismatic joint, and a helical joint) of a robot that we talked about in the degrees of freedom lesson has a **joint axis defined by a screw axis**, and thus, we can conclude that the matrix exponential and the logarithm can be used to study the robot kinematics as we will see in the coming lessons. 

Now let’s see some examples that use all the knowledge that we have learned thus far to find solutions.

## Example: Homogenous Transformation Matrix to Exponential Coordinates of Motion

Suppose that the configuration of the body frame relative to the space frame is as the following figure:

<figure>
<p align="center">
<img width="451" alt="homogenous transformation to exp coordinates of motion" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/97874ca7-f68b-4270-9326-d4b6969a7bb0">
</p>
</figure>

In which the origin of the {b} frame is at (3,0,0) in terms of the space frame coordinates. The configuration of the {b} frame relative to the {s} frame, as we learned in the lesson about the homogenous transformation matrices, can be found using the transformation matrix $`T_{sb}`$ as:

$`T_{sb} = \begin{pmatrix}
0 & -1 & 0 & 3\\
0 & 0 & -1 & 0\\
1 & 0 & 0 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}`$

We want to find the **screw motion** (the screw axis 𝘚 and the amount of traveled distance q about the screw axis) that can generate the **same configuration**.

Since the orientation of the body frame is **not the same as** the orientation of the space frame, then we have a **rotational motion**. Using the approach we learned in the lesson about the exponential coordinates of orientation for the matrix logarithm of rotations, we can easily find the unit axis and the amount of rotation about this axis that can produce the given orientation as:

$`1 + 2c\theta = r_{11} + r_{22} + r_{33} \rightarrow 1 + 2c\theta = 0 \rightarrow \theta = 120^{o} = 2.094 \text{ rad or } \theta = 240^{o}`$

Here, I only solve for the $`\theta = 120^{o}`$ case and the second case is similar. 

$`[\hat{\omega}] = \frac{1}{2 sin\theta} (R - R^T) = \frac{1}{\sqrt{3}} \begin{pmatrix}
0 & -1 & -1\\
1 & 0 & -1\\
1 & 1 & 0
\end{pmatrix} \rightarrow \hat{\omega} = \frac{1}{\sqrt{3}}\begin{pmatrix}
1\\
-1\\
1
\end{pmatrix}`$

So, a rotation of 120 deg about the unit axis calculated above will create the same orientation. Now, using the second approach to calculate the screw axis, we can say that:

$`\mathcal{S}_{\omega} = \hat{\omega} = \frac{1}{\sqrt{3}}\begin{pmatrix}
1\\
-1\\
1
\end{pmatrix}`$

And

$`\mathcal{S}_v =  G^{-1}(\theta) p = (\frac{1}{\theta}I - \frac{1}{2}[\mathcal{S}_{\omega}] + (\frac{1}{\theta} - \frac{1}{2}cot{\frac{\theta}{2}})[\mathcal{S}_{\omega}]^2)p = \begin{pmatrix}
1.055\\
-1.055\\
-0.677
\end{pmatrix}`$.

And thus the screw axis can be found as:

$`\mathcal{S} = \begin{pmatrix}
\mathcal{S}_{\omega}\\
\mathcal{S}_v
\end{pmatrix} = \begin{pmatrix}
\frac{1}{\sqrt{3}}\\
-\frac{1}{\sqrt{3}}\\
\frac{1}{\sqrt{3}}\\
1.055\\
-1.055\\
-0.6769
\end{pmatrix}`$.

Therefore, a screw motion about the screw axis 𝘚 calculated above with the amount of θ calculated earlier produces the same configuration defined by the homogenous transformation matrix $`T_{sb}`$.

For more practice, let's draw this screw axis. For this we need a point on the screw axis and the direction of the screw axis. Since we have rotational motion here, then the direction of the screw axis is in the direction of the axis of rotation. To find a point on the screw axis, we should first calculate it's pitch and then use the equation $`\mathcal{S}_v = a \times \mathcal{S}_{\omega} + h\mathcal{S}_{\omega}`$ to find the point a (note that this point is not the only solution). 

$`h = \mathcal{S}^T_{\omega} \mathcal{S}_v = 0.8275`$

And solving the above equation, we can find one possible point as: $`a = (1,1,0)`$. Now we have the direction and the point on the screw axis. We can visualize it as:

<figure>
<p align="center">
<img width="451" alt="screw-axis-example-screw-interpretation-of-a-twist-in-robotics" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e9d9c9d7-bd8a-49ad-890a-f958c3ad6c25">
</p>
</figure>

Now let’s see another example.

## Example: Exponential Coordinates of Motion to Homogenous Transformation Matrix

In this example, we want to go backward and find the homogenous transformation matrix corresponding to the given exponential coordinates of the motion.

Suppose that the exponential coordinates of the motion are given by the following matrix:

$`\mathcal{S}\theta = \begin{pmatrix}
0\\
1\\
2\\
3\\
0\\
0
\end{pmatrix}`$

In order to find the homogenous transformation matrix representing the same configuration, we should find the matrix exponential corresponding to the exponential coordinates. Since the upper matrix part is not zero, we have rotational motion and thus the rotational part of the screw axis should be normalized. Thus we can write:

$`\mathcal{S}\theta = \sqrt{5} \begin{pmatrix}
0\\
\frac{1}{\sqrt{5}}\\
\frac{2}{\sqrt{5}}\\
\frac{3}{\sqrt{5}}\\
0\\
0
\end{pmatrix} \rightarrow \theta = 2.23 \text{ rad} = 128^{o}, \mathcal{S} = \begin{pmatrix}
0\\
\frac{1}{\sqrt{5}}\\
\frac{2}{\sqrt{5}}\\
\frac{3}{\sqrt{5}}\\
0\\
0
\end{pmatrix}`$

From the screw axis that we calculated, it will be easy to calculate the matrix exponential of the motion through the following process:

$`\mathcal{S}_\omega = \begin{pmatrix}
0\\
0.447\\
0.8944
\end{pmatrix} \rightarrow [\mathcal{S}_\omega] = \begin{pmatrix}
0 & -0.8944 & 0.447\\
0.8944 & 0 & 0\\
-0.447 & 0 & 0
\end{pmatrix}`$

And using the Rodrigues’ formula that we learned in the lesson about the exponential coordinates of orientation, we can find the rotational part of the transformation matrix as:

$`e^{[\mathcal{S}_{\omega}]\theta} = I + sin\theta [\mathcal{S}_{\omega}] + (1-cos\theta)[\mathcal{S}_{\omega}]^2 =  \begin{pmatrix}
-0.6121 & -0.7070 & 0.3533\\
0.7070 & -0.2899 & 0.6447\\
-0.3533 & 0.6447 & 0.6778
\end{pmatrix}`$

And the linear part can be calculated as:

$`G(\theta)\mathcal{S}_v = (I\theta + (1-cos\theta)[\mathcal{S}_{\omega}] + (\theta - sin\theta)[\mathcal{S}_{\omega}]^2)\mathcal{S}_v =  \begin{pmatrix}
0.7908 & -1.4422 & 0.7208\\
1.4422 & 1.0785 & 0.5755\\
-0.7208 & 0.5755 & 1.9424
\end{pmatrix}\begin{pmatrix}
1.345\\
0\\
0
\end{pmatrix} = \begin{pmatrix}
1.0637\\
1.9398\\
-0.9695
\end{pmatrix}`$

Therefore the homogenous transformation matrix representing the same configuration can be calculated as:

$`T_{sb} = e^{[\mathcal{S}]\theta} \underbrace{T_{sb}(0)}_{I} = \begin{pmatrix}
-0.6121 & -0.7070 & 0.3533 & 1.0637\\
0.7070 & -0.2899 & 0.6447 & 1.9398\\
-0.3533 & 0.6447 & 0.6778 & -0.9695\\
0 & 0 & 0 & 1
\end{pmatrix}`$. 

This homogenous transformation matrix represents the same configuration as the configuration of the frame after going through a screw motion about the defined screw axis. The initial configuration is the identity matrix since the {b} frame is initially coincident with the frame {s}.

Now let’s see how we can find the body frame’s final configuration after traveling a distance θ along the screw axis 𝘚 if the screw axis is defined in the space or the body frame.

## Body Frame’s Final Configuration After Travelling along the Screw Axis Defined in the Space or the Body Frame

Suppose that the space frame {s} and the body frame {b} are configured in space as the following figure:

<figure>
<p align="center">
<img width="451" alt="body-frame-space-frame-transformation-screw-interpretation-twist" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7a0e2963-1f90-498d-92c4-96ddcee78612">
</p>
</figure>

The configuration of the body frame relative to the space frame can be found using the matrix $`T_{sb}`$. We would like to know the body frame’s final configuration, $`T_{sb’}`$ if it travels a distance θ along the screw axis 𝘚. We would have two cases since 𝘚 can be represented in either {b} or {s} frame.

- If the screw axis 𝘚 is expressed in the {s} frame then the final configuration of the body frame can be calculated using the equation $`T_{sb’} =  e^{[\mathcal{S}_s]\theta}T_{sb}`$. In this case, the transformation matrix representation of the {b} frame relative to the {s} frame, $`T_{sb}`$, is pre-multiplied by the matrix exponential.

- If the screw axis 𝘚 is expressed in the {b} frame then the final configuration of the body frame can be calculated using the equation $`T_{sb"} = T_{sb} e^{[\mathcal{S}_b]\theta}`$. In this case, the transformation matrix representation of the {b} frame relative to the {s} frame, $`T_{sb}`$, is post-multiplied by the matrix exponential.

## References

- Kevin, M.L. and Frank, C.P., 2017. Modern robotics: mechanics, planning, and control

- Murray, R.M., Li, Z. and Sastry, S.S., 2017. A mathematical introduction to robotic manipulation. CRC press.

- Corke, P., 2023. Robotics, Vision and Control: Fundamental Algorithms in Python (Vol. 146). Springer Nature.

- Yang, F., Babaiasl, M. and Swensen, J.P., 2019. Fracture-directed steerable needles. Journal of Medical Robotics Research, 4(01), p.1842002.

- Babaiasl, M., Yang, F., Boccelli, S. and Swensen, J.P., 2020. Fracture-directed Waterjet Needle Steering: Design, Modeling, and Path Planning. In 2020 8th IEEE RAS/EMBS International Conference for Biomedical Robotics and Biomechatronics (BioRob) (pp. 1166-1173). IEEE.