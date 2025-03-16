## Introduction

In robotics, degrees of freedom (DOFs) refers to the number of independent variables or parameters required to uniquely specify the configuration of a robot (answers the question where is the robot?). It essentially describes the ways in which a robot can move. Each degree of freedom represents a single independent motion that the robot can perform.

In this lesson, you will become familiar with:
- the concept of DOF for robots
- different joints in robots and how they put constraints on the motion of the links
- a general formula that you can find the DOFs of any mechanism and not just the robot arms

What does it mean when we say that DOFs are the number of independent variables or parameters required to uniquely specify the configuration of a robot? What is the configuration of something? 

## What is the Configuration of Something?

The configuration of something answers the question, where is that thing? For example, to know where a door is, we only need to know the angle of its hinge when it changes from 0 to some degrees.

<figure>
<p align="center">
  <img width="500" height="187.5*2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/cdfafd01-13f3-4681-8ea5-b40ead61d429">
<figcaption> <p align="center">The configuration of a door can be determined by the angle of its hinge.</figcaption> </p>
</p>
</figure>

As another example, to know the configuration of a point on a plane, we need to know the x and y coordinates of that point.

Just as knowing the angle of a door's hinge or the x and y coordinates of a point on a plane defines their positions, the configuration of a robot is the specific arrangement of its movable parts, which includes the positions and orientations of its joints and links. It defines the robot's exact location and orientation in space at a given moment. On the other hand, the degrees of freedom (DOF) of a robot represent the number of independent ways the robot can move. Each DOF corresponds to a distinct motion that the robot can perform. These motions can involve translations (linear movements) and rotations (angular movements) along specific axes. 

We will come back to the concept of configuration when we study position and orientation for robots but here it is essential to understand its connection to the concept of DOFs. The configuration of a robot defines its position and orientation using various joint variables, while the degrees of freedom specify the number of independent motions it can perform. These concepts are deeply intertwined, forming the foundation for understanding a robot's mobility, kinematics, and design considerations.

Now that we understand how the configuration and DOFs relate to each other, let's first discuss the DOFs of a rigid body in space. 

## DOFs of a Rigid Body in a 3D Space

A rigid body in three-dimensional space has six degrees of freedom (DOFs). Three of them are for the position: motion along the x, motion along the y, and motion along the z, and three are for orientation: rotation around x or roll, rotation around y or pitch, and rotation around z or yaw. See the video below for a visualization (excuse the quality of the video because it is recorded years ago):

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d5130e15-2481-4a4a-9dc6-73f39bd4c71a
<figcaption> <p align="center">A rigid body in a 3D space has three translational and three rotational DOFs.</figcaption> </p>

## Degrees of Freedom of a Rigid Body in a 2D Space

With the same analogy, we can say that the rigid body on a 2D plane has three degrees of freedom. Two linear DOFs and one rotational DOF (think about a rigid body in space that is confined to the plane so it cannot move in the z direction and we cannot rotate it about the x and y axes. therefor 6 DOFs of the rigid body - 3 constraints = 3 DOFs):

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/84fd885e-bc6b-4819-b0a1-fed89fcf1c80
<figcaption> <p align="center">A rigid body on a 2D plane has three DOFs: two for position and one for orientation.</figcaption> </p>

Now, let's talk about the DOFs of a robot. 

## Robot Joints Put Constraints on the Motion of the Robot Links Reducing Their Degrees of Freedom (DOFs)

We have seen that a rigid body in a 2D space has three DOFs, but why does a planar robot with two rigid bodies have only two DOFs?

<figure>
<p align="center">
  <img width="500" height="187.5*2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b0ac74c5-6bdc-4c98-beab-dcadbaae830b">
<figcaption> <p align="center">Each rigid body in this planar robot has one DOF and not three DOFs.
</figcaption> </p>
</p>
</figure>

The answer to this question lies in the constraints that its joints put on the links’ movement with respect to each other. This robot has two revolute or hinge joints. Each of the joints puts two constraints on the movement of the corresponding link. That is, its links can only rotate about the z-axis.

## Degrees of Freedom (DOFs) of a 3R Robot Arm

If we imagine a robot in 3D like a 3R robot in space with three revolute joints, the revolute joint (R) will put five constraints on the motion of one link with respect to the other link. So again, it will provide only one DOF:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/f8be064b-d4b5-4569-b11e-1aa6f3be3fc8
<figcaption> <p align="center">A revolute joint puts five constraints on the motion of the rigid bodies (links). Robot video reference: Educational Videos & Lectures Youtube Channel</figcaption></p>

So, we can conclude that constraints on the robot links come from joints. But how many different joints are used in robots?

## Types of Different Joints Used in Robots

### Revolute (Rotary) Joints Provide One degree-of-freedom (DOF) for the Robot Links

As we saw in the industrial robot of the above video, a revolute joint is like a door hinge. It provides one DOF of motion between two bodies that it connects. The rotation is about the joint axis, and the positive rotation can be determined using the right-hand rule (According to the RHR, if your thumb is in the direction of the joint axis, the positive rotation is the direction your other 4 fingers curl):

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b6c19278-25ee-45c9-8698-a3caf383aaf5
<figcaption> <p align="center">The revolute or rotary joint provides one DOF of motion between the two bodies it connects. Left video credit: Novanta Inc. </figcaption> </p>

### Linear (sliding) Joints Provide One DOF for the Robot Links

A linear, sliding, or prismatic joint (P) provides a linear motion between two links. It will again provide only one DOF between two links:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c3312f4c-b520-4003-a21e-9e7e6ee895e9
<figcaption> <p align="center">Linear (sliding) Joints Provide one DOF for the Robot Links. Robot video credit: Educational Videos & Lectures YouTube</p>

### Universal Joints (U) Provide Two Degrees of Freedom for the Links they Connect

Next is the universal (U) joint, which is two revolute joints with joint axes orthogonal to each other thus, it can provide two rotational DOFs around roll (x) and pitch (y) axes: 

<figure>
<p align="center">
  <img width="500" height="187.5*2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/013cbe6d-c8b3-43d5-b37c-28a709a38c84">
<figcaption> <p align="center">The Universal Joint Provides Two Degrees of Freedom for the Links it Connects.</figcaption></p>
</p>
</figure>

See a video of how this joint rotates at the link below:

https://youtu.be/Tjn5BAqSb2Q

### Spherical Joints (S) Provide Three Degrees of Freedom Between the Connecting Links

The spherical (S), ball-and-socket, or shoulder joint can provide three DOFs, which are two degrees of freedom of the U joint plus spinning about the joint axis:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/34bf6260-31e1-42a3-ae75-afde9e598a89
<figcaption> <p align="center">The Spherical Joint (S) provides three degrees of freedom between the rigid bodies it connects.</figcaption></p>

Is this joint familiar to you? Can you identify that in your own body?

### Cylindrical Joints (C) Provide Two Degrees of Freedom Between the Connecting Links

Next is a cylindrical (C) joint that can provide an independent translation and rotation about a single fixed joint axis; thus, it has two DOFs:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/51e30471-d125-4a90-8fee-ae7c57af7a65
<figcaption> <p align="center">A cylindrical joint provides two degrees of freedom between the links it connects. Video credit: Keyan Ghazi-Zahedi YouTube</figcaption></p>

### Helical Joints (H) Provide One Degree of Freedom (DOF) Between the Rigid Bodies They Connect

And the final joint is the helical (H), or screw joint that provides a simultaneous rotation and translation about a screw axis and can provide one DOF:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/74246331-f410-4920-bcf7-5e8d7a0d58cc
<figcaption> <p align="center">A helical joint provides one degree of freedom between the links it connects. Video credit: Keyan Ghazi-Zahedi Youtube</figcaption></p>

The difference between this joint and the cylindrical joint is that in the cylindrical joint, the rotation and translation are independent, thus providing us with two degrees of freedom (DOFs), but in the helical joint, this motion is simultaneous, so it only has one degree of freedom.

## Grübler’s Formula to Find the Degrees of Freedom (DOfs) of Any Mechanism Including the Robots

Grübler’s Formula is a general formula that can be used to find the DOFs of any mechanism and not just the robots. Grübler’s formula says that the number of DOFs is equal to the sum of the freedoms of the bodies minus the number of independent constraints put on the motion of those bodies:

$dof = \sum(\text{freedoms of bodies}) - \text{number of independent constraints}$

If we take N as the number of bodies or links (note that we **traditionally** also take the ground as one link):

$N = \text{number of bodies including ground}$

and if we take J as the number of joints:

$J = \text{number of Joints}$

and if m is the number of degrees of freedom of a single body, that is six for spatial bodies and three for planar bodies:

$`\left\{
\begin{array}{cc}
    m = 6 & \text{for spatial bodies} \\
    m = 3 & \text{for planar bodies} \\
\end{array}
\right.`$

Then we can write the DOFs are equal to rigid body freedoms minus joint constraints. We deduct 1 from N because we want to exclude the ground:

$$\text{dof} = m (N - 1) -  \sum_{i=1}^{J} c_i$$

We know that the degree of freedom of movement of one link w.r.t another can be found by deducting the number of constraints that the joint puts on the movement from the DOFs of a rigid body:

$$f_i = m - c_i$$

Rewriting the formula in terms of the DOFs of the joints we can get this formula:

$$\text{dof} = m (N - 1 - J) + \sum_{i=1}^{J} f_i$$

This is called Grübler’s formula which can be used to find the degrees of freedom of any mechanism. **Remember that all constraints are independent.** Also, not that each joint connects two links. For example for the planar robot below, the first joint connects the ground link to the first link of the robot.

Now let’s see some examples.

### Grübler’s Formula to Find the Degrees of Freedom (DOFs) of Planar Mechanisms

**Two-Degree-of-Freedom (2-dof) Planar Robot Arm**

We have seen that our planar two-link robot arm has two degrees of freedom (DOFs). Now let’s see if we can get the same answer with Grübler’s formula:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c22f1ecd-2192-40bd-9c09-befca1aae73c
<figcaption> <p align="center">Grübler’s formula also verifies that our two-link planar robot has two degrees of freedom (DOFs).
</figcaption></p>

**Four-Bar Linkage**

Now let’s step it up a notch and find the degrees of freedom of a four-bar linkage. It has four links because remember we take the ground as one of our links too:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/463c4f8d-d874-4a50-a261-5eb747749b02
<figcaption> <p align="center">According to Grübler’s formula, the four-bar linkage has one degree of freedom.</figcaption></p>

Keep in mind that for Grübler’s formula to work, the constraints must be independent.

The next examples are for spatial robots.

### Grübler’s Formula to Find the Degrees of Freedom (DOFs) of Spatial Mechanisms

**Stewart Platform**

The first mechanism is a Stewart mechanism, which is a parallel mechanism with six legs. Each leg has two spherical joints and one prismatic joint. We can also replace the bottom spherical joints with universal joints with two degrees of freedom (DOFs). Because it is a parallel robot, each leg supports a fraction of the weight of the payload.

Now let’s find the degrees of the robot using Grübler’s formula:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/29ae4e5f-0e20-467e-a965-a26bb73babad
<figcaption> <p align="center">According to Grübler’s formula, the Stewart Platform has twelve degrees of freedom.</figcaption></p>

Of these twelve degrees of freedom, only six degrees of freedom are shown in the top platform since the other six are torsional rotations about the leg axis and do not affect the mobile platform’s motion. Because the top platform can move with the full six degrees of freedom (DOFs) of a rigid body in space, the Stewart platform is usually used to simulate airplanes:

https://youtu.be/uxYMh2VpDSc

Have you seen it in other simulators or games for motion simulation? 

**Delta Robot**

Now let’s find the degrees of freedom (dofs) of a Delta robot.

Delta robot is a parallel robot that can maintain its end-effector orientation, unlike the Stewart platform that can change the orientation of its end-effector. It has three legs, and each leg has three Revolute joints, four spherical joints, and five links:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/942f0e6d-c385-4f0b-92bf-9876920fc888
<figcaption> <p align="center">According to Grübler’s formula, the Delta robot has fifteen degrees of freedom.</figcaption></p>

Of these fifteen degrees of freedom, twelve are related to the torsion of the twelve links because of being connected to the spherical joints. These degrees of freedom are called internal degrees of freedom (DOFs). Only three are visible at the end-effector on the moving platform. Delta robot acts as an x-y-z Cartesian positioning device:

https://youtu.be/yXNbG4P8fTU

## Conclusion

In conclusion, the concept of DOFs is fundamental in robotics as it defines the number of independent ways a robot can move. This concept is closely tied to the **configuration of a robot**, which represents the arrangement of its movable parts, including joint positions and orientations.

DOFs are determined by the types of **joints** in a robot's mechanism, such as revolute, linear, universal, spherical, cylindrical, and helical joints. These joints impose constraints on the motion of the robot's links, ultimately influencing the robot's overall DOFs. The **Grübler’s Formula** provides a general method to calculate the DOFs of any mechanism, not just robots, by considering the freedoms of bodies and the constraints imposed by joints.

Through various examples of planar and spatial robot arms such as four-bar linkages, Stewart platforms, and Delta robots, we've seen how Grübler’s Formula can be applied to determine the DOFs of complex mechanisms.

## References

- Modern Robotics: Mechanics, Planning, and Control by Frank Park and Kevin Lynch
- https://www.youtube.com/@Vid010
- https://www.youtube.com/@novantainc.3375
- https://www.youtube.com/@KeyanZahedi