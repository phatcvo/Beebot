Can you recall a time when you were frustrated when attempting to compute the forward kinematics for robotic systems? Did you find it frustrating to assign coordinate frames to each link when robots become more sophisticated? 

<figure>
<p align="center">
<img width="514" alt="denavit hartenberg - forward kinematics - frustration" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/9fac2d8f-5a54-468a-9b77-e5e11c18374e">
<figcaption> <p align="center">The frustration caused by assigning coordinate frames for each link while solving the forward kinematics of robotic systems using methods like Denavit-Hartenberg is REAL.</figcaption> </p>
</p>
</figure>

If your answer to these questions is yes, and you’re tired of cumbersome methods like Denavit-Hartenberg to calculate the forward kinematics of robotic chains, this lesson is for you. 

<figure>
<p align="center">
<img width="343" alt="forward kinematics - denavit hartenberg" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/cce1cf6e-2da3-4c29-83d2-44a1717281d9">
<figcaption> <p align="center">The Denavit-Hartenberg is one of the methods to derive the forward kinematics in Robotics, but it grows complicated as robotic systems become more complex. The photo is from Wikipedia.</figcaption> </p>
</p>
</figure>

Rather than going into mathematical details first, let’s get right to the point with an educational example.

## Example: Forward Kinematics of a 3 DOF Planar Open Chain Robot Arm

Consider a three dof planar open-chain robot arm modeled using STEM building blocks:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8cfa092e-627f-46c6-8ef1-66f8d535cb6d

Here is what it will look like once the coordinate frames, joints, and link lengths are identified:

<figure>
<p align="center">
<img width="514" alt="forward kinematics of 3 dof robot" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/488def6d-ca61-4b84-832d-12db32847afd">
<figcaption> <p align="center">three dof planar robot arm with coordinate frames, joints, and links identified.</figcaption> </p>
</p>
</figure>

$`L_1`$, $`L_2`$, and $`L_3`$ are link lengths. {0} is the fixed frame with the origin located at the base joint. {4} is the end-effector frame attached to the tip of the third link. We do not need all the frames to derive the forward kinematics using the **product of the exponentials formula**, but we depicted them for educational purposes. Since the motion is on the plane, the $`\hat{z}`$-axes are parallel and out of the page. But again, to reiterate, we don’t need all the coordinate frames.

In order to find the forward kinematics of a 3-dof planar open-chain robot, we will follow the following simple steps:

**Step 1:** Write the **vector of joint angles**.

Since all the joints, in this case, are revolute, the vector of joint angles can be written as:

$`\begin{pmatrix}
\theta_1\\
\theta_2\\
\theta_3
\end{pmatrix}`$

**Step 2:** Draw the robot in its **zero position** and assign the **base frame** and the **end-effector frame**. The 3-dof planar open-chain robot in its zero position can be depicted as the figure below:

<figure>
<p align="center">
<img width="514" alt="3 dof robot arm zero or zero position" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e3dda4c2-580b-4a16-ae52-ad3c836df7f6">
<figcaption> <p align="center">3 dof robot arm in zero position</figcaption> </p>
</p>
</figure>

**Step 3:** Calculate the matrix M, which is the **position and orientation** of frame {4} relative to frame {0} in the robot’s **zero position**. For our 3-dof planar open-chain robot, the matrix representing the position and orientation of the frame {4} relative to the frame {0} can be written using the knowledge that we gained in the transformation matrices lesson:

$`M = \begin{pmatrix}
1 & 0 & 0 & L_1 + L_2 + L_3\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
\end{pmatrix}`$

The rotational part is the identity matrix since the orientation of frame {4} is the same as the frame {0}, and the position of the origin of frame {4} is $`L_1 + L_2 + L_3`$ units in the direction of the x-axis of the {0} frame.

**Step 4.** Calculate the coordinates of 3D **arbitrary points** on the joint axes at zero configuration with respect to the coordinates of the fixed base frame. For our 3-dof planar open-chain robot, we go back to the figure of the robot in the zero position. We can take the three arbitrary points on the joint axes as the following points:

<figure>
<p align="center">
<img width="514" alt="forward kinematics-3 dof planar robot-arbitrary points" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b37a5c89-283a-4996-93c7-eda96d10313f">
<figcaption> <p align="center">Three arbitrary points on the joint axes of the 3-dof planar robot.</figcaption> </p>
</p>
</figure>

Then we can write the coordinates of these points with respect to the base frame as:

$`a_1 = \begin{pmatrix}
0\\
0\\
0
\end{pmatrix}, a_2 = \begin{pmatrix}
L_1\\
0\\
0
\end{pmatrix}, a_3 = \begin{pmatrix}
L_1 + L_2\\
0\\
0
\end{pmatrix}`$

The first point is on the origin of the base frame, and that’s why its coordinates are all zero. The second point is located at $`L_1`$ units from the x-axis of the base frame, and similarly, the third point is at $`L_1 + L_2`$ units from the x-axis of the base frame.

**Step 5.** Calculate the **screw axis of each joint** expressed in the base frame in the robot’s zero position. 

$`\mathcal{S} = \begin{pmatrix}
\mathcal{S}_{\omega_1}\\
\mathcal{S}_{v_1}\\
\end{pmatrix} \text{ where } \mathcal{S}_{\omega_1} = \begin{pmatrix}
0\\
0\\
1
\end{pmatrix}, \mathcal{S}_{v_1} = a_1 \times \mathcal{S}_{\omega_1} = (0,0,0) \rightarrow \mathcal{S}_1 = \begin{pmatrix}
0\\
0\\
1\\
0\\
0\\
0
\end{pmatrix}`$

Note that here the arbitrary point is at the center of the base frame and thus the linear part is zero.

Similarly:

$`\mathcal{S}_{\omega_2} = \begin{pmatrix}
0\\
0\\
1
\end{pmatrix}, \mathcal{S}_{v_2} = a_2 \times \mathcal{S}_{\omega_2} = \begin{pmatrix}
L_1\\
0\\
0
\end{pmatrix} \times \begin{pmatrix}
0\\
0\\
1
\end{pmatrix} = \begin{pmatrix}
0\\
-L_1\\
1
\end{pmatrix} \rightarrow \mathcal{S}_2 = \begin{pmatrix}
0\\
0\\
1\\
0\\
-L_1\\
0
\end{pmatrix}`$

$`\mathcal{S}_{\omega_3} = \begin{pmatrix}
0\\
0\\
1
\end{pmatrix}, \mathcal{S}_{v_3} = a_3 \times \mathcal{S}_{\omega_3} = \begin{pmatrix}
L_1+L_2\\
0\\
0
\end{pmatrix} \times \begin{pmatrix}
0\\
0\\
1
\end{pmatrix} = \begin{pmatrix}
0\\
-(L_1+L_2)\\
1
\end{pmatrix} \rightarrow \mathcal{S}_3 = \begin{pmatrix}
0\\
0\\
1\\
0\\
-(L_1+L_2)\\
0
\end{pmatrix}`$ 

Then, the **final configuration** of the end-effector in the base frame can be updated as the following transformation matrix:

$`T_{04} = e^{[\mathcal{S}_1]\theta_1}e^{[\mathcal{S}_2]\theta_2}e^{[\mathcal{S}_3]\theta_3} M`$ . 

At the end of this lesson, I will provide you with a MATLAB code that you can easily find the position and orientation of the end-effector frame in the base frame using the PoE formula. You will be then asked to write the Python version of this code in the coming lab. 

Using the code provided in this lesson, we can find the position and orientation of the end-effector frame in the base frame as the following equations:

$`\begin{cases}
x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3) \\
y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3) \\
\phi = \theta_1 + \theta_2 + \theta_3
\end{cases}`$

Note that x, y, and $`\phi`$ provide the position and orientation of the end-effector frame with respect to the base frame given the joint values, and we call it the **forward kinematics** of this robotic arm.

The steps taken to find the forward kinematics of the 3-dof planar robot are applicable to **all open-chain manipulators**. Now let’s learn about the general form of forward kinematics of serial manipulators using screw theory in robotics.

If you take a look at the literature on robot kinematics, you will figure out that most approaches are based on Denavit-Hartenberg (D-H) parameters presented first by guess what two people named Denavit and Hartenberg in an article in 1955.

<figure>
<p align="center">
<img width="729" alt="Denavit Hartenber article" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8c277e3b-fcb8-4378-a124-e5ac444ba5d4">
<figcaption> <p align="center">Article written by Denavit and Hartenberg to find the forward kinematics of serial manipulators. You can download the full paper here: https://asmedigitalcollection.asme.org/appliedmechanics/article-abstract/22/2/215/1110292/A-Kinematic-Notation-for-Lower-Pair-Mechanisms.</figcaption> </p>
</p>
</figure>

For reasons you’ll learn in this lesson, we prefer **Brockett’s 1983 Product of Exponentials (PoE) formula** based on screw theory to represent the forward kinematics of open-chain robot arms. You can refer to the references section for the article. 

<figure>
<p align="center">
<img width="822" alt="Forward kinematics using PoE formula paper" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/56fd276a-8001-47ef-bb75-47bff8dda137">
<figcaption> <p align="center">Brockett’s 1983 Product of Exponentials (PoE) formula to calculate the forward kinematics of open chain robots. You can download the paper here: https://link.springer.com/chapter/10.1007/BFb0031048.</figcaption> </p>
</p>
</figure>

Forward kinematics calculates the position and orientation of the robot end-effector from the joint coordinates q. Therefore, it is a **map** that takes the **joint values** to the **position and orientation of the end-effector** frame or {b} frame with respect to the fixed reference frame {s}.

<figure>
<p align="center">
<img width="411" alt="forward kinematics definition" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/34d07a3c-b8a5-47d2-8ddb-badae1672326">
<figcaption> <p align="center">Forward Kinematics FK is a map that takes the joint values to the position and orientation of the end-effector frame.</figcaption> </p>
</p>
</figure>

## Product of Exponentials Formula (PoE)

Now that we have seen how the PoE formula is used to find the forward kinematics of a three-link planar robot arm, let’s look at a more general description. Let’s consider the figure below, which consists of n one-dof joints connected serially in an n-link spatial open chain:

<figure>
<p align="center">
<img width="411" alt="product of  exponentials poe formula - n one dof joints connected serially" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/6c8df8a0-71a8-4977-828c-167e950f17e8">
<figcaption> <p align="center">n one-dof joints connected serially.</figcaption> </p>
</p>
</figure>

The key concept behind the PoE formula is that every joint applies a screw motion to all outward links. In order to find the configuration of the end-effector frame relative to the base frame, we first choose a fixed frame {s} and the end-effector frame {b} attached to the last link.

<figure>
<p align="center">
<img width="411" alt="PoE formula forward kinematics - base frame and end effector frame" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7b1fe455-26d2-41a0-a181-01a5d679b85f">
<figcaption> <p align="center">To find the forward kinematics of an open-chain robot using the PoE formula, we only need to assign the base frame and the end-effector frame.</figcaption> </p>
</p>
</figure>

Then, identify all the joint variables $`q_1 … q_n`$. 

Then, we should place the robot in its **zero position** and we should consider the direction of positive displacement (rotation for revolute joints, translation for prismatic joints) for each joint. Then find M ∈ SE(3), which is the configuration of the end-effector frame with respect to the fixed base frame when the robot is in its zero position:

<figure>
<p align="center">
<img width="411" alt="PoE formula forward kinematics - zero position" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e177a7a9-fe51-4eca-a0b6-50777a48f566">
<figcaption> <p align="center">The robot in zero position is used to find M, which is the configuration of the end-effector frame with respect to the fixed base frame in this position.</figcaption> </p>
</p>
</figure>

Now it's the time to find the screw axes $`\mathcal{S}_1 ... \mathcal{S}_n`$ expressed in the **fixed base frame**, corresponding to the joint motions when the robot is at its zero position. Here, we will have two cases:

- If the joint i is revolute, that corresponds to a screw motion of zero pitch since there is **no linear motion**, then the angular part of the screw axis $`\mathcal{S}_{\omega_{i}} \in \mathbb{R}^3`$ is the unit vector in the positive direction of the joint axis, and the **linear part** of the screw axis can be calculated by the cross product of the angular part and $`a_i`$, $`\mathcal{S}_v = a_i \times \mathcal{S}_\omega`$ where $`a_i`$ is any arbitrary point on joint axis i written in coordinates of the fixed base frame. Here, $`q_i = \theta_i`$ is the joint rotation.

- If joint i is prismatic, then the angular part of the screw axis is zero $`\mathcal{S}_\omega = 0`$, and the linear part $`\mathcal{S}_v \in \mathbb{R}^3`$ is the **unit vector** in the **direction of the positive translation**. Here, $`q_i = d_i`$ is the prismatic extension/retraction.

Note: To understand the different types of robot joints, please refer to the lesson on the degrees of freedom of a robot.

Then, the configuration of the end-effector frame relative to the base frame can be written as the following matrix: 

$`T(q) = e^{[\mathcal{S}_1]q_1} ... e^{[\mathcal{S}_{n-1}]q_{n-1}}e^{[\mathcal{S}_n]q_n} M`$. 

This is called **the Product of Exponentials formula** that describes the forward kinematics of an n-dof open chain. This representation is the **space form** of the PoE formula because the screw axes are expressed in the fixed space frame. Consequently, the forward kinematics is a product of matrix exponentials, each corresponding to a screw motion. 

Note that there is also the **body form of the PoE formula** where the joint axes are screw axes that are expressed in the end-effector or body frame. We will not study this in this lesson since I think that it is basically the same as the space form, and one can easily deduce that by learning the space form.

To summarize, in order to calculate the **forward kinematics** of an **open-chain manipulator** using the **space form** of the PoE formula, we should first find the following elements:

- the joint variables $`q_1 … q_n`$

- the end-effector configuration M ∈ SE(3) when the robot is at its zero position

- the screw axes $`\mathcal{S}_1 ... \mathcal{S}_n`$ expressed in the fixed base frame, corresponding to the joint motions when the robot is at its zero position.

- the pose of the end-effector w.r.t the base frame using the product of exponentials formula

Now let’s talk about why we prefer the PoE formula to other approaches like Denavit-Hartenberg. As you may know, Denavit-Hartenberg (D-H) is another convention for calculating the FK of open-chain robots, which requires a minimum number of parameters. For an n-joint robot, 3n numbers are required to describe the robot’s structure, and n numbers to describe the joint values. PoE representation, however, isn’t minimal. Besides the n joint values, 6n numbers are required to describe the n screw axes. There are, however, advantages that outweigh the only disadvantage.

As opposed to D-H representation, link reference frames do not need to be defined, which is a huge advantage. All we need is the base frame and M. On the other hand, revolute and prismatic joints are treated the same, and the joint axes can intuitively be interpreted geometrically as screws. A major disadvantage of D-H parameters is that link frames are assigned differently, and both link frames and D-H parameters should be given together to fully describe the robot’s forward kinematics.

We can conclude that there’s no compelling reason to prefer D-H parameters over PoE representation unless using a small number of parameters is very important. We will see more advantages of PoE representation when studying velocity kinematics. 

Let’s now see some more examples.

## Example: Forward Kinematics of UR5e 6R Robot Arm from Universal Robots

Consider the UR5e 6R robot arm from [Universal Robots](http://www.universal-robots.com/) shown in the figure below:

<figure>
<p align="center">
<img width="411" alt="ur5e robotic arm from universal robots" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8d6c019a-55e0-41ee-8a52-229ee024dd4c">
</p>
</figure>

Our goal is to determine the forward kinematics of this open-chain collaborative industrial robot. By following the steps outlined in this lesson, we can easily accomplish this.

All joints in this robot are revolute joints, so the vector of joint angles can be written as:

$`q = \begin{pmatrix}
\theta_1\\
\theta_2\\
\theta_3\\
\theta_4\\
\theta_5\\
\theta_6\\
\end{pmatrix}`$

Below is a figure of this robot in its zero position with the base and end-effector frames attached. The screw axes in the robot’s zero position are chosen based on the positive rotation of the joints based on the right-hand rule:

<figure>
<p align="center">
<img width="411" alt="ur5e robot in its zero position - forward kinematics using screw theory" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/1347998b-e5ff-4178-80fa-357522a48fec">
<figcaption> <p align="center">UR5e robotic arm in its zero position with all screw axes defined.</figcaption> </p>
</p>
</figure>

You can see the simulation of the positive direction of the rotation for each joint in the video below:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d26e74fc-9d4d-4042-be7b-8076718b10f2

Note that $`W_1`$ is the distance between the anti-parallel axes of joints 1 and 5. Now it’s time to calculate M, which is the position and orientation of the tool frame relative to the base frame in the robot’s zero position:

$`M = \begin{pmatrix}
1 & 0 & 0 & -L_1 - L_2\\
0 & 0 & -1 & -W_1 - W_2\\
0 & 1 & 0 & H_1 - H_2\\
0 & 0 & 0 & 1
\end{pmatrix}`$

The next step is to calculate the coordinates of **3D arbitrary points** on the joint axes at zero configuration with respect to the coordinates of the fixed base frame:

$`a_1 = \begin{pmatrix}
0\\
0\\
0
\end{pmatrix}, a_2 = \begin{pmatrix}
0\\
0\\
H_1
\end{pmatrix}, a_3 = \begin{pmatrix}
-L_1\\
0\\
H_1
\end{pmatrix}`$

$`a_4 = \begin{pmatrix}
-L_1 - L_2\\
0\\
H_1
\end{pmatrix}, a_5 = \begin{pmatrix}
-L_1 - L_2\\
-W_1\\
0
\end{pmatrix}, a_6 = \begin{pmatrix}
-L_1-L_2\\
0\\
H_1 - H_2
\end{pmatrix}`$

Note that these points can easily be found on the screw axes using simple geometry. Next, we will calculate the screw axis of each joint expressed in the **base frame** in the **robot’s zero position**. First, we calculate the rotation parts of the screw axes:

$`\mathcal{S}_{\omega_1} = \begin{pmatrix}
0\\
0\\
1
\end{pmatrix}, \mathcal{S}_{\omega_2} = \begin{pmatrix}
0\\
-1\\
0
\end{pmatrix}, \mathcal{S}_{\omega_3} = \begin{pmatrix}
0\\
-1\\
0
\end{pmatrix}`$

$`\mathcal{S}_{\omega_4} = \begin{pmatrix}
0\\
-1\\
0
\end{pmatrix}, \mathcal{S}_{\omega_5} = \begin{pmatrix}
0\\
0\\
-1
\end{pmatrix}, \mathcal{S}_{\omega_6} = \begin{pmatrix}
0\\
-1\\
0
\end{pmatrix}`$

Note that, for example, the rotational part of the screw axis of joint 4 is in the opposite direction of the y-axis of the base frame, and it is a unit vector.

Now that we have the rotational parts and the arbitrary points on the screw axes, we can find the linear parts of the screw axes by finding the cross product of the rotational part and the 3D arbitrary point:

$`\mathcal{S}_{v_1} = a_1 \times \mathcal{S}_{\omega_1} =  \begin{pmatrix} 
0\\
0\\
0
\end{pmatrix}, \mathcal{S}_{v_2} = \begin{pmatrix}
H_1\\
0\\
0
\end{pmatrix}, \mathcal{S}_{v_3} = \begin{pmatrix}
H_1\\
0\\
L_1
\end{pmatrix}`$

$`\mathcal{S}_{v_4} =  \begin{pmatrix} 
H_1\\
0\\
L_1 + L_2
\end{pmatrix}, \mathcal{S}_{v_5} = \begin{pmatrix}
W_1\\
-L_1 - L_2\\
0
\end{pmatrix}, \mathcal{S}_{v_6} = \begin{pmatrix}
H_1 - H_2\\
0\\
L_1 + L_2
\end{pmatrix}`$

You can easily calculate this cross product using MATLAB. For instance, for the linear part of the joint 3 axis, we can write:

``` MATLAB
syms L1 L2 H1 H2 W1 W2
a3 = [-L1;0;H1];
Sw3 = [0;-1;0];
Sv3 = cross(a3,Sw3)
```
Stacking up the rotational part and the linear part will give us the screw axis.

Now we have all the necessary elements to find the forward kinematics of the UR5e robotic arm representing the position and orientation of the end-effector with respect to the base frame as:

$`T(\theta) = e^{[\mathcal{S}_1]\theta_1}e^{[\mathcal{S}_2]\theta_2}e^{[\mathcal{S}_3]\theta_3}e^{[\mathcal{S}_4]\theta_4}e^{[\mathcal{S}_5]\theta_5} e^{[\mathcal{S}_6]\theta_6}M`$

As an example for a visual representation of the forward kinematics of this robot, suppose that $`\theta_2 = -\frac{\pi}{2}`$, and $`\theta_5 = \frac{\pi}{2}`$ with all other joint angles to be zero. Also, suppose that $`W_1 = 109`$ mm, $`W_2 = 82`$ mm, $`L_1 = 425`$ mm, $`L_2 = 392`$ mm, $`H_1 = 89`$ mm, and $`H_2 = 95`$ mm. Using the MATLAB function at the end of this lesson, we can find the configuration of the end-effector frame in the base frame as:

$`T_{06} = \begin{pmatrix}
0 & 1 & 0 & -0.095\\
-1 & 0 & 0 & -0.109\\
0 & 0 & 1 & 0.988\\
0 & 0 & 0 & 1
\end{pmatrix}`$

The visualization of this configuration is as follows:

<figure>
<p align="center">
<img width="411" alt="ur5e robotic arm forward kinemactis using screw theory - example configuration" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c475be59-db04-43d4-9a96-893ed880a7d5">
<figcaption> <p align="center">UR5e robotic arm in example configuration (θ2 = -π/2 , and θ5 = π/2 with all other joint angles to be zero)</figcaption> </p>
</p>
</figure>

See the simulation of this pose in the video below:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/4977013b-0805-4d6f-8413-9ca41e4ed553

A question arises here: will we get the same configuration if the base frame is different? To answer this question, we will use RoboDK to show that we’ll get the same configuration as above by selecting a different base frame. We begin by selecting the following reference frame as the base frame:

<figure>
<p align="center">
<img width="411" alt="ur5e robotic arm  forward kinematics with different base frame in zero position" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/9b87d032-c0d5-4fb6-bbf9-cb203e1252dd">
<figcaption> <p align="center"UR5e robotic arm with a different base frame in zero position</figcaption> </p>
</p>
</figure>

Using the same robot joint angles, the matrix representing the configuration of the end-effector frame relative to the base frame can be calculated as follows:

$`T_{06} = \begin{pmatrix}
0 & -1 & 0 & 0.095\\
1 & 0 & 0 & 0.109\\
0 & 0 & 1 & 0.988\\
0 & 0 & 0 & 1
\end{pmatrix}`$

This matrix gives the same configuration as the previous base frame (you can also confirm this by drawing both frames by hand):

<figure>
<p align="center">
<img width="616.5" alt="ur5e robot arm in two poses diff base frame" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/6e3879df-c102-4c0a-b156-d6e83afd132c">
<figcaption> <p align="center"The choice of the base frame is arbitrary when calculating the forward kinematics.</figcaption> </p>
</p>
</figure>

Watch a simulation of this in the video below:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d97abd86-24e6-4b3f-aedc-da02ac5b7b02

This example shows that the choice of the base frame is **arbitrary** when calculating the forward kinematics. Similarly, the end-effector frame can also be chosen arbitrarily as long as it follows the right-hand rule. Typically, frames are chosen with their z-axis aligned in the direction of the joint’s positive movement.

Here, it’s noteworthy that 6-dof robotic arms are commonly referred to as general purpose manipulators, and they are important for robotics and industrial applications since they have a minimum number of joints, allowing the end-effector to move a rigid body in all its degrees of freedom subject to only limits on the robot’s workspace (do you remember from the degrees of freedom lesson that a rigid body in space has 6 degrees of freedom?).

Let’s take a look at an example in which a prismatic joint is used.

## Example: Forward Kinematics of KUKA KR5 SCARA R550 Z200

Consider the RRPR KUKA SCARA robot depicted in the figure below:

<figure>
<p align="center">
<img width="411" alt="KUKA scara robot forward kinematics using screw theory - home position-2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/994a1fbf-f74a-4463-bf56-0f6f12bd1ddb">
<figcaption> <p align="center"KUKA KR5 SCARA R550 Z200 robot in zero position</figcaption> </p>
</p>
</figure>

<figure>
<p align="center">
<img width="411" alt="KUKA scara robot forward kinematics using screw theory - home position-side view-2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/dbce27a0-45c1-4488-b596-82885022202f)">
<figcaption> <p align="center"KUKA KR5 Scara R550 Z200 robot in zero position – side view</figcaption> </p>
</p>
</figure>

As shown in the video below, this robot has three **rotational** degrees of freedom and one **translational** degree of freedom, which makes it a useful pick-and-place robot. The set of joint variables are $`q = (\theta_1,\theta_2,d_3,\theta_4)`$.

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/17dd97cc-f986-47eb-b991-dc9e8829c72d

From the [robot’s technical specs](https://www.kuka.com/-/media/kuka-downloads/imported/6b77eecacfe542d3b736af377562ecaa/pf0056_kr_5_scara_en.pdf), we can calculate $`\ell_1 = 325`$ mm, $`\ell_2 = 225`$ mm, and $`\ell_0 = 46`$ mm. Now following the steps to calculate the forward kinematics of this robot using screw theory, we can write the matrix M and the screw axes as:

$`M = \begin{pmatrix}
1 & 0 & 0 & \ell_1 + \ell_2\\
0 & -1 & 0 & 0\\
0 & 0 & -1 & l_0\\
0 & 0 & 0 & 1
\end{pmatrix}, \mathcal{S}_1 = \begin{pmatrix}
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
-\ell_1\\
0
\end{pmatrix}, \mathcal{S}_3 = \begin{pmatrix}
0\\
0\\
0\\
0\\
0\\
1
\end{pmatrix}, \mathcal{S}_4 = \begin{pmatrix}
0\\
0\\
-1\\
0\\
\ell_1 + \ell_2\\
0
\end{pmatrix}`$

Note that since one of the joints (3rd joint) is prismatic, the linear part of the screw axis is the unit vector in the direction of the translation which is in the z-axis direction of the base frame.

For educational purposes and using our MATLAB code, the configuration of the end-effector relative to the base frame for the set of joint variables $`q = (0,\frac{\pi}{2}, 10 \, mm, -\frac{\pi}{2})`$ can be found as:

$`T = \begin{pmatrix}
-1 & 0 & 0 & 325\\
0 & 1 & 0 & 225\\
0 & 0 & -1 & 56\\
0 & 0 & 0 & 1
\end{pmatrix}`$

This configuration can be visualized as the following figure:

<figure>
<p align="center">
<img width="411" alt="KUKA scara robot forward kinematics using screw theory - sample configuration" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/db01e8af-b49c-4b60-8352-f76a52b066a4">
<figcaption> <p align="center"Configuration of the end-effector of the KUKA KR5 Scara R550 Z200 robot relative to the base frame for the set of joint variables q = (0,π/2,10mm,-π/2).</figcaption> </p>
</p>
</figure>

See the simulation below for this pose:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/cb8eeff7-3448-4f57-90c8-49caaf69dd80

The following code helps you to calculate the forward kinematics using the PoE formula:

``` MATLAB
function [R,p]=FK_PoE(q,a,rot,jt,M)

% this code caclulate the forward kinematics using the product of
% exponentials PoE formula
% written by Madi Babaiasl

% function inputs:

% q: vector of joint positions --> theta for revolute joints and d for 
% prismatic joints

% a: 
% for revolute joints: matrix of 3D arbitrary points on the joint axes at zero configuration 
   % written in coordinates of the fixed base frame where each row belongs 
   % to one point
% for prismatic joint: the row is the unit vector in the direction of the
% joint movement

% rot: matrix of 3D rotational part of the screw axes at zero configuration, where each
% row belongs to one joint and each row is a unit vector. For rotational
% joints this is rotation axis and for prismatic joints, this is
% zero

% jt: a string of letters showing joint types, e.g. 'RRP'

% M: Homogenous transformation matrix of end-effector w.r.t. base at zero configuration

% outputs:

% R is the rotation matrix of the end-effector w.r.t. base
% p is the position of the end-effector w.r.t. base
%-------------------------------------------------------------------------

    T=POE(q,a,rot,jt);
    Tf=T*M;    

    R=Tf(1:3,1:3);
    p=Tf(1:3,4);

end

% utility function ----------------------------------------------------
function T=POE(q,a,rot,jt)

    T=eye(4,4);
    
    % number of joints
    n=length(q);

    for ii=n:-1:1

        if jt(ii)=='R'     
            rot_hat=[0       -rot(ii,3)     rot(ii,2);...
                   rot(ii,3)      0      -rot(ii,1);...
                   -rot(ii,2)    rot(ii,1)      0];        
            e_rot_hat=eye(3,3)+rot_hat*sin(q(ii))+rot_hat*rot_hat*(1-cos(q(ii)));
        elseif jt(ii)=='P'
            rot_hat=zeros(3,3);
            e_rot_hat=eye(3,3);
        end
    
        if (jt(ii)=='R') & (ii>1)
            Sv=-transpose(cross(rot(ii,:),a(ii,:)));
        elseif (jt(ii)=='R') & (ii==1)
            Sv=[0;0;0];
        elseif jt(ii)=='P'
            Sv=a(ii,:)';
        end

        p=(eye(3,3)*q(ii)+(1-cos(q(ii)))*rot_hat+(q(ii)-sin(q(ii)))*rot_hat*rot_hat)*Sv;

        e_zai=[e_rot_hat p;0 0 0 1];
    
        T=e_zai*T;
    
    end

end
```

Example of using this function for the UR5e robot arm: 

``` MATLAB
t1 = 0;
t2 = -pi/2;
t3 = 0;
t4 = 0;
t5 = pi/2;
t6 = 0;
theta = [t1;t2;t3;t4;t5;t6];
W1 = 109/1000;
W2 = 82/1000;
L1 = 425/1000;
L2 = 392/1000;
H1 = 89/1000;
H2 = 95/1000;

a = [0 0 0;0 0 H1;-L1 0 H1;-L1-L2 0 H1; -L1-L2 -W1 0;-L1-L2 0 H1-H2];
rot = [0 0 1;0 -1 0;0 -1 0;0 -1 0;0 0 -1;0 -1 0];
jt = 'RRRRRR';
M = [1 0 0 -L1-L2;0 0 -1 -W1-W2;0 1 0 H1-H2;0 0 0 1];

[R,p]=FK_PoE(theta,a,rot,jt,M)
```

## References

- [Modern Robotics: Mechanics, Planning, and Control by Frank Park and Kevin Lynch](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)

- A Mathematical Introduction to Robotic Manipulation by Murray, Lee, and Sastry

- [Denavit, Jacques, and Richard S. Hartenberg. “A kinematic notation for lower-pair mechanisms based on matrices.” (1955): 215-221.](https://asmedigitalcollection.asme.org/appliedmechanics/article-abstract/22/2/215/1110292/A-Kinematic-Notation-for-Lower-Pair-Mechanisms)

- [Brockett, R.W., 1984. Robotic manipulators and the product of exponentials formula. In Mathematical theory of networks and systems (pp. 120-129). Springer, Berlin, Heidelberg.](https://link.springer.com/chapter/10.1007/BFb0031048)

- [RoboDK software](https://robodk.com/)
