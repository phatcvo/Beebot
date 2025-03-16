## Introduction 

In the previous lesson, we saw how to calculate the robot's tool frame's **pose** (position and orientation) for a given set of joint **positions**. In this lesson, we will start to talk about **velocity kinematics** and this time we are interested in relating the end-effector's **velocity** to **joint velocities**. In order to be able to do this, we first should study about velocities in robotics and in particular **angular velocities** and **twists**. In this lesson, we will become familiar with the concept of velocities in robotics, and we will study angular velocities and twists.

A rigid body’s velocity can be represented as a point in a **6D space**, consisting of the **angular velocity** and the **linear velocity**, which together are called a **spatial velocity** or a **twist**. There are several ways to express the angular velocities and twists that we
will discuss in the coming paragraphs.

## Angular Velocities in Robotics  

In order to understand the angular velocity concept, suppose a rotating body with the coordinate frame axes attached to it at time t and rotating with the body about an arbitrary axis passing through the origin by an angle $`\Delta{\theta}`$. The axis of rotation is coordinate-free for now, which means that it is not yet represented in any particular frame. A small change in the orientation of this frame can be depicted in the figure below (note that each of the coordinate axes is of unit length, so only the direction can vary with time):

<figure>
<p align="center">
<img width="416" alt="rotation-around-arbitrary-axis" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b1ea6031-7dea-40a7-b67b-eeb93c62eec6">
<figcaption> <p align="center">This figure shows a slight change in the orientation of the frame instantaneously attached to a rotating body that is rotating about an arbitrary axis in space.</figcaption> </p>
</p>
</figure>

The above figure shows a slight change in the orientation of the frame instantaneously attached to a rotating body that is rotating about an arbitrary axis in space. For a visualization of the rotation of the coordinate frame instantaneously attached to a rotating body about an arbitrary axis in space, you can refer to the lesson about [exponential coordinates](https://github.com/madibabaiasl/modern-robotics-course/wiki/Lesson-4:-Orientation-in-Robotics-%E2%80%90-Exponential-Coordinates,-and-Euler-Angles). 

The rate of rotation of the moving body about the arbitrary axis can be defined as:

$`\lim_{\Delta t\to 0} \frac{\Delta \theta}{\Delta t} = \dot{\theta}`$

Note that the **rate of rotation** is analogous to the concept of **speed** in physics, which is a number compared to the **velocity** that has an **amplitude** and a **direction**. Therefore the rotation of a rotating body about an arbitrary axis with a known speed (rate) of rotation can be visualized as the following figure:

<figure>
<p align="center">
<img width="443" alt="rotating-body-around-arbitrary-axis-space" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/90490319-9bab-400c-96ee-30095dc70366">
<figcaption> <p align="center">The coordinate axes are attached to a rotating body that is rotating about an arbitrary axis with a known speed of rotation (rate of rotation).</figcaption> </p>
</p>
</figure>

The motion of the frame as it rotates about the arbitrary axis is according to the right-hand rule, and any **angular velocity** can be represented by a **rotation axis** and the **speed of rotation** about it:

$`\omega = \hat{\omega}\dot{\theta}`$

As the coordinate axes rotate about the arbitrary axis, the coordinate axes trace a **circular path**, as we saw in the video in the exponential coordinates lesson. For instance, if the coordinate frame axes shown in the figure below rotate with a constant rotational speed about the arbitrary axis, each of the coordinate axes traces a circular path as shown for the x-axis:

<figure>
<p align="center">
<img width="295" alt="frame-rotation-around-arbitrary-axis-circular-path" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/99e39eee-5731-47ff-ab65-f23ac0e66189">
<figcaption> <p align="center">Any axis of a coordinate frame rotating about an arbitrary axis traces a circle.</figcaption> </p>
</p>
</figure>

From physics, we know that an object in circular motion experiences a **centripetal acceleration**, and the **linear velocity** is in the direction **tangent** to the circle that can be calculated from the **cross product** of the **angular velocity** and the **radius vector**. If you are not familiar with circular motion the following lecture from Dr. Walter Lewin is highly recommended:

https://youtu.be/mWj1ZEQTI8I

Similarly, here, the linear velocity of each axis is in the direction tangent to the circle, and it is the cross product of the angular velocity and the radius vector, which is the axis itself. For example for the x-axis, it is like this:

<figure>
<p align="center">
<img width="291" alt="frame-rotating-arbitrary-axis-linear-velocity" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/1e9b00f5-416a-407f-9df3-7849de581fca">
<figcaption> <p align="center">The linear velocity of an axis of a frame attached to a body rotating about an arbitrary axis with a known angular velocity is in the direction tangent to the circular path and can be calculated by the cross product of the angular velocity and radius vector, which is the axis itself in this case.</figcaption> </p>
</p>
</figure>

So the linear velocities of frame axes rotating about an arbitrary axis with a known angular velocity are in the direction tangent to the circular path and can be calculated as:

$`\dot{\hat{x}} = \omega \times \hat{x}, \, \dot{\hat{y}} = \omega \times \hat{y}, \, \dot{\hat{z}} = \omega \times \hat{z}`$

**Example:** Consider that the Pepper robot from SoftBank Robotics is rotating its elbow while keeping all other joints fixed:

<figure>
<p align="center">
<img width="436" alt="pepper-robot-rotation-of-the-elbow-angular-velocity-robotics-1" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c85b3f89-2f0b-43f8-a185-c1915e36538e">
</p>
</figure>

A motion like this:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7f1b5401-5524-4632-9fa8-ff6953c36319

Then the linear velocities of the points on the forearm satisfy the following equations:

$`\begin{matrix}
v_i = \omega \times r_i = \omega \times (p_i - p_E), \, i = 1,2,3\\
\omega = \hat{\omega}_s \dot{\theta}
\end{matrix}`$

Note that the points farthest from the center of the elbow have larger linear velocities. The radius of rotation is measured from the center of rotation to the point.

Now let’s see how we can determine the angular velocity with respect to different coordinate frames.

## Expressing the Angular Velocity Relative to Different Coordinate Frames 

Now we want to express the above equations with respect to coordinate frames. We can choose a reference frame for the angular velocity, and our natural choices are the fixed frame {s} and the body frame {b}. If the rotation axis is defined in the {s} frame, then the angular velocity expressed in the {s} frame can be written as:

$`\omega_s = \hat{\omega}_s \dot{\theta}`$

A similar equation can be written for the angular velocity expressed in the {b} frame. We then want to find expressions for the angular velocities in the {s} and {b} frames in terms of the rotation matrix. Suppose that R(t) is a rotation matrix that represents the orientation of the body frame with respect to the fixed frame at time t that can be expressed in terms of its column vectors as:

$`R(t) = \begin{bmatrix}
r_1(t) & r_2(t) & r_3(t)
\end{bmatrix}`$

As we know from the rotation matrices lesson, the column vectors of this rotation matrix are the **coordinate axes** of the body frame expressed in the fixed frame coordinates. Thus the time rate of change of R(t) can be described as:

$`\begin{aligned}
\dot{R} &= \begin{bmatrix}
\dot{r}_1 & \dot{r}_2 & \dot{r}_3
\end{bmatrix} \\
&= \begin{bmatrix}
\omega_s \times r_1 & \omega_s \times r_2 & \omega_s \times r_3
\end{bmatrix} \\
&= \omega_s \times R\\
&= [\omega_s] R
\end{aligned}`$

Where $`[\omega_s]`$ is the $`3 \times 3`$ skew-symmetric matrix representation of the angular velocity $`\omega_s \in \mathbb{R}^3`$ expressed in the fixed frame coordinates at a specific time. We can use this notation to eliminate the cross-product. With the above definition, we can now write the angular velocity vector in {s} frame as:

$`[\omega_s] R = \dot{R} \rightarrow [\omega_s] R R^{-1} = \dot{R}R^{-1} \rightarrow [\omega_s] = \dot{R}R^{-1} = \dot{R} R^T`$

From the rotation matrices lesson, we know that the inverse of a rotation matrix is the same as its transpose. Now we want to find the angular velocity vector in the body frame coordinates. Note that $`\omega_s`$ and $`\omega_b`$ are two different representations of the same angular velocity $`\omega`$. In the lesson about rotation matrices, we learned that one of the applications of a rotation matrix is to serve as an operator to change the frame of reference of a frame or a vector. Thus we can find the relationship between the two different representations of the angular velocity using the rotation matrix and the subscript cancellation rule:

$`\omega_s = R_{sb} \omega_b`$

Therefore, $`\omega_b = R^{-1}_{sb} \omega_s = R^{-1}\omega_s = R^{T}\omega_s`$. 

Now we want to write the skew-symmetric matrix form of the angular velocity vector in the body frame:

$`[\omega_b] = [R^{T}\omega_s] =R^{T}[\omega_s]R = R^{T} \dot{R} R^T R = R^{T}\dot{R}`$

This equation is derived using the fact that for any angular velocity $`\omega \in \mathbb{R}^3`$ and rotation matrix $`R \in SO(3)`$, we can write: $`R[\omega]R^T = [R\omega]`$.

**In summary**, equations relating R (orientation of the rotating frame as seen from the fixed frame) and $`\dot{R}`$ to the angular velocity of the rotating frame $`\omega`$ can be written as follows:

$`\displaylines{[\omega_s] = \dot{R} R^{T}\\
[\omega_b] =  R^{T}\dot{R}}`$

where $`\omega_b \in \mathbb{R}^3`$ is the body frame vector representation of $`\omega`$ and $`[\omega_b] \in so(3)`$ is its $`3 \times 3`$ skew-symmetric matrix representation. Similarly, $`\omega_s \in \mathbb{R}^3`$ is the fixed frame representation of $`\omega`$ and $`[\omega_s] \in so(3)`$ is its $`3 \times 3`$ matrix representation. 

Note that fixed-frame angular velocity $`\omega_s`$ is not dependent on the choice of the body frame, and the body-frame angular velocity $`\omega_b`$ is not dependent on the choice of the fixed frame. This is because although R and Ṙ are individually dependent on both the {s} and {b} frames, the product $`\dot{R}R^{T}`$ is independent of the {b} frame, and the product $`R^{T}\dot{R}`$ is independent of the {s} frame.

Note also that we can represent the angular velocity expressed in an arbitrary frame {d} in another frame {c} if we know the rotation that takes the {c} to {d}: $`\omega_c = R_{cd}\omega_d`$. 

**Example:** Suppose that in terms of the axes of the space frame {s}, the body frame’s x-axis and y-axis are in (0,0,1) and (-1,0,0) directions, respectively. An angular velocity $`\omega`$ is expressed in the {s} frame as $`\omega_s = (3,2,1)`$. We want to find the representation of the angular velocity in {b} frame coordinates. 

For this, the two frames {s} and {b} can be drawn as:

<figure>
<p align="center">
<img width="442" alt="angular-velocities-two-frames-example" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c201ada0-63fc-4d33-aad4-c31297a63ee4">
</p>
</figure>

Note that since we do not have the position information, we can suppose that the origins are the same, and they are illustrated separately for clarification. As we just learned, we can represent the angular velocity in another coordinate frame using the rotation matrix representation of the orientation of one frame relative to the other. For our example, we can write:

$`\omega_b = R_{bs} \omega_s`$

The rotation matrix representing the orientation of the {s} frame relative to the {b} frame can be calculated by taking the transpose of the rotation matrix representing the orientation of the {b} frame relative to the {s} frame as:

$`R_{bs} = R_{sb}^T = \begin{pmatrix}
0 & 0 & 1\\
-1 & 0 & 0\\
0 & -1 & 0
\end{pmatrix}`$

Thus the representation of the angular velocity in the body frame can be calculated as:

$`\omega_b = \begin{pmatrix}
0 & 0 & 1\\
-1 & 0 & 0\\
0 & -1 & 0
\end{pmatrix}\begin{pmatrix}
3\\
2\\
1
\end{pmatrix} = \begin{pmatrix}
1\\
-3\\
-2
\end{pmatrix}`$

Now, let’s see what the concept of twists means in Robotics.

## Twists in Robotics 

In robotics, a twist is **a continuous change in pose over time**. A twist can be defined as an angular velocity and the linear velocity of a
moving body combined together into a compact 6-vector:

$`\mathcal{V} = \begin{pmatrix}
\omega\\
v
\end{pmatrix}_{6 \times 1} \in \mathbb{R}^6`$

In the previous section of this lesson about angular velocities, we saw that pre- or post-multiplying $`\dot{R}`$ by $`R^T`$ results in the skew-symmetric matrix representation of the angular velocity vector either in the fixed or body frame coordinates. Let’s find out if similar physical interpretations hold for the multiplications $`T^{-1} \dot{T}`$ and $`\dot{T}T^{-1}`$. 

Suppose that {s} is the fixed (space) frame and {b} is the moving (body) frame: 

<figure>
<p align="center">
<img width="442" alt="represenation-robot-motion-space" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/9251508d-2386-4531-89c6-083caa09ade7">
</p>
</figure>

and the configuration of the body frame {b} as seen from the space frame {s} is defined as:

$`T_{sb}(t) = T(t) = \begin{pmatrix}
R(t) & p(t)\\
o & 1
\end{pmatrix}`$

Then we can write:

$`T^{-1}\dot{T} = \begin{pmatrix}
R^T & -R^Tp\\
o & 1
\end{pmatrix}\begin{pmatrix}
\dot{R} & \dot{p}\\
o & 0
\end{pmatrix} = \begin{pmatrix}
R^T\dot{R} & R^T\dot{p}\\
o & 0
\end{pmatrix}`$

Refer to the lesson on homogenous transformation matrices to understand how to calculate the inverse of the transformation matrix T. As we saw in the angular velocities section, $`R^T \dot{R}`$ is the skew-symmetric matrix representation of the angular velocity in {b} coordinates,$`[\omega_b]`$. ṗ is the **linear velocity of the origin of the {b} frame expressed in the fixed frame {s}**, and thus $`R^T \dot{p}`$ is the linear **velocity of the origin of the {b} frame defined in the {b} frame**: $`R^T \dot{p} = v_b`$. This makes sense since $`R_{sb}^T = R_{sb}^{-1} = R_{bs}`$ and the multiplication of $`R_{bs}`$ and $`\dot{p}_s`$ changes the reference frame from {s} to {b} (subscript cancellation rule). Therefore, $`T^{-1}\dot{T}`$ can be re-written in terms of the angular velocity and the linear velocity as:

$`T^{-1}\dot{T} = [\mathcal{V}_b] = \begin{pmatrix}
[\omega_b] & v_b\\
o & 0
\end{pmatrix} \in se(3)`$

Where $`[\omega_b] \in so(3)`$ is the skew-symmetric matrix representation of the angular velocity and $`v_b \in \mathbb{R}^3`$ is the **linear velocity of a point at the origin of the body frame expressed in that frame**. Thus we can conclude that $`T^{-1}\dot{T}`$ represents the matrix representation of the body twist which is the spatial velocity in the body frame and be defined as:

$`\mathcal{V}_b = \begin{pmatrix}
\omega_b\\
v_b
\end{pmatrix} \in \mathbb{R}^6`$

Note that here bracket notation [] shows the matrix representation of the body twist. 

We can find a similar physical interpretation for the multiplication $`\dot{T}T^{-1}`$ as:

$`\begin{aligned}
\dot{T}T^{-1} &= \begin{pmatrix}
\dot{R} & \dot{p}\\
o & 0
\end{pmatrix}\begin{pmatrix}
R^T & -R^T p\\
o & 1
\end{pmatrix}\\
&= \begin{pmatrix}
\dot{R}R^T & \dot{p}-\dot{R}R^T p\\
o & 0
\end{pmatrix}\\
&= \begin{pmatrix}
[\omega_s] & v_s\\
o & 0
\end{pmatrix}
\end{aligned}`$

In the angular velocity section, we saw that $`\dot{R}R^T`$ is the skew-symmetric matrix representation of the angular velocity expressed in the fixed frame coordinates. Now let’s figure out what $`v_s = \dot{p} - \dot{R} R^T p = \dot{p} - [\omega_s]p = \dot{p} - \omega_s \times p`$ means which is obviously not the linear velocity of the body frame origin expressed in the fixed frame (which we know that equals to ṗ). Suppose we have an infinitely large rigid body that the origins of the frames {s} and {b} are two points on this rigid body, and p is the location of the origin of the {b} frame relative to the {s} frame: 

<figure>
<p align="center">
<img width="436" alt="Physical interpretation of vs" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/6d042155-6fc3-4ca4-9817-6878c15a3d94">
</p>
</figure>

As the body moves, the origin of the {b} frame traces a circular path of radius p relative to the origin of the frame {s}. From the dynamics of rigid bodies, we can easily write the motion of the origin of the frame {b} relative to the motion of the origin of the frame {s} as:

$`\dot{p} = v_s + \omega_s \times p \rightarrow v_s = \dot{p} - \omega_s \times p`$

where $`\omega_s = \hat{\omega}_s \dot{\theta}`$ is the angular velocity of the rigid body expressed in the frame {s}; therefore $`\omega_s \times p`$ is the linear velocity of the origin of the frame {b} during rotational motion that is tangent to the circular path expressed in the {s} frame. 

**Start of dynamics of rigid bodies note**

In rigid dynamics, if we have two points on a rigid body like the following figure:

<figure>
<p align="center">
<img width="436" alt="rigid body relative velocity" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/3096eb38-7a4f-4d12-8c60-0df1795a222c">
</p>
</figure>

Then, the relative motion of one point w.r.t to the other point can be expressed as:

$`v_A = v_B + \omega \times r_{A|B}`$

**End of dynamics of rigid bodies note**

Therefore, we can conclude that $`v_s`$ is the **instantaneous velocity of the point** on the infinitely large rigid body currently **at the origin of the {s} frame expressed in the {s} frame**. With these definitions, the spatial twist, which is the spatial velocity in the space frame, can be expressed as:

$`\mathcal{V}_s = \begin{pmatrix}
\omega_s\\
v_s
\end{pmatrix} \in \mathbb{R}^6`$

And the matrix representation of the spatial twist can be found as:

$`[\mathcal{V}_s] = \begin{pmatrix}
[\omega_s] & v_s\\
o & 0
\end{pmatrix} = \dot{T}T^{-1} \in se(3)`$

Note that here, as we discussed earlier, the bracket notation does not imply a skew-symmetric matrix, but it only wants to show the 4×4 matrix representation of the twist. 

Also, note that like the angular velocity, for any given twist, its fixed-frame representation $`\mathcal{V}_s`$ is not dependent on the choice of the body frame, and its body frame representation $`\mathcal{V}_b`$ does not depend on the choice of the fixed frame.

From the equations that we obtained for the body twist and the spatial twist, we can find a relationship between them. The matrix representations of the body twist and the spatial twist will then have the following relationship: 

$`[\mathcal{V}_b] = T^{-1} \dot{T} = T^{-1}[\mathcal{V}_s]T \text{ \& } [\mathcal{V}_s] = T[\mathcal{V}_b]T^{-1}`$

Expanding one of the equations and inserting the values for T and $`T^{-1}`$, we can find a relationship between the spatial and the body twists:

$`\begin{aligned}
{[\mathcal{V}_s]}  
&= T[\mathcal{V}_b]T^{-1} \\
&= \begin{pmatrix}
R & p\\
o & 1
\end{pmatrix}\begin{pmatrix}
[\omega_b] & v_b\\
o & 0
\end{pmatrix}\begin{pmatrix}
R^T & -R^Tp\\
o & 1
\end{pmatrix}\\
&= \begin{pmatrix}
R[\omega_b]R^T & -R[\omega_b]R^Tp + Rv_b\\
o & 0
\end{pmatrix}
\end{aligned}`$

We saw that $`R[\omega]R^T = [R\omega]`$ and it is easy to see that $`[\omega]p = -[p]\omega`$ for $`p,\omega \in \mathbb{R}^3`$. Note: $`[\omega]p = \omega \times p = -p \times \omega = -[p] \omega`$. Inserting these into the above equation, we get the following relationship for the matrix representation of the spatial twist:

$`[\mathcal{V}_s] = \begin{pmatrix}
[R\omega_b] & [p]R\omega_b + Rv_b\\
o & 0
\end{pmatrix} = \begin{pmatrix}
[\omega_s] & v_s\\
o & 0
\end{pmatrix} \in se(3)`$

Therefore, we can write the relationship between the spatial twist and the body twist as:

$`\mathcal{V}_s = \begin{pmatrix}
\omega_s\\
v_s
\end{pmatrix} = \begin{pmatrix}
R & o\\
[p]R & R
\end{pmatrix} \begin{pmatrix}
\omega_b\\
v_b
\end{pmatrix} = [Ad_{T_{sb}}]\mathcal{V}_b`$

This relationship is beneficial when changing the frame of reference for twists and wrenches that we will study in the later lessons. Similarly, we can find a similar relationship for the body twist in terms of the spatial twist:

$`\begin{aligned}
{[\mathcal{V}_b]}  
&= T^{-1}[\mathcal{V}_s]T \\
&= \begin{pmatrix}
R^T & -R^Tp\\
o & 1
\end{pmatrix}\begin{pmatrix}
[\omega_s] & v_s\\
o & 0
\end{pmatrix}\begin{pmatrix}
R & p\\
o & 1
\end{pmatrix}\\
&= \begin{pmatrix}
R^T[\omega_s]R & -R^T[\omega_s]p + R^Tv_s\\
o & 0
\end{pmatrix}\\
&= \begin{pmatrix}
[\omega_b] & v_b\\
o & 0
\end{pmatrix}\in se(3)
\end{aligned}`$

Therefore,

$`\mathcal{V}_b = \begin{pmatrix}
\omega_b\\
v_b
\end{pmatrix} = \begin{pmatrix}
R^T & o\\
-R^T [p] & R^T
\end{pmatrix}\begin{pmatrix}
\omega_s\\
v_s
\end{pmatrix} = [Ad_{T_{bs}}]\mathcal{V}_s`$

Note that both the body twist and the spatial twist represent the same motion just in different coordinate frames. The matrix that can change the frame of representation of a twist or a screw is called the **Adjoint representation** of a transformation matrix $`T_{ab}`$. The adjoint and not the transformation matrix T is used to change the frame of representation of a twist because of dimension mismatch. $`\mathcal{V}_a = [Ad_{T_{ab}}]\mathcal{V}_b`$ is the modified version of the subscript cancellation rule to change the frame of representation of a twist. 

So formally, If $`T = (R,p) \in SE(3)`$, then its adjoint representation $`[Ad_T]`$ is:

$`[Ad_T] = \begin{pmatrix}
R & o\\
[p]R & R
\end{pmatrix} \in \mathbb{R}^{6 \times 6}`$

For any $`\mathcal{V} \in \mathbb{R}^6`$, the adjoint map associated with T is $`\mathcal{V}' = [Ad_T] \mathcal{V}`$. In terms of the matrix form $`[\mathcal{V}] \in se(3)`$ of $`\mathcal{V} \in \mathbb{R}^6`$:

$`[\mathcal{V}'] = T[\mathcal{V}]T^{-1}`$.

**Example.** Suppose that the body frame’s x-axis and y-axis are in the direction (0,0,1) and (-1,0,0) relative to the {s} frame and the origin of the {b} frame relative to the {s} frame is (3,0,0). The twist $`\mathcal{V}`$ is represented in the space frame {s} as $`\mathcal{V}_s = (3,2,1,-1,-2,-3)`$. What is the representation of the same twist in body frame {b}?

Based on the coordinates of the {b} frame relative to the space frame, we can draw the two frames as the following figure:

<figure>
<p align="center">
<img width="436" alt="twist-one-frame-to-another-example-screw-theory" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/5af52353-fafd-46eb-9a83-b38e2b93d979">
</p>
</figure>

We learned that the adjoint representation of the transformation matrix could be used to relate the spatial twist and the body twist. So, the body twist can be written as the following equation in terms of the spatial twist:

$`\begin{aligned}
{\mathcal{V}_b}  
&= \begin{pmatrix}
R_{sb}^T & o\\
-R_{sb}^T[p_{sb}] & R_{sb}^T
\end{pmatrix} {\mathcal{V}_s} \\
&= \begin{pmatrix}
0 & 0 & 1 & 0 & 0 & 0\\
-1 & 0 & 0 & 0 & 0 & 0\\
0 & -1 & 0 & 0 & 0 & 0\\
0 & -3 & 0 & 0 & 0 & 1\\
0 & 0 & 0 & -1 & 0 & 0\\
0 & 0 & -3 & 0 & -1 & 0
\end{pmatrix}\begin{pmatrix}
3\\
2\\
1\\
-1\\
-2\\
-3
\end{pmatrix}\\
&=\begin{pmatrix}
1\\
-3\\
-2\\
-9\\
1\\
-1
\end{pmatrix}
\end{aligned}`$

## Screws: a Geometric Description of Twists in Robotics 

In the previous lessons, we saw that every rigid body displacement could be obtained by a finite rotation about and translation along a fixed screw axis 𝘚 and we became familiar with the exponential coordinates of robot motions.

<figure>
<p align="center">
<img width="277" alt="screw interpretation of a twist" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8e878cf8-d44f-490c-a8f1-be1cf2d84495">
</p>
</figure>

In this lesson, we learned that the angular velocity ω could be represented by a unit axis $`\hat{\omega}`$ and the rate of rotation $`\dot{\theta}`$ about this axis. Similarly, we can express a twist (that has an angular component and a linear component) as a screw axis 𝘚 and the rate $`\dot{q}`$ about this axis:

$`\mathcal{V} = \mathcal{S}\dot{q}`$

As for the calculation for the screw axis that we saw in Lesson 6, we can do a similar approach for the calculation of the twist $`\mathcal{V} = (\omega,v)`$:

- Case 1: There is a rotation, so $`\omega = \hat{\omega}\dot{\theta}`$ and $`v = a \times \omega + h\omega`$. In this case, a is a point (any point) on the screw axis, and h is called the pitch of the screw axis that we saw how to calculate it before. 

<figure>
<p align="center">
<img width="624" alt="screw motion and twist" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b8897206-6bdf-498b-abc8-c4122c63b057">
</p>
</figure>

In this case, then $`\mathcal{V} = \mathcal{S}\dot{\theta}`$. 

- Case 2: There is only translation, so $`\omega = 0`$ and $`v = \dot{d} \hat{d}`$ in which $`\hat{d}`$ is the unit vector in the direction of the translation. 

In this case, $`\mathcal{V} = \mathcal{S}\dot{d}`$. 

## References

- Kevin, M.L. and Frank, C.P., 2017. Modern robotics: mechanics, planning, and control
- Murray, R.M., Li, Z. and Sastry, S.S., 2017. A mathematical introduction to robotic manipulation. CRC press.
- Corke, P., 2023. Robotics, Vision and Control: Fundamental Algorithms in Python (Vol. 146). Springer Nature