As we talked in the previous lesson, the configuration of the robot answers the question: where is the robot? In order to know where the robot is, we need to know the position and orientation of the robot's body frame {b} w.r.t some base (space) frame {s}:

<figure>
<p align="center">
<img width="514" alt="pose-in-space" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/071bc984-0549-487e-bf38-0e88eff3b12d">
<figcaption> <p align="center">The position and orientation of a robot in space can be represented by the position of the origin of the body frame expressed in the space frame coordinates and the directions of the coordinate axes of the body frame expressed in the space frame coordinates.</figcaption> </p>
</p>
</figure>

In this lesson, we want to talk about the robot's orientation and one of the ways that we can express this orientation. Before divining into more details, let's see some **preliminary concepts** that we will use when studying the robot's configuration.  

## Free Vector vs. a Vector

A **free vector** is a **geometric quantity** and an arrow in n-dimensional flat space $`R^n`$ that is not rooted anywhere. It has a **length** and a **direction**. In robotics, a free vector is denoted by v. 

A **vector** is a free vector expressed with its **coordinates** in a **reference frame** and **length scale** chosen for that space. In robotics, vectors are represented by an italic letter $`v`$ in $`R^n`$.

A vector is **dependent** on the **choice of the coordinate frame** and **length scale**, whereas the underlying **free vector** is **unchanged** by the choice of the coordinate frame or the length scale. In other words, v is **coordinate-free**.

<figure>
<p align="center">
<img width="514" alt="free-vector-vs-a-vector" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/822acd81-b50a-4715-905d-217c48a409ae">
<figcaption> <p align="center">A free vector is coordinate-free, whereas a vector is dependent on the choice of the coordinate frames and length scale.</figcaption> </p>
</p>
</figure>

A vector can also represent a point p in the physical space. If we choose to give this physical space a reference frame and a length scale, the point is a vector from the origin of this reference frame to the point. It is then represented as an italic $`p \in R^n`$. The same point has a different representation by changing the length scale and the reference frame:

<figure>
<p align="center">
<img width="514" alt="point-coordinates-in-different-frames" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c7f5c6b2-3131-45c5-9321-d961043f4804">
<figcaption> <p align="center">A physical point p can have different representations relative to different coordinate frames.</figcaption> </p>
</p>
</figure>

## Frames in Robotics

In Robotics, frames are important. We use **frames** to represent the robot’s **configurations**, **velocities**, and **forces** causing the motion. Frames in robotics:

- have an **origin**
- consist of orthogonal x, y, and z **coordinate axes**:

<figure>
<p align="center">
<img width="514" alt="Right-handed-coordinate-frame" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e7eb1d93-305d-4688-b7f3-1dfabdbc870f">
<figcaption> <p align="center">A right-handed coordinate frame consists of x, y, and z coordinate axes. The index finger is in the x-direction, the middle finger is in the y-direction, and the thumb is in the z-direction.</figcaption> </p>
</p>
</figure>

- are right-handed, and this means that the cross product of the x and y axes is z, and so on:

$\displaylines {\hat{x} \times \hat{y} = \hat{z}\\
\hat{y} \times \hat{z} = \hat{x}\\
\hat{z} \times \hat{x} = \hat{y}}$

- are stationery! This is from Newton’s laws that the reference frames are always considered to be inertial.

A **positive rotation** about an axis follows the **right-hand rule**.
After these short preliminaries, let's go into detail about one of the ways to represent orientation in robotics. 

# Rotation Matrix: An Implicit Representation of the Orientation

As we saw at the beginning of this lesson, in order to know the configuration of a robot, we should know the position and orientation w.r.t to some base frame. One **implicit** way to express the **orientation** is using **rotation matrices**. 

From the DOF lesson, we learned that a rigid body in space has 3 rotational DOFS. Therefore, we need **three parameters** to **explicitly** represent a rigid body’s orientation (rotations about the x, y, and z axes). One **implicit** way to represent the orientation of a rigid body is using 3×3 rotation matrices (note that this is one of the applications of the rotation matrix) to express the orientation of the body frame relative to the base frame. With the **9-dimensional space** of the 3×3 rotation matrices subject to **6 constraints**, we can implicitly represent the 3-dimensional space of orientations. In other words, of these 9 parameters, only 3 can be chosen independently. The reason that we opt for implicit representation is to take advantage of the **algebraic calculations** on matrices. 

Suppose a robot in space as shown in the figure at the **beginning of this lesson** where {b} is the stationary body frame instantaneously attached to the moving body and {s} is the space or reference frame. We want a representation of the orientation of the robot in space.

The rotation matrix R can be defined as a representation of the **body frame unit axes** expressed in the **base frame** as:

$R = \begin{bmatrix}
\hat{x}_b & \hat{y}_b & \hat{z}_b
\end{bmatrix} = \begin{pmatrix}
\hat{x}_b.\hat{x}_s & \hat{y}_b.\hat{x}_s & \hat{z}_b.\hat{x}_s\\
\hat{x}_b.\hat{y}_s & \hat{y}_b.\hat{y}_s & \hat{z}_b.\hat{y}_s\\
\hat{x}_b.\hat{z}_s & \hat{y}_b.\hat{z}_s & \hat{z}_b.\hat{z}_s
\end{pmatrix} = \begin{pmatrix}
r_{11} & r_{12} & r_{13}\\
r_{21} & r_{22} & r_{23}\\
r_{31} & r_{32} & r_{33}
\end{pmatrix}$

The dot represents the **dot product** between the two vectors, and since the coordinate axes are of unit lengths, it represents the **cosine** of the angle between the two vectors. 

**Beginning of the Math Note:**

Recall that if a and b are two vectors with known lengths and angle between them:

<figure>
<p align="center">
<img width="224" alt="cross-prod-two-vectors" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/3694fa08-7e73-4b6b-963a-d4f7da712547">
<figcaption> <p align="center">The dot product of two vectors is the multiplication of the length of the two vectors and cosine of the angle between the two vectors.</figcaption> </p>
</p>
</figure>

Then, the dot product of the two vectors can be expressed as the **projection** of one vector onto the other multiplied by the length of the other vector (in other words, it is the multiplication of the lengths of the two vectors and cosine of the angle between the two vectors):

$`a.b = ||a||||b||cos\theta`$

**End of the Math Note.**

The nine numbers in this rotation matrix are subject to **six constraints** (so that we have 3 degrees of freedom). The constraints are:

- The **unit norm constraint** says that the columns of the rotation matrix R are unit vectors (because they are coordinate axes):

$`\begin{matrix}
||\hat{x}_b||^2 = 1 \rightarrow {r_{11}}^2 + {r_{21}}^2 + {r_{31}}^2 = 1\\
||\hat{y}_b||^2 = 1 \rightarrow {r_{12}}^2 + {r_{22}}^2 + {r_{32}}^2 = 1\\
||\hat{z}_b||^2 = 1 \rightarrow {r_{13}}^2 + {r_{23}}^2 + {r_{33}}^2 = 1
\end{matrix}`$

Note that ||.|| represents the **norm of a vector**.

- The **orthogonality condition** says that the column vectors of the rotation matrix are **orthogonal** to each other since they are coordinate axes, and thus the **dot/inner product** of any two column vectors is **zero**:

$`\hat{x}_b.\hat{y}_b = \hat{x}_b.\hat{z}_b = \hat{y}_b.\hat{z}_b = 0 \rightarrow \left\{
\begin{array}{cc}
    r_{11}r_{12} + r_{21}r_{22} + r_{31}r_{32} = 0  \\
    r_{11}r_{13} + r_{21}r_{23} + r_{31}r_{33} = 0  \\
    r_{12}r_{13} + r_{22}r_{23} + r_{32}r_{33} = 0 
\end{array}
\right.`$

We can write these six constraints in a **compact form** as $`R^T R = I`$. Where $`R^T`$ is the **transpose** of R and I is the **identity** matrix.

As we discussed in the preliminaries, all frames in robotics are taken to be **right-handed** meaning that the cross product of the x and y axes is the z-axis (and so on). But the six constraints that we just discussed do not account for this since:

$`R^T R = I \rightarrow det(R^T R) = det R^T det R = (det R)^2 = det I = 1 \rightarrow det R = \pm 1`$

If we take the right-handed coordinate system, then the determinant of R will be calculated as:

$`R = \begin{bmatrix}
\hat{x}_b & \hat{y}_b & \hat{z}_b 
\end{bmatrix} \rightarrow det R = {\hat{x}_b}^T({\hat{y}_b} \times {\hat{z}_b}) = {\hat{z}_b}^T({\hat{x}_b} \times {\hat{y}_b}) = {\hat{y}_b}^T({\hat{z}_b} \times {\hat{x}_b}) = 1`$

This result comes from the fact that our frames are right-handed. 

**Beginning of math note:**

We know from algebra that for a 3×3 matrix A defined by its columns as:

$`A = \begin{bmatrix}
x & y & z
\end{bmatrix}`$

The determinant of A can be found by the following formula: $`det A = x^T(y \times z) = z^T (x \times y) = y^T (z \times x)`$

**End of math note.**

So, we can conclude that for a right-handed frame, $`det R = 1`$, whereas the 6 constraints do not account for this, and thus we need to have another constraint that the **determinant of R is equal to 1** because our frames are right-handed. This constraint obviously **does not** reduce the degrees of freedom, and the number of independent constraints is still six. This rotation matrix can represent **spatial orientations**. 

## Special Orthogonal Group SO(3)

By definition, the **group of rotation matrices** is called the **special orthogonal group SO(3)**, which is a set of all **3×3** real matrices R that satisfy $`R^T R = I`$ and $`det R = 1`$. Elements of SO(3) are called **spatial orientations**. More generally, the space of rotation matrices in $`R^{n \times n}`$ can be defined as:

$`SO(n) = \{R \in R^{n \times n}: RR^T = I, det R = 1\}`$

A **subgroup** of SO(3) is the set of **2×2** rotation matrices called the special orthogonal group SO(2) that consists of all 2×2 real matrices R satisfying $`R^T R = I`$, and $`det R = 1`$. Based on what we have learned in the beginning of this lesson on how to calculate the rotation matrix, let's find the rotation matrix representing the planar orientations which is an element of SO(2). Let's do this with an example.

Suppose a toy car with its motion confined to the plane and two coordinate frames {s}, and {b} with their corresponding unit axes (a hat notation shows a unit vector):

<figure>
<p align="center">
<img width="680" alt="robot-motion-on-plane-toy-car" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/a11dfc90-10c4-475f-a442-50214d0f2cc1">
<figcaption> <p align="center">A toy car on a plane with two coordinate frames {s} and {b} with the unit axes. The orientation of the toy car can be expressed by finding the orientation of the body fixed-frame with respect to the space frame {s}.
</figcaption> </p>
</p>
</figure>

{b} is called a body frame since it is a fixed frame attached instantaneously to the moving body. Therefore, to find the orientation of the toy car, we should express the orientation of the body’s fixed-frame coordinates in the base frame coordinates. One way to represent the orientation of the body coordinates in terms of the base coordinates is, as we saw in this lesson, by using a rotation matrix:

$`R = \begin{bmatrix}
\hat{x}_b & \hat{y}_b\\
\end{bmatrix} = \begin{pmatrix}
\hat{x}_b. \hat{x}_s & \hat{y}_b.\hat{x}_s \\
\hat{x}_b.\hat{y}_s & \hat{y}_b.\hat{y}_s 
\end{pmatrix} = \begin{pmatrix}
cos\alpha & -sin\alpha\\
sin\alpha & cos\alpha
\end{pmatrix}_{2\times2} = \begin{pmatrix}
r_{11} & r_{12}\\
r_{21} & r_{22}
\end{pmatrix}`$

The columns of this matrix are the coordinate axes of the {b} frame expressed in the coordinate axes of the {s} frame. And since both the base frame axes and the body frame axes are unit vectors, their dot product is the cosine of the angle between the vectors. Note that $`\alpha \in [0,2\pi)`$. 

## Properties of Rotation Matrices

Sets of rotation matrices SO(2) and SO(3) are groups and thus have **properties of a mathematical group**. In general, the SO(n) groups are called **Lie groups**. A **mathematical group** has a **set of elements** and **an operation on two elements** (this operation is **matrix multiplication** for SO(n)) such that for all $`R_1`$ and $`R_2`$ in the group SO(n):

- The group has **closure property** and this means that the multiplication of the two matrices in the group also belongs to the group: $`R_1 R_2 \in SO(n)`$. 

To show this for the SO(3), suppose that $`R_1`$ and $`R_2`$ belong to SO(3), then we can write:

$`\begin{matrix}
(R_1 R_2)^T(R_1 R_2) = R^T_2 R^T_1 R_1 R_2 = I\\
det R_1 R_2 = det R_1 det R_2 = 1
\end{matrix}`$

And thus the product of two rotation matrices is also a rotation matrix.

- The group has **associativity** property: $`(R_1 R_2) R_3 = R_1 (R_2 R_3)`$

Note that the multiplication of rotation matrices is **associative** but generally not **commutative**: $`R_1 R_2 \neq R_2 R_1`$. Rotations only commute for the special case of rotation matrices in SO(2). To show this suppose that R and M are two rotation matrices in SO(2):

$`R = \begin{pmatrix}
r_{11} & r_{12}\\
r_{21} & r_{22}
\end{pmatrix}, M = \begin{pmatrix}
m_{11} & m_{12}\\
m_{21} & m_{22}
\end{pmatrix}`$

Where because of the case for planar rotations, we can write: 

$`\begin{matrix}
r_{11} = r_{22}, r_{12} = -r_{21}\\
m_{11} = m_{22}, m_{12} = m_{21}
\end{matrix}`$

Thus with an **easy calculation**, we can see that RM = MR, and thus for the special case of the planar rotations, the multiplication of the rotation matrices **commute**. For example, if you rotate a coordinate frame attached instantaneously to a body confined to a plane first by 45 degrees and then by 30 degrees, the result is the same as when you rotate it first by 30 degrees and then by 45 degrees:

![rotations in 2d plane can commute](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e98d357b-72d8-461d-b7ce-e7cd9cb2208d)

- There exists an identity element in SO(n) such that RI = IR = R.

Note that the identity matrix I can be viewed as a trivial example of a rotation matrix. Imagine this by visualizing the first frame aligned with the reference frame.

- There exists an inverse element $`R^{-1}`$ in SO(n) such that $`R R^{-1} = R^{-1} R = I`$.

For SO(3), we can say that the inverse of a rotation matrix R in SO(3) is also a rotation matrix, and $`R^{-1} = R^T`$ (this is obvious from the fact that $`R^T R = I`$). To check this, we should check that $`R^{-1}`$ satisfies the two properties of the rotation matrices:

$`\left\{
\begin{array}{cc}
    (R^{-1})^T R^{-1} = (R^T)^T R^T = R R^T = I  \\
    det R^{-1} = det R^T = det R = 1  
\end{array}
\right.`$

And thus the **inverse** of R is also a **rotation matrix**. Note that $`R R^T = I`$ because:

$`R^T R = I \rightarrow R^{-1} = R^T \rightarrow R R^{-1} = R R^T = I`$

Also, note that when a rotation matrix is applied to a vector to **rotate** it, this does **not change** the **length of the vector**. In other words: For any vector $`x \in \mathbb R^3`$ and $`R \in SO(3)`$, the vector $`y = Rx`$ has the same length as $`x`$:

Proof. $`||y||^2 = y^T y = x^T R^T R x = x^T x = ||x||^2`$

Where $`||.||`$ is the norm of the vector.

## Uses of Rotation Matrices

A rotation matrix can be used for three different purposes that we will discuss in the coming paragraphs.

### Implicit Representation of the Orientation

As we discussed earlier, a rotation matrix can be used to implicitly represent an orientation; in other words, it can be used to represent the **orientation** of the **body frame** relative to the **base frame**. In order to show this, I use an example that is adapted from the reference book “Modern Robotics: Mechanics, Planning, and Control” by Kevin Lynch and Frank Park. 

Suppose we have three reference frames {s}, {b}, and {c} that represent the same space with different orientations:

<figure>
<p align="center">
<img width="865" alt="rotation-matrix-orientations" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b665fd0c-702a-4ef2-996c-f2407acecf11">
<figcaption> <p align="center">One of the applications of a rotation matrix is to represent the orientation of one frame relative to another. Coordinate frames {s}, {b}, and {c} represent the same space with different orientations. The orientation of frames {b} and {c} relative to {s} can be represented by a rotation matrix. Point p has the same location in space but has different coordinates depending on the choice of coordinate frame. Note that the three reference frames have the same origin and only orientations are different. I showed them in different spots to make the presentation clear. In other words, the same space is drawn three times.</figcaption> </p>
</p>
</figure>

Coordinate frame {b} can be achieved by rotating the {s} frame by 90 degrees about the {s} frame’s z-axis, and frame {c} is achieved by rotating the {b} frame by -90 degrees about the {b} frame’s y-axis. Watch the short demonstration below to understand how these coordinate frames are achieved:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/0be3e81d-0e67-4d40-9134-b7afcd3d70e7
<figcaption> <p align="center">Three coordinate frames {s}, {b}, and {c} representing the same space with different orientations. A rotation matrix is used to represent the orientation of one frame relative to another frame.</figcaption> </p>

As we discussed, a rotation matrix can be used to represent the orientation of one frame relative to the base frame. Now, let’s see the rotation matrices representing the orientations of the {b} and {c} coordinate frames relative to the {s} frame. By writing the coordinate axes of the {b} and {c} frames in the {s} frame we can get:

$`\begin{matrix}
R_{sb} = \begin{pmatrix}
\hat{x}_b.\hat{x}_s & \hat{y}_b.\hat{x}_s & \hat{z}_b.\hat{x}_s\\
\hat{x}_b.\hat{y}_s & \hat{y}_b.\hat{y}_s & \hat{z}_b.\hat{y}_s\\
\hat{x}_b.\hat{z}_s & \hat{y}_b.\hat{z}_s & \hat{z}_b.\hat{z}_s
\end{pmatrix} = \begin{pmatrix}
0 & -1 & 0\\
1 & 0 & 0\\
0 & 0 & 1
\end{pmatrix}\\
R_{sc} = \begin{pmatrix}
\hat{x}_c.\hat{x}_s & \hat{y}_c.\hat{x}_s & \hat{z}_c.\hat{x}_s\\
\hat{x}_c.\hat{y}_s & \hat{y}_c.\hat{y}_s & \hat{z}_c.\hat{y}_s\\
\hat{x}_c.\hat{z}_s & \hat{y}_c.\hat{z}_s & \hat{z}_c.\hat{z}_s
\end{pmatrix} = \begin{pmatrix}
0 & -1 & 0\\
0 & 0 & -1\\
1 & 0 & 0
\end{pmatrix}
\end{matrix}`$

$`R_{sb}`$ and $`R_{sc}`$ represent the orientations of the {b} and {c} frames relative to the {s} frame, respectively.

It’s easy to note that the coordinate axes of the {s} frame in the for example {c} frame can be achieved by inverting (or transposing in the case of rotation matrices) the rotation matrix representing the orientation of the {c} frame relative to the {s} frame:

$`R_{cs} = {R^T_{sc}} = {R^{-1}_{sc}} = \begin{pmatrix}
0 & 0 & 1\\
-1 & 0 & 0\\
0 & -1 & 0
\end{pmatrix}`$

Note that because of the properties of a rotation matrix, the **inverse** of a rotation matrix is equal to the **transpose** of a rotation matrix.

Also, note that the point p has the same location in the space but has different representations depending on the choice of the coordinate frame:

$`p_s = \begin{pmatrix}
1 \\
1\\
0 
\end{pmatrix}, p_b = \begin{pmatrix}
1 \\
-1\\
0 
\end{pmatrix}, p_c = \begin{pmatrix}
0 \\
-1\\
-1
\end{pmatrix}`$

### Changing the Reference Frame of a Vector or a Frame (Rotation Matrix is an Operator)

One of the applications of a rotation matrix is to **change the reference frame** of a vector or a frame. Here, a rotation matrix is an operator that acts on a vector or a frame to change its reference frame. For example, to show the change of frame of reference for a coordinate frame, suppose that we have the rotation matrix representing the orientation of the {c} frame relative to the {b} frame as (frames are defined above):

$`R_{bc} = \begin{pmatrix}
\hat{x}_c.\hat{x}_b & \hat{y}_c.\hat{x}_b & \hat{z}_c.\hat{x}_b\\
\hat{x}_c.\hat{y}_b & \hat{y}_c.\hat{y}_b & \hat{z}_c.\hat{y}_b\\
\hat{x}_c.\hat{z}_b & \hat{y}_c.\hat{z}_b & \hat{z}_c.\hat{z}_b
\end{pmatrix} = \begin{pmatrix}
0 & 0 & -1\\
0 & 1 & 0\\
1 & 0 & 0
\end{pmatrix}`$

The goal is to express the {c} frame in {s} frame coordinates instead of the {b} coordinates. To achieve this goal, we can use the rotation matrix $`R_{sb}`$ as a math operator that changes the reference frame from {b} to {s} and then pre-multiply $`R_{bc}`$ by this math operator to get $`R_{sc}`$:  

$`R_{sc} = R_{sb} R_{bc} = \begin{pmatrix}
0 & -1 & 0\\
1 & 0 & 0\\
0 & 0 & 1
\end{pmatrix}\begin{pmatrix}
0 & 0 & -1\\
0 & 1 & 0\\
1 & 0 & 0
\end{pmatrix} = \begin{pmatrix}
0 & -1 & 0\\
0 & 0 & -1\\
1 & 0 & 0
\end{pmatrix}`$ 

By pre-multiplying $`R_{bc}`$ by $`R_{sb}`$, we changed the representation of the {c} frame from the {b} frame to the {s} frame. Note that the **subscript cancellation** rule says that the second frame of the first subscript should be the same as the first frame of the second subscript (both are {b}).

A rotation matrix can also serve as an **operator** to **change the frame of reference** for a **vector**. For instance, suppose that we have $`p_b`$ that represents the position of point p expressed in {b} frame coordinates as the following figure:

<figure>
<p align="center">
<img width="536" alt="rotation-matrix-point-p-in-two-coordinate-frames" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/ed530141-9755-40c9-8f69-8acbdf8aaebb">
<figcaption> <p align="center">A rotation matrix can serve as an operator to change the reference frame in which a vector is expressed.</figcaption> </p>
</p>
</figure>

We want to express p in {s} frame coordinates. We can do this by pre-multiplying $`p_b`$ by $`R_{sb}`$ to get the point coordinates in the {s} frame:

$`p_{b} = \begin{pmatrix}
-1\\
0\\
0
\end{pmatrix} \rightarrow p_s = R_{sb}p_b = \begin{pmatrix}
0 & -1 & 0\\
1 & 0 & 0\\
0 & 0 & 1
\end{pmatrix}\begin{pmatrix}
-1\\
0\\
0
\end{pmatrix} = \begin{pmatrix}
0\\
-1\\
0
\end{pmatrix}`$

Note that the **subscript cancellation** rule works here too.

### Rotating a Vector or a Frame (Rotation matrix is an operator)

Another application (and the final one) of a rotation matrix is to rotate a vector or a frame. Here again, a rotation matrix is an operator that acts on a vector or a frame to rotate it.

Consider the three coordinate axes {s}, {b}, and {c} described as in the above figure. As we saw before, {b} is achieved by rotating the {s} frame by 90 degrees about the z-axis of the {s} frame. $`R_{sb}`$ here can also serve as an operator that can rotate a vector or a frame by 90 degrees about the z-axis: $`R_{sb} = R = Rot (\hat{z}, 90^{o})`$.

Pre-multiplying $`p_s`$ by $`R_{sb}`$ does **not follow** the **subscript cancellation rule** and thus the rotation matrix **does not** serve as the operator to **change the reference frame** but rather it acts as an operator to **rotate the vector** $`p_s`$ by 90 degrees about the z-axis of the {s} frame: $`p'_s = Rp_s`$. 

This vector represents the rotated $`p_s`$ vector by 90 degrees about the z-axis of the {s} frame. **The vector is rotated but it is still represented in the original frame {s}**:

![rotation-matrix-vector-rotation-same-frame](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/cfdebfc2-b0eb-4140-bdf9-8681552b379c)
<figcaption> <p align="center">A rotation matrix can act as an operator to rotate a vector. For example, in this simulation, you can see that by multiplying a vector by a rotation operator that can rotate 90 degrees about the z-axis, the vector is rotated in the same coordinate frame by 90 degrees.</figcaption> </p>

Note that to rotate a vector $`v`$, there is only one frame involved and that is the frame in which $`v`$ is represented and the axis of rotation is interpreted in this frame: $`v' = Rv`$. $`v’`$ is the rotated vector $`v`$ in the same frame.

A rotation matrix can also be used to rotate a frame. Suppose that we have the rotation operator R that can rotate a frame by 90 degrees about the z-axis: $`R = Rot(\hat{z},90^{o})`$. 

Now let’s see what happens to a frame if we **pre-multiply or post-multiply** it by this rotation operator. Note that the choice of the z-axis to be from which frame depends on the pre-multiplication or post-multiplication of the rotation matrix. Now suppose the {s} and {c} coordinate frames that we had before:

<figure>
<p align="center">
<img width="640" alt="rotation-matrix-rotating-frames" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/cb332db4-8e9a-47de-b8c3-1ec1df10cb47">
</p>
</figure>

The {c} frame orientation will be **different** depending on whether the rotation matrix representing its orientation with respect to the base frame ($`R_{sc}`$) is **pre-multiplied** or **post-multiplied** by the rotation operator R.

If we pre-multiply by the rotation operator R defined above, then the rotation axis is the z-axis of the first subscript, which is s so that the rotation is about the z-axis of the {s} coordinate frame:

<figure>
<p align="center">
<img width="681" alt="rotation-matrix-operator-pre-multiplying-rotating-frame-1" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/1e4210e7-2a06-4c78-ab86-6809feaa084d">
<figcaption> <p align="center">Pre-multiplication of a rotation matrix representing the orientation of one frame with respect to another by a rotation operator defined above (a rotation operator that can rotate a vector or a frame by 90 degrees about the z-axis), rotates the frame about the z-axis of the coordinate frame related to the first letter of the subscript which is s.</figcaption> </p>
</p>
</figure>

If we **post-multiply** by the rotation operator R defined above, then the **rotation axis** is the z-axis of the **second subscript**, which is c so that the rotation is about the z-axis of the {c} coordinate frame:

<figure>
<p align="center">
<img width="741" alt="rotation-matrix-operator-post-multiplying-rotating-frame-1" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/2aa1a1f1-0a3e-43ce-9c27-60513562d603">
<figcaption> <p align="center">Post-multiplication of a rotation matrix representing the orientation of one frame with respect to another by a rotation operator defined above (a rotation operator that can rotate a vector or a frame by 90 degrees about the z-axis), rotates the frame around the z-axis of the coordinate frame related to the second letter of the subscript which is c.</figcaption> </p>
</p>
</figure>

Generally speaking, if $`R_{sb}`$ represents the orientation of some frame {b} w.r.t the base frame {s}, and if we want to rotate {b} by $`\theta`$ about a unit axis $`\hat{\omega}`$, in other words, by a rotation operator $`R = Rot(\hat{\omega}, \theta)`$, then:

- Coordinate frame {b’} is the new frame after a rotation by $`\theta`$ about $`\hat{\omega}_s = \hat{\omega}`$, and this means that the **rotation axis** is considered to be in the **fixed frame** {s}: $`R_{sb'} = R R_{sb}`$. 

- Coordinate frame {b”} is the new frame after a rotation by $`\theta`$ about $`\hat{\omega}_b = \hat{\omega}`$, and this means that the rotation axis is in the body frame {b}: $`R_{sb"} = R_{sb} R`$. 

So, for the rotation operator $`R = Rot(\hat{\omega}, \theta)`$, $`\hat{\omega}`$ can be either in {s} frame or {b} frame based on pre-multiplying by R or post-multiplying by R. As we just saw, if we pre-multiply by R, then the rotation is about an axis $`\hat{\omega}`$ in the fixed frame, and if we post-multiply by R, then the rotation is about $`\hat{\omega}`$ in the body frame. 

The **multiplication** of $`\hat{\omega}`$ and $`\theta`$ is a **3-vector representation** of the **orientation** that is called **exponential coordinates** of orientation that is an **explicit representation** and will be discussed in the coming lessons.

**Rotation Operators about the x, y, and z Axes**

Now let’s find general forms of **rotation operators** representing the rotations about the x, y, and z axes by $`\theta`$ degrees. 

The rotation operator representing the rotation about the x-axis by $`\theta`$ degrees can be visualized as:

![rotation operator about x](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d6f1c459-e43e-4844-a991-6db21f9e126b)

Using the figure below:

<figure>
<p align="center">
<img width="414" alt="rotation-about-x-axis-figure" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/887e6348-d162-4617-bb56-a67579a8e1f5">
</p>
</figure>

We can find the rotation operator representing the rotation about the x-axis as:

$`Rot(\hat{x},\theta) = \begin{pmatrix}
\hat{x}'.\hat{x} & \hat{y}'.\hat{x} & \hat{z}'.\hat{x}\\
\hat{x}'.\hat{y} & \hat{y}'.\hat{y} & \hat{z}'.\hat{y}\\
\hat{x}'.\hat{z} & \hat{y}'.\hat{z} & \hat{z}'.\hat{z}
\end{pmatrix} = \begin{pmatrix}
1 & 0 & 0\\
0 & cos\theta & -sin\theta\\
0 & sin\theta & cos\theta
\end{pmatrix}`$

With the same approach rotation operators representing the rotations about the y-axis and the z-axis can be found as:

$`Rot(\hat{y},\theta) = \begin{pmatrix}
cos\theta & 0 & sin\theta\\
0 & 1 & 0\\
-sin\theta & 0 & cos\theta
\end{pmatrix}`$

and 

$`Rot(\hat{z},\theta) = \begin{pmatrix}
cos\theta & -sin\theta & 0\\
sin\theta & cos\theta & 0\\
0 & 0 & 1
\end{pmatrix}`$

**Generally**, the rotation about an arbitrary unit axis $`\hat{\omega}`$: $`\hat{\omega} = (\hat{\omega}_1,\hat{\omega}_2,\hat{\omega}_3)`$

by $`\theta`$ can be represented by a rotation operator as:

$`Rot(\hat{\omega},\theta) = \begin{pmatrix}
c_{\theta} + {\hat{\omega}_1}^2 (1-c_{\theta}) & \hat{\omega}_1\hat{\omega}_2(1-c_{\theta})-\hat{\omega}_3 s_{\theta} & \hat{\omega}_1 \hat{\omega}_3 (1-c_{\theta}) + \hat{\omega}_2 s_{\theta} \\
\hat{\omega}_1\hat{\omega}_2(1-c_{\theta}) + \hat{\omega}_3 s_{\theta} & c_{\theta} + {\hat{\omega}_2}^2(1-c_{\theta}) & \hat{\omega}_2\hat{\omega}_3(1-c_{\theta})-\hat{\omega}_1 s_{\theta}\\
\hat{\omega}_1\hat{\omega}_3(1-c_{\theta})-\hat{\omega}_2 s_{\theta} & \hat{\omega}_2\hat{\omega}_3(1-c_{\theta}) + \hat{\omega}_1 s_{\theta} & c_{\theta} + {\hat{\omega}_3}^2(1-c_{\theta})
\end{pmatrix}`$

Where: $`s_{\theta} = sin\theta, c_{\theta} = cos\theta`$.

We will see how to derive this rotation matrix in the coming lessons when talking about the **exponential coordinates** of rotation. For now, you can verify that it is correct by assuming $`\hat{\omega} = (1,0,0), (0,1,0), and (0,0,1)`$ for rotations about x, y, and z, respectively, and get rotation matrices that we calculated above.

Visualize this rotation as the following figure:

<figure>
<p align="center">
<img width="388" alt="rotation-about-arbitrary-axis" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/a36fe323-b5b9-4bb5-b795-4816d0f49b40">
<figcaption> <p align="center">The rotation about the arbitrary axis ῶ by θ.</figcaption> </p>
</p>
</figure>

Note that any $`R \in SO(3)`$ can be obtained by rotating from the identity matrix by some $`\theta`$ about some $`\hat{\omega}`$. Also, note that: $`Rot(\hat{\omega},\theta) = Rot(-\hat{\omega},-\theta)`$. In other words, rotation about $`-\hat{\omega}`$ by $`-\theta`$ is the same as the rotation about $`\hat{\omega}`$ by $`\theta`$. 

Now, let's finish up this lesson with an example. 

**Example: The Representation of the Orientation of the Elements in the Robot’s Workspace**

Suppose that a camera and a gripper are attached to the end-effector of the industrial arm. The camera is used to observe the workpiece and position the end-effector in the right position, and the gripper is used to grip the workpiece. The overall system can be depicted in the figure below:

![robot-arm-camera-endeffector-workpiece-rotation-matrix](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/5dd2a404-6099-4179-9d80-0f096fdc98c8)
<figcaption> <p align="center">A camera is attached to the robot end-effector to observe the object and position the end-effector in the right position. Four reference frames are attached to different elements in the robot’s workspace, as shown in the figure. The orientation of one frame relative to the other can be implicitly represented by a rotation matrix. This example is adapted from the first textbook by Lynch et al.</figcaption> </p>

Four frames are attached to different elements in the robot’s workspace, as shown above. {a} is the frame coincident with the space frame {s}, {b} is the gripper frame, {c} is the camera frame, and {d} is the workpiece frame.

The orientation of the workpiece frame relative to the base frame can be expressed by the rotation matrix $`R_{ad}`$ as:

$`R_{ad} = \begin{pmatrix}
{\hat{x}}_d.{\hat{x}}_a & {\hat{y}}_d.{\hat{x}}_a & {\hat{z}}_d.{\hat{x}}_a\\
{\hat{x}}_d.{\hat{y}}_a & {\hat{y}}_d.{\hat{y}}_a & {\hat{z}}_d.{\hat{y}}_a\\
{\hat{x}}_d.{\hat{z}}_a & {\hat{y}}_d.{\hat{z}}_a & {\hat{z}}_d.{\hat{z}}_a
\end{pmatrix} = \begin{pmatrix}
1 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 1
\end{pmatrix}`$

This means that the two frames have the same orientation.

The orientation of the workpiece frame relative to the camera frame can be calculated as follows:

$`R_{cd} = \begin{pmatrix}
{\hat{x}}_d.{\hat{x}}_c & {\hat{y}}_d.{\hat{x}}_c & {\hat{z}}_d.{\hat{x}}_c\\
{\hat{x}}_d.{\hat{y}}_c & {\hat{y}}_d.{\hat{y}}_c & {\hat{z}}_d.{\hat{y}}_c\\
{\hat{x}}_d.{\hat{z}}_c & {\hat{y}}_d.{\hat{z}}_c & {\hat{z}}_d.{\hat{z}}_c
\end{pmatrix} = \begin{pmatrix}
0 & 1 & 0\\
1 & 0 & 0\\
0 & 0 & -1
\end{pmatrix}`$

Now suppose we have the rotation matrix representing the orientation of the camera frame relative to the gripper frame as:

$`R_{bc} = \begin{pmatrix}
1 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 1
\end{pmatrix}`$

To calculate the orientation of the gripper frame relative to the space frame $`R_{ab}`$, we can do so by changing the reference frame using consecutive rotation matrices:

$`R_{ab} = R_{ad}R_{dc}R_{cb} = R_{ad} {R^T_{cd}} {R^T_{bc}} = \begin{pmatrix}
1 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 1
\end{pmatrix}\begin{pmatrix}
0 & 1 & 0\\
1 & 0 & 0\\
0 & 0 & -1
\end{pmatrix}\begin{pmatrix}
1 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 1
\end{pmatrix} = \begin{pmatrix}
0 & 1 & 0\\
1 & 0 & 0\\
0 & 0 & -1
\end{pmatrix}`$

Note that we used the inverse of a rotation matrix equals its transpose and the subscript cancellation rule to calculate the result.

Let's see another example.

**Example: Successive Rotations of a Point about the Coordinate Axes of the Base Frame**

Suppose p is a point in space with coordinates relative to the space frame as $`p_s = (5,-7,10)`$. As we saw at the beginning of this lesson, we can represent a **point in space** with a **vector**. The point p and the corresponding vector representation can be visualized as the following figure:

<figure>
<p align="center">
<img width="514" alt="point-in-space-subject-to-different-rotations-rotation-matrix-operator-1" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c97e9bd6-8a65-4223-84ef-568d3fc98022">
<figcaption> <p align="center">point p in space and the corresponding vector representation in {s} frame.</figcaption> </p>
</p>
</figure>

Now suppose that p is rotated about the fixed-frame x-axis by 30 degrees, then about the fixed-frame y-axis by 135 degrees, and finally about the fixed-frame z-axis by -120 degrees. The rotation matrix that rotated p to the new location can be expressed as (note the order in which the rotations are written): 

$`R = Rot(\hat{z},-120)Rot(\hat{y},135)Rot(\hat{x},30)`$. 

These rotations can easily be calculated using the rotation operators about the coordinate axes provided above. Then the coordinates of the rotated point can be calculated as:

$`p'_s = R p_s = \begin{pmatrix}
0.3536 & 0.5732 & -0.7392\\
0.6124 & -0.7392 & -0.2803\\
-0.7071 & -0.3536 & -0.6124
\end{pmatrix}\begin{pmatrix}
5\\
-7\\
10
\end{pmatrix} = \begin{pmatrix}
-9.6364\\
5.4334\\
-7.1843
\end{pmatrix}`$

The rotated point can be visualized as:

<figure>
<p align="center">
<img width="514" alt="point-in-space-subject-to-different-rotations-rotation-matrix-operator-rotated-point" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d4a37e2c-5c67-4abb-a5be-7edf518b8173">
<figcaption> <p align="center">Vector representation of the rotated point p after going through a rotation by 30 degree about the x-axis of the base frame, then a rotation by 135 degree about the y-axis of the base frame, and finally a rotation by -120 degree about the z-axis of the base frame.</figcaption> </p>
</p>
</figure>

## References

- Modern Robotics: Mechanics, Planning, and Control by Frank Park and Kevin Lynch
- A Mathematical Introduction to Robotic Manipulation by Murray, Lee, and Sastry 
- Cao, C.T., Do, V.P. and Lee, B.R., 2019. A novel indirect calibration approach for robot positioning error compensation based on neural network and hand-eye vision. Applied Sciences, 9(9), p.1940.

## Resources

If you want to strengthen your math foundation, the following course is recommended (especially the Matrix Algebra for Engineers course):

- https://www.coursera.org/specializations/mathematics-engineers

Statistics is very important if you want to do a lot of Machine Learning:

- https://www.udacity.com/course/statistics--st095?irclickid=VqE0xSSAAxyNTz%3ARtMS-yR5NUkAX-SzFrRySUQ0&irgwc

Learn physics from the lectures below:

- https://www.youtube.com/@lecturesbywalterlewin.they9259

Especially go to playlists and start from Classical Mechanics. The textbook that he uses is Physics for Engineers by Ohanian. 