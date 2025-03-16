## Introduction

Up to now, we have become familiar with [forward](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-7:-Forward-Kinematics-of-Robot-Arms-Using-Screw-Theory) and [velocity kinematics](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-9:-Robot-Velocity-Kinematics-Using-Screw-Theory-%E2%80%90-Jacobian-Matrix). We saw that forward kinematics calculates the position and orientation of the robot end-effector from the joint coordinates q (FK: $`\text{given }q, \text{find } T(q) \in SE(3)`$). In this lesson, we are interested in the opposite problem: finding **joint variables** that produce a **desired end-effector configuration**. This problem is called **inverse kinematics**. 

<figure>
<p align="center">
  <img width="500" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/7f7de603-e758-4e99-9ce3-1c45602ed844">
</p>
</figure>

Formally, the inverse kinematics problem can be stated as follows: 

Given an $`n`$ degree of freedom open chain, with forward kinematics represented by the variable homogeneous transformation $`T(q)`$, where $`q ∈ R^{n}`$, it is required to find a solution $`q`$ that satisfies $`T(q) = X`$, where $`X`$ is a **desired** (and given) homogenous transformation and $`X ∈ SE(3)`$ (IK: $`\text{given }X \in SE(3), \text{find } q \text{ such that }T(q) = X`$).

Inverse kinematics is a very important problem in the sense that here, we want to control the end-effector's configuration for it to be able to interact with the world. However, it is a more complicated problem to solve than the forward kinematics. In forward kinematics, we will have a **unique end-effector configuration** for a **given set of joint values**, but the inverse kinematics problem can have **zero**, **one**, or **multiple solutions** for the joint values q that can produce the given desired end-effector configuration.  

There are two approaches to solving the inverse kinematics of an open-chain robot:

- **Analytic approach** to inverse kinematics in which closed-form solutions for the joint variables can be found. Here, geometric approaches to the problem are utilized. These analytic solutions may not always exist. 
- **Iterative numerical** approach to inverse kinematics in which **Jacobian** is used to iteratively find a solution using the **Newton-Raphson** method. An **initial guess** for the solution should be made, and then this method **iteratively** pushes the initial guess towards a solution. This approach will only give us **one solution** and not all possible solutions. 

As always, let's begin our discussion with a simple example. 

## Example: Inverse Kinematics of Two-link Planar Robot Arm Using Analytic Approach

Consider the 2R planar robot arm depicted in the figure below:

<figure>
<p align="center">
  <img width="453" alt="2dof-planar-no-background with coordinates" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/9ed21b25-7f02-4cc4-83f5-bc5d0ca5417a">
</p>
</figure>

We want to determine the **set of joint angles** that can produce a **desired end-effector position** (x,y). 

In order to first geometrically solve this and find an analytic solution, let's first intuitively take a look at the stated problem and possible solutions.

We first start by looking into the workspace of this 2R robot arm. Just a reminder that the **workspace** of a robot is a specification of the **reachable configurations** of the end-effector. If you are not familiar with the concept of workspace and its difference with the configuration space, you can read the lessons below first:

https://mecharithm.com/learning/lesson/task-space-and-workspace-for-robots-102
https://mecharithm.com/learning/lesson/configuration-space-topology-representation-robot-5

As you see in the simulation video below, the workspace of a 2R robot depends on the lengths of the links of the robot arm. If $`L_1 = L_2`$, then the workspace is all the points in and on the circle of radius $`L_1 + L_2`$, if $`L_1 > L_2`$, then the workspace of this robot is an **annulus** of inner radius of $`L_1 - L_2`$, and if $`L_1 < L_2`$, then the workspace is an annulus of inner radius of $`L_2 - L_1`$ (the outer radius in both cases is $`L_1 + L_2`$):

https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/9f94fa08-fb09-4447-93ea-8f811c7f3edc

For now, let's assume that $`L_1 > L_2`$, then as we saw in the simulation, the workspace is an annulus. If the desired position of the end-effector is (x,y), then it is pretty easy to see that the inverse kinematics solution has **zero**, **one**, or **two solutions** depending on where the (x,y) is located on the annulus. If (x,y) is on the **boundary** of the annulus, then the inverse kinematics has **one solution**, no matter the elbow up or elbow down. If (x,y) is in the blue region of the annulus in the simulation below, then there will be **two sets of joint angles** that can provide us with the **desired position** (the **elbow up** (second joint angle is negative) or **elbow down** (second joint angle is positive) solutions), and thus, the inverse kinematics has **two solutions**. If (x,y) lies in the white region of the annulus, then as it is evident, there are **no sets of joint angles** that can give us this desired position. See the simulation below for all these cases:

https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/6ade0401-aa91-465a-a734-9fd655bc1762

Let's now find a set of joint angles $`(\theta_1,\theta_2)`$ for the desired position (x,y) using a geometric approach:

<figure>
<p align="center">
  <img width="453" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/47e65f9d-52f2-438e-9e6d-268f4e72dab1">
</p>
</figure>

From the figure above and using the **law of cosines**, we can write:

$`{L_1}^2 + {L_2}^2 - 2L_1L_2cos\beta = x^2 + y^2 \rightarrow \beta = cos^{-1}(\frac{{L_1}^2 + {L_2}^2-x^2-y^2}{2L_1L_2})`$ 

Also, by using the law of cosines again on another triangle, we can get a similar equation for $`\alpha`$:

$`{L_2}^2 = {L_1}^2 + x^2 + y^2 -2L_1\sqrt{x^2+y^2}cos\alpha \rightarrow \alpha = cos^{-1}(\frac{x^2+y^2+{L_1}^2 - {L_2}^2}{2L_1\sqrt{x^2+y^2}})`$ 

The angle $`\gamma`$ can be determined using the four-quadrant arctangent as: $`\gamma = atan2(y,x)`$. 

**Start of the note on atan2(y,x)**

atan2(y,x) is a **two-argument function** that is similar to **arctangent**, but the difference is that since $`tan^{-1}(\frac{y}{x}) = tan^{-1}(\frac{-y}{-x})`$ then $`tan^{-1}`$ only returns angles in the range (-π/2,π/2) but atan2(y,x) returns angles in the range (-π,π) and thus it is called a **four-quadrant arctangent**. In other words, arctangent does not differentiate between angles in the first and third quadrants as well as angles in the second and fourth quadrants (the first and the third quadrants are shown as an example), and in robotics, we prefer the four-quadrant arctangent to make the quadrant in which the angle lies clear.

<figure>
<p align="center">
<img width="514" alt="arctangent-first-third-quadrant-same" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/975a91d1-5ca9-458a-9e51-731cd1c2b1d5">
<figcaption> <p align="center">Arctangent does not differentiate between angles in the first and third quadrants as well as angles in the second and fourth quadrants (the first and the third quadrants are shown as an example), and in robotics, we prefer the four-quadrant arctangent to make the quadrant in which the angle lies clear.
</figcaption> </p>
</p>
</figure>

**End of the note on atan2(y,x)**

With these angles, the **elbow down** solution is:

$`\theta_1 = \gamma - \alpha, \quad \theta_2 = \pi - \beta`$

<figure>
<p align="center">
<img width="475" alt="elbow down solution" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/915bb72b-3250-4fcb-8120-a3c179fde179">
</p>
</figure>

and the **elbow up** solution is:

$`\theta_1 = \gamma + \alpha, \quad \theta_2 = \beta - \pi`$

<figure>
<p align="center">
<img width="474" alt="elbow up solution" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/4087fecd-e571-48cd-8151-c7407fc89e42">
</p>
</figure>

Note that as we discussed earlier, if (x,y) is in the white region of the annulus, then no solution exists, and there is no set of joint angles that can provide the desired position. Furthermore, for the case where (x,y) is at the boundary of the workspace, $`\alpha = 0, \beta = \pi \text{ or } 0`$ (depending on if the desired position is on the outer circle boundary or inner circle boundary), then the inverse kinematics problem only has one solution: $`\theta_1 = atan2(y,x), \theta_2 = 0 \text{ or } \pi`$.  

**Question:** Considering the inverse kinematics of the 2R planar robot arm, how many solutions do you think exist for the inverse kinematics problem of the 3R planar robot arm? 

## Numerical Inverse Kinematics Using Newton-Raphson Iterative Method

If the inverse kinematics equations cannot be solved analytically, **iterative numerical techniques** may be utilized. Moreover, numerical methods are frequently employed to enhance the precision of analytic solutions, even when such solutions are available. In this case, the analytic solution can be used as an **initial guess** for the iterative numerical approach. For this purpose, we will use **Newton–Raphson** method, which is an integral method in **nonlinear root finding**. In situations where there is **no exact solution exists**, we will need **optimization methods** to find the **closest approximate solution**. If the manipulator is **redundant**, there will be **infinite solutions** to the inverse kinematics problem. In this case, we need to find a solution that is **optimal** with respect to some criterion.

Now, let's start by providing a general algorithm for **numerical inverse kinematics** using the Newton-Raphson method. Note that I will not go over the details behind the Newton-Raphson method, as most engineering students have encountered it in their **numerical analysis** courses, and I will directly go to its application to find the **numerical inverse kinematics** for open chain robot arms. 

Suppose that $`T_{sb}`$ is the **pose of the end-effector frame in the base frame** (calculated from the forward kinematics) and the **desired end-effector configuration** is given by the transformation matrix $`T_{sd}`$:

<figure>
<p align="center">
<img width="474" alt="space body and desired frames" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/8abf6dd7-521e-4d7e-b8cf-06410855b628">
</p>
</figure>

Solving the inverse kinematics means that we need to find the **set of joint angles** that can take the end-effector frame {b} to the desired frame {d}. Before solving this problem, let's first take a brief look at the **Newton-Raphson method** for **nonlinear root finding**.  

### Brief Overview of Newton-Raphson Method for Nonlinear Root Finding 

The **Newton-Raphson method** is a powerful technique for finding approximate solutions (roots) of a nonlinear equation of the form $`f(x) = 0`$. In other words, you want to find a value for $`x`$ where $`f(x)`$ becomes zero. Here are the summary of steps to iteratively find the solution to this problem:

**Step 1**. Start by selecting an **initial guess** $`x_0`$. This can be any value, but it's better to choose a value close to the actual root for faster convergence.

**Step 2**. Find the tangent line: At the current guess $`x_k`$, find the equation of the tangent line to the curve $`f(x)`$ at that point. We can easily find the equation of the tangent line as:

$`y = f(x_k) + f'(x_k) (x - x_k)`$

Where, $`f(x_k)`$ is the value of the function at $`x_k`$, and $`f'(x_k)`$ is the derivative of the function at $`x_k`$. 

**Step 3**. Now, we should solve for the next approximation. To find the next approximation, $`x_{k+1}`$, we should set $`y = 0`$ (since we're looking for where the tangent line crosses the x-axis):

$`0 = f(x_k) + f'(x_k) (x_{k+1} - x_k)`$

From here, solve for $`x_{k+1}`$:

$`x_{k+1} = x_k - \frac{f(x_k)}{f'(x_k)}`$

This is the **core equation** of the **Newton-Raphson** method. It calculates the **next approximation** based on the current one and the function's value and derivative at that point:

<figure>
<p align="center">
<img width="474" alt="newton_raphson_one_D_case_illustration" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/357571ec-3eb7-46e3-b933-f5b9d9716af1">
</p>
</figure>


**Step 4**. Repeat steps 2 and 3 until you achieve the desired level of accuracy or until $`f(x_{k+1})`$ is sufficiently close to zero. In each iteration, you'll get a new and better approximation of the root.

**Step 5**. Now, we should check for **convergence**. Monitor the convergence of the method by calculating the absolute error at each iteration: 

$`|x_{k+1} - x_k|`$ 

If this value becomes **very small**, you can **stop the iterations** since you've likely found a **good approximation** of the **root**.

As a very simple example, consider $`f(x) = x^2 - 2`$ that we want to use the Newton-Raphson method for its root-finding. Following the above algorithm, the root of this function can simply be found as the animation below shows:

https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/5e0e551c-94dd-4553-b6c7-7b51cc301379

The above algorithm was for when f is a function that maps real numbers to real numbers (1D case): $`f: \mathbb{R} \rightarrow \mathbb{R}`$. The same formula applies when f is **multi-dimensional**, that is: $`f: \mathbb{R}^n \rightarrow \mathbb{R}^n`$. Here, f is a vector function; therefore, we should extend the concept from scalar to vector functions. What do we do for the vector functions? You guessed it right. We should **replace** the **derivative** with the **Jacobian matrix**. So, the updated formula in the multidimensional case can be written as:

$`x_{k+1} = x_k - {J^{-1}(x_k)} f(x_k)`$

Here $`x_k`$, and $`x_{k+1}`$ are vectors in $`\mathbb{R}^n`$, and $`{J^{-1}(x_k)}`$ is the inverse of the Jacobian matrix evaluated at $`x_k`$. The Jacobian matrix is a $`n \times n`$ matrix where each element $`J_{ij}`$ is the partial derivative $`\frac{\partial f_i}{\partial x_j}`$. 

**Summary:**

- Start with an **initial guess** $`x_0 \in \mathbb{R}^n`$. 
- Apply the above **update formula** iteratively to find $`x_{k+1}`$.
- Continue until a **convergence** criterion is met, e.g., when the norm $`||f(x_{k+1})||`$ is less than a specified tolerance or when the next value is so close to the previous value. 

**Notes:**

- The method requires that the Jacobian is invertible at each iteration.
- Convergence is not guaranteed and can be sensitive to the initial guess.

Now let's see how we can apply this algorithm to solve **inverse kinematics** in robotics. 

### Newton-Raphson Method to Solve Inverse Kinematics Numerically

Applying the **Newton-Raphson** method to solve **inverse kinematics** problem numerically involves using the method to find **joint variables** (angles or positions) that result in a **desired end-effector position or orientation**. 

Let's first start with the coordinate **vector** case. Suppose, we express the end-effector frame using a **coordinate vector** $`x`$ that is related to joint angles using the forward kinematics: $`x = f(q)`$. Here f is a nonlinear vector function that maps the n joint coordinates to the m end-effector coordinates ($`f: \mathbb{R}^n \rightarrow \mathbb{R}^m`$). We assume that the function f is differentiable and $`x_d`$ is the **desired** end-effector coordinates. The goal is to find $`q_d`$ such that $`f(q_d) = x_d`$. This is essentially a **root-finding** problem for the function $`g(q) = f(q) - x_d`$, where we seek for $`q_d`$ such that $`g(q_d) = f(q_d) - x_d = 0`$. This can serve as the **error function**, and our goal is to minimize the error until it reaches to zero. Now, applying the Newton-Raphson method as before, we can develop the following algorithm for this case:

- Given $`x_d \in \mathbb{R}^m`$, start with an **initial guess** $`q_0 \in \mathbb{R}^n`$.
- Apply the **update iteratively**: $`q_{k+1} = q_k + J^{-1}(q_k)(x_d - f(q_k))`$. Here, J is the **Jacobian matrix** that represents the partial derivatives of the end-effector positions with respect to the joint variables (we know from the velocity kinematics lesson that this Jacobian is called the **coordinate Jacobian** of the robot).
- Continue until the norm of the error, $`||x_d - f(q_k)||`$, is less than a specified **tolerance**, indicating that the end-effector is sufficiently close to the desired pose (or alternatively, you can continue until the next value of x is sufficiently close to the previous value of x).

Note that the initial guess, $`q_0`$, can significantly affect **convergence** and the solution found, especially in systems with multiple degrees of freedom as you might have multiple solutions and, therefore, multiple local peaks. Sometimes, the solution doesn't even converge if you pick an initial guess that is very far away from any potential solution. Therefore, the initial guess should be picked so that it is close enough to the most likely desired solution.

Also, we should remember that the **Jacobian** is **not always invertible** (in fact, in most cases in robotics). We can address this computational problem by using the **Moore–Penrose pseudoinverse** $`J^\dagger`$. The **Moore-Penrose pseudoinverse**, also known simply as the pseudoinverse or the **generalized inverse**, is a mathematical tool used in linear algebra and matrix theory. It is a generalization of the matrix inverse for non-square matrices or singular matrices, which cannot be inverted using standard matrix inversion techniques. The Moore-Penrose pseudoinverse of the Jacobian matrix J, denoted as $`J^\dagger`$ can be defined as follows for $`J_{m \times n}`$: 

$`J^\dagger = J^T (JJ^T)^{-1} \text{ for the case }n>m`$

$`J^\dagger = (J^T J)^{-1}J^T \text{ for the case }n\lt m`$

$`J^T`$ represents the transpose of J. 

We can also use **MATLAB or Python** to calculate the pseudoinverse. The _pinv_ function in MATLAB and the _numpy.linalg.pinv_ function in Python can do this for you. Therefore, the update in the above algorithm can be rewritten as:

$`q_{k+1} = q_k + J^{\dagger}(q_k)(x_d - f(q_k))`$

There's an **issue** with this first version, however, as it doesn't suit our approach that considers **frames** rather than **coordinate points**. Now, we need to go back to our frames at the beginning of this section and **modify** the above algorithm to work with **frame transformations** instead of **coordinate points**. 

In the above algorithm, we could simply calculate the **error** by **subtracting** the forward kinematics solution at the specific joint position from the desired end-effector position since both were **vectors**. Now we have the frame $T_{sd} \in SE(3)$ as the **desired frame** that the end-effector wants to reach instead of the $`x_d`$, which was the **desired** end-effector **coordinates vector**. Here, we need to **minimize the error** between the **end-effector frame** w.r.t the base frame and this **desired frame** to find the solution (**joint positions**) that can get the end-effector to this desired frame. Here, since we are dealing with **transformation matrices** that consist of **position** and **orientation** (and therefore do not belong to a vector space), we **cannot** simply find the error by subtracting the two frames. Rather, we will use the interchangeable concepts of the **Twist** and the **screw motion**.  

Like the concepts of **position and velocity** in physics, **twist and screw motion** are related to each other. If body twist $`\mathcal{V}_b`$ is followed over unit time, then it can cause a motion from $`T_{sb}(q_i)`$ to the desired configuration $`T_{sd}`$. In other words, this can interpreted as follows: We imagine that a set of angular velocities act on the joints of the robot, resulting in an end-effector body twist. Both velocities last for a particular period of time, effectively resulting in an end-effector frame position change, which results in a new desired frame position relative to the old body frame position. To further explain this, if we assume that the velocities act for a unit period of time, the motion can be interpreted as a velocity acting for a specific period of time, resulting in a **frame position change**. Therefore, we can write the body twist using the **matrix logarithm** as:

$`[\mathcal{V}_b] = log T_{bd}(q_i)`$ 

Where $`T_{bd}`$ is the desired configuration in the body frame and can be expressed in terms of {b} and {d} frames as:

$`T_{bd}(q_i) = T^{-1}_{sb}(q_i) T_{sd}`$ 

Now, we can **update** our **Newton-Raphson inverse kinematics algorithm** for **frame transformations**:

- Given the **desired end-effector frame transformation** $`T_{sd}`$, start with an **initial guess** for the joint angles, denoted as $`q_0 \in \mathbb{R}^n`$.
- Define the **"error" transformation matrix** as $`T_{bd}(q_i) = T_{sb}^{-1}(q_i) T_{sd}`$, where $`q_i`$ is the **current guess** for the joint angles.
- We seek to **minimize** the **error** between the **current end-effector configuration** $`T_{sb}(q_i)`$ and the **desired configuration** $`T_{sd}`$. We represent this error as a **body twist** $`[\mathcal{V}_b]`$, which can be computed using the **matrix logarithm**:

$`[\mathcal{V}_b] = \log T_{bd}(q_i) = log (T^{-1}_{sb}(q_i) T_{sd})`$

- Update the joint angles $`q_{i+1}`$ iteratively, we use the following equation:

$`q_{i+1} = q_i + J_b^{\dagger}(q_i) \mathcal{V}_b`$

Where $`q_i`$ is the **current** **joint angles guess**, $`\mathcal{V}_b`$ is the **body twist** calculated from the **error** between the **current end-effector configuration** and the **desired configuration**, and $`J_b^{\dagger}(q_i)`$ is the **pseudoinverse** of the **body Jacobian** matrix evaluated at the current joint angles $`q_i`$. This is used to account for situations where the Jacobian may not be invertible.

- Continue the iteration until the error (represented by $`[\mathcal{V}_b]`$) is **sufficiently small**, indicating that the end-effector is close to the desired frame configuration; alternatively, until when the **new joint angle** is close enough to the **previous calculated joint angle**. 

## Class Activity: Numerical Inverse Kinematics of the 2R Planar Robot Arm 

Using the numerical inverse kinematics algorithm that we learned above, write a MATLAB code that solves the inverse kinematics solution for the 2R planar robot arm depicted at the beginning of this lesson. Find the set of joint angles that can take the end-effector frame to the following desired pose:

$`T_{sd} = \begin{pmatrix}
-0.7071 & -0.7071 & 0 & 0.7071\\
0.7071 & -0.7071 & 0 & 2.1213\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\
\end{pmatrix}`$

**Notes:**

- Suppose $`L_1 = 2, L_2 = 1`$. 
- You also have $`T_{sb}`$ from forward kinematics. You just need to evaluate it at the current guess in the for loop.
- You already have the space Jacobian for this arm from the previous class activity. Use that and convert it to the body Jacobian using the adjoint transformation. 
- The algorithm can be implemented by a simple _for_  or _while_ loop. 
- It is easy to set the convergence criteria as when the previous calculated angles are close enough to the current calculated joint angles. _abs_ function in MATLAB can help you with the norm 
- _pinv_ function calculates the pseudo-inverse in MATLAB.
- _logm_ function calculates the matrix logarithm in MATLAB.
- Submit the code as it will be graded. 

## References 

- Kevin, M.L. and Frank, C.P., 2017. Modern robotics: mechanics, planning, and control
- Murray, R.M., Li, Z. and Sastry, S.S., 2017. A mathematical introduction to robotic manipulation. CRC press.
- Corke, P., 2023. Robotics, Vision and Control: Fundamental Algorithms in Python (Vol. 146). Springer Nature
- https://www.wolfram.com/mathematica/