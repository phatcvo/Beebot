This homework assignment is designed to test your understanding of rotation matrices and especially their application to represent orientation in robotics. 

Note: Most of these questions are adapted from the Modern Robotics textbook by Kevin Lynch and Frank Park with some modifications. 

**Question 1:** 

In terms of the $`\hat{x}_s`$, $`\hat{y}_s`$, $`\hat{z}_s`$ coordinates of a fixed space frame {s}, the frame {a} has its $`\hat{x}_a`$ axis pointing in the direction (0,0,1) and its $`\hat{y}_a`$ axis pointing in the direction (-1,0,0), and the frame {b} has its $`\hat{x}_b`$ axis pointing in the direction (1,0,0) and its $`\hat{y}_b`$ axis pointing in the direction (0,0,-1) (**feel free to use your coordinate frames to determine the missing axis**).

(a) Draw by hand the three frames, at different locations so that they are easy to see (9 points).

(b) Write down the rotation matrices $`R_{sa}`$ and $`R_{sb}`$ (9 points).

(c) Given $`R_{sb}`$, how do you calculate $`{R^{-1}_{sb}}`$ without using a matrix inverse? Write down $`{R^{-1}_{sb}}`$ and verify its correctness using your drawing (9 points).

(d) Given $`R_{sa}`$ and $`R_{sb}`$, how do you calculate $`R_{ab}`$ (again without using matrix inverses)? Compute the answer and verify its correctness using your drawing (9 points).

(e) Let $`R = R_{sb}`$ be considered as a transformation operator consisting of a rotation about $`\hat{x}`$ by $`-90^{o}`$. Calculate $`R_1 = R_{sa}R`$, and think of $`R_{sa}`$ as a representation of an orientation, $`R`$ as a rotation of $`R_{sa}`$, and $`R_1`$ as the new orientation after the rotation has been performed. Does the new orientation $`R_1`$ correspond to a rotation of $`R_{sa}`$ by $`-90^{o}`$ about the world-fixed $`\hat{x}_s`$-axis or about the body-fixed $`\hat{x}_a`$-axis? Now calculate $`R_2 = RR_{sa}`$. Does the new orientation $`R_2`$ correspond to a rotation of $`R_{sa}`$ by $`-90^{o}`$ about the world-fixed $`\hat{x}_s`$-axis or about the body-fixed $`\hat{x}_a`$-axis? Draw all the coordinate frames and show the rotation using your 3D coordinate frames (9 points). 

(f) Use $`R_{sb}`$ to change the representation of the point $`p_b = (1,2,3)`$ (which is in {b} coordinates) to {s} coordinates. Does this move the position of point p in the physical space? (9 points)

(g) Choose a point p represented by $`p_s = (1,2,3)`$ in {s} coordinates. Calculate $`p' = R_{sb}p_s`$ and $`p" = R^{T}_{sb}p_s`$. For each operation, should the result be interpreted as changing coordinates (from the {s} frame to {b}) without moving the point p or as moving the location of the point without changing the reference frame of the representation?(9 points)

**Question 2:** 

Let p be a point whose coordinates are $`p = (\frac{1}{\sqrt{3}}, -\frac{1}{\sqrt{6}}, \frac{1}{\sqrt{2}})`$ with respect to the fixed frame $`\hat{x}-\hat{y}-\hat{z}`$. Suppose that p is rotated about the fixed-frame $`\hat{x}`$-axis by 30 degrees, then about the fixed-frame $`\hat{y}`$-axis by 135 degrees, and finally about the fixed-frame $`\hat{z}`$-axis by -120 degrees. Denote the coordinates of this newly rotated point by p'. Note: You can use MATLAB/Python to do the matrix multiplication. (9 points)

**Question 3:**

(a) Given a fixed frame {0} and a moving frame {1} initially aligned with {0}, perform the following sequence of rotations on {1}: (9 points)

1. Rotate {1} about the {0} frame $`\hat{x}`$-axis by $`\alpha`$; call this new frame {2}.

2. Rotate {2} about the {0} frame $`\hat{y}`$-axis by  $`\beta`$; call this new frame {3}.

3. Rotate {3} about the {0} frame $`\hat{z}`$-axis by $`\gamma`$; call this new frame {4}.

What is the final orientation $`R_{04}`$ (just write in terms of the matrices in and no need for matrix expansion or multiplication)?

(b) Suppose that the third step above is replaced by the following: “Rotate {3} about the $`\hat{z}`$-axis of frame {3} by $`\gamma`$; call this new frame {4}.” What is the final orientation $`R_{04}`$ (same here)? (9 points)

**Question 4 (Programming):** (10 points)

Write a function in Python that checks to see if a given $`3\times 3`$ matrix is a rotation matrix. This matrix should be close enough to be a member of SO(3). You can assign a tolerance (1e-5) so that for example 0.999999 is considered to be 1 or 0.000001 is considered to be 0. Show some output example as well.  

**Submission:**

- Submit your answers via Canvas in a PDF format. 
- For the coding section, give a link to your GitHub page or submit the code directly through the canvas. 

Good luck!
