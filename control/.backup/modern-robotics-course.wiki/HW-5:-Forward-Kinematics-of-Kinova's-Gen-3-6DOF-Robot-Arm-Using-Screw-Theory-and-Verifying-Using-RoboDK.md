In this homework, you will calculate the forward kinematics of Kinova's Gen 3, 6 DOF robot arm using screw theory.  

## Part 1. Forward Kinematics of Kinova's Gen 3 6DOF Robot Arm Using Screw Theory

**Step 1.** Open up RoboDK, go to file, and then choose open online library.

**Step 2.** Type Kinova in the search box and download the Kinova Gen3 6DOF:

<figure>
<p align="center">
<img width="565.3" alt="kinova gen3 6dof robodk" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/2763a28c-ca98-4abf-a130-1a0d3808267e">
</p>
</figure>

**Step 3.** Open up the robot in the software and put it in its zero position by setting all joint angles to zero:

<figure>
<p align="center">
<img width="788.6" alt="kinova gen3 robot arm in its zero position" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/0d82dbb6-29a4-436b-b917-5068b4d8e0d2">
</p>
</figure>

**Step 4.** Follow the steps in the lesson to find the forward kinematics of this robot using screw theory (40 points). 

**Step 5.** Choose a set of joint angles, calculate the transformation matrix representing the pose of the end-effector w.r.t the base frame for those angles (you can use the Python code from the lab or MATLAB code from the lesson), then set these joint angles in the RoboDK and verify that your calculation is the same as what RoboDK is showing you (30 points). 

**Notes:** 

- you can determine the positive direction of the joints by changing the joint values in the RoboDK software
- you can find the link lengths, etc. from the product's manual that can be found at the link below:

https://www.kinovarobotics.com/product/gen3-robots

## Part 2 (required for only graduate students). Screw Theory in Robotics in the Literature 

Go to Google Scholar and search for robotics problems that are solved using screw theory. List some of those for this part. Do the calculations make sense to you now? Explain (for now, you are only interested in the forward kinematics part) (30 points). 

## Submission

- Submit your answers via Canvas in a PDF format. 
- The PDF should contain the math, descriptive photos, and explanations and will be graded based on the above rubric.

Good luck