## Objectives of Lab 3:
- Working with Python-ROS interface 
- Doing a simple pick-and-place task

Now that we have successfully [installed the ROS interface](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lab-2:-ROS2-Humble-Interface-Installation-&-Running-the-Robot-in-Simulation-&-Reality) and have interacted with the robot in simulation and reality, it is time to work with the [Python-ROS interface](https://docs.trossenrobotics.com/interbotix_xsarms_docs/python_ros_interface.html) (note that this requires that you have already installed the ROS interface) that sits on top of the ROS interface and you can easily control the robot using a bunch of Python code even if you do not know how to work with ROS.

**Important Note:** There will be a lot of terminology associated with this interface like base frame, space frame, transformation, pose, etc. in the codes. If you encounter these do not fret because in this lab we want to see the "end of the movie" first and we will go back and study all of these terminologies again. For now, do not worry about them, and follow the lab to get a peek at how the robot works. 

Here are some implementations by students in the previous semesters in case you are interested:

https://youtu.be/s19rsuavsDo

Now let's see the functionality of the Python API with a couple of examples. 

## Commanding Some Arbitrary Positions to the Robot Arm's Joints

**Objective:** Controlling the position of individual joints on the robot arm 

**Step 1:** Open up a terminal by pressing `Ctrl + Alt + T`  
**Step 2:** Run the command below to launch the robot arm in simulation (the reason we do it first in simulation is that if we impose a certain pose on the real robot that is beyond its limit, **we can damage our robot arm**. So, **make sure that everything works fine in the simulation before trying it on the robot**):

`ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true`

**Explanation of this command:** Here, we launched the ROS2 launch file named [xsarm_control.launch.py](https://github.com/Interbotix/interbotix_ros_manipulators/blob/rolling/interbotix_ros_xsarms/interbotix_xsarm_control/launch/xsarm_control.launch.py) from the [interbotix_xsarm_control](https://github.com/Interbotix/interbotix_ros_manipulators/tree/rolling/interbotix_ros_xsarms/interbotix_xsarm_control) package. It configures the launch to use the "px100" robot model and enables simulation mode (use_sim:=true). Note that a launch file in ROS2 is used to start and configure various nodes.   
 
By running this code, you will have the robot in the RviZ environment. 

**Step 3:** Using the following code, you can control the joint positions (they are in **radians**). The joint positions in this code are **intentionally** missing. First, go to the [lab 1](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lab-1:-Introducing-the-PincherX-100-Robot-Arm-and-Vision-Kit) and look at the table containing the **joint limits**. After that, fill in the blanks with proper joint angles in radians. Make a .py file in the **Visual Studio Code** and put your code there and then save it as **joint_control_px100.py** in the **current** directory.  

``` Python
# Copyright 2022 Trossen Robotics

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():

    # TODO: Define the joint angles in radians considering the joint limits
    joint_positions = [joint1, joint2, joint3, joint4]

    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(joint_positions)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()
```

This Python code uses the **InterbotixManipulatorXS** class from the [interbotix_xs_modules.xs_robot.arm](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/rolling/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py) module to control our robot arm. Locate **InterbotixManipulatorXS** class within **arm.py** and see what functionalities it can give to our robot arm. 

In the code snippet above, the InterbotixManipulatorXS **class** from the interbotix_xs_modules.xs_robot.arm **module** is employed to control our robotic arm. Could you elaborate on how this code demonstrates the principles of **classes and objects** in Python?

In particular:

- How does the code create an instance of the InterbotixManipulatorXS class to represent the robotic arm?
- How are the object methods such as go_to_home_pose(), set_joint_positions(), and go_to_sleep_pose() utilized to interact with the robotic arm?
- How is the lifecycle of the arm object managed, and why is calling bot.shutdown() important?

**Step 4:** In the command line type: `python3 joint_control_px100.py` or **alternatively** you can run the code right from the VS code. 

You should observe that first, the robot goes to the home position (in simulation), each joint moves to the specified positions, then goes to the home pose, and finally to the sleep pose. 

If everything **works fine in the simulation** and the joint positions are **attainable** by the robot arm go to Step 5 to run the physical robot. 

**Step 5:** Go to **Step 1** and repeat the instructions but this time **without the use_sim argument** to see your implementation on the robot arm. 

## Performing a Simple Pick & Place Task Using the Robot Arm

**Note:** This is just a simple pick and place and does not use any sensor such as a camera for feedback. 

**Step 0:** Follow the same method as the previous example to open the simulator (to test) and then on the physical robot arm. 

**Step 1:** Write a code similar to the below code and save it as pick_and_place.py in the current directory later to perform a simple pick and place task. Try to pick up one of the cubes, rotate and toss it somewhere like into a bin, etc. (Use your **imagination**! but be sure what you ask the robot to do is **within its capabilities**).

The **safe and recommended control Sequence** for the robot arm through a series of movements from its Sleep pose is as follows:

1. Command the arm to go to its **Home pose**.
2. Command the **waist joint** until the end-effector is pointing in the desired direction (if you want it to be in the home pose then do not rotate the waist).
3. Command poses to the end-effector using the **set_ee_cartesian_trajectory()**, **grasp()**, and **release()** functions (these functions are defined in [arm.py](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/rolling/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py)) as many times as necessary to do a task (pick, place, etc.).
4. Repeat the above steps as necessary.
5. Command the arm to its **Home pose**.
6. Command the arm to its **Sleep pose**.

Complete the following code with your **imagined scenario** (Important: **Do not replicate the code** because it is just an example and **not** necessarily **doable** by our robot arm):

``` Python
# Copyright 2022 Trossen Robotics 

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )

    bot.arm.go_to_home_pose()

    # Here we give the desired orientation to the waist. The joint position is in radians. We imported numpy library because we wanted to
    # use the the mathematical constant π 
    # Note that you can substitute the waist with any of the joints of the robot 
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)

    # Using the set_ee_cartesian_trajectory() function from the arm.py library, you can define a linear trajectory using a series of 
    # waypoints that the end-effector should follow as it travels from its current pose to the desired pose

    bot.arm.set_ee_cartesian_trajectory(z=-0.1)
    bot.arm.set_ee_cartesian_trajectory(x=-0.2)

    # grasp() and release() functions from gripper.py library to control the gripper
    # TODO: go to the gripper.py library and figure out what 2.0 represents. 
    bot.gripper.grasp(2.0)

    bot.arm.set_ee_cartesian_trajectory(z=0.1)
    
    # TODO: Go to the gripper.py library and figure out why the bot object uses the set_pressure() method from the 
    # InterbotixGripperXSInterface class
    bot.gripper.set_pressure(1.0)

    bot.gripper.release(2.0)

    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()

```

**Questions:**

- According to the [Python Programming](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Prerequisite-2:-Introduction-to-Python-Programming-with-Emphasis-on-Robotics-Applications) lesson, and [Arm.py library](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/rolling/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py) explain how this code works in terms of classes and objects. 
- Using the [Arm.py](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/rolling/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py) library, explain what set_single_joint_position(), and set_ee_cartesian_trajectory() do? Don't overthink and do not get bogged down with the terminology that we will study later (orientation, rotation matrices, transformation, etc.), just in simple words what is your understanding of these functions? 
- set_ee_cartesian_trajectory() function makes the linear displacement along the x, y, and z axis possible. How are these x, y, and z are calculated based on the [arm.py library](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/rolling/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py)? (Do not worry about the terminology here. You do not want to be exact. For example, you can say, it is the distance in meters from the base frame, etc.) Just report on your observation from experimenting on the robot arm. 
- Using [gripper.py library](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/rolling/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/gripper.py) explain what grasp(), release(), and set_pressure() functions do? 


## Guidelines for Lab 3 report

- Submit one report per person (disclose all your collaborators including the AI ones) through Canvas. 
- The codes can be submitted through a GitHub repo (with link provided) or alternatively be uploaded to the submission. 
- A good report for this lab that can get the whole points has these components: 1. A short summary of the activities done (40 points), 2. Answers to all questions within the lab manual (20 points), 3. The video demos of the implementations (30) 4. Challenges faced and how you solved them (10 points). Include photos/diagrams when appropriate. 
- Reports are due **one week after the class** that they completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 3! You've done a fantastic job.