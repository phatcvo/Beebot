## Objectives of Project 1

* The purpose of this project is to develop a code for end-effector twist using the the Jacobian method. You will be asked to derive the Jacobian matrix of the robot and use it to compute the gripper twist by multiplying it with the joint velocities.
* You will then need to validate your calculation by computing the **ground truth** actual velocity of the end-effector using ROS 2 utilities and comparing it against the velocity derived using the Jacobian operator.

**Note:** Always feel free to add your **own "twist"** to the project. Experiment with **different moves**, study the end-effector twist by changing the joint velocities, etc. Creative results or extra analysis (for example, you can analyze the impact of each joint velocity on the gripper twist) will have **extra credit**. 

The expected output will look something like this (you are supposed to **tweak the dance moves** based on your liking), where you will publish the error between the two twists calculated using the above methods (note that in the video, I am just publishing the ground truth twist (\end_eff_vel topic) and not the error):

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7ec75926-2e53-45f4-b672-83ad0c72ab42

And, here are some cool implementations by students in the previous semesters in case you are interested:

https://youtu.be/9mVqpG6WoVQ

https://www.linkedin.com/posts/mahdiehbabaiasl_modernrobotics-saintlouisuniversity-slu-activity-7132852019148591104-cXe_/

## Required hardware and software

* Computer running Ubuntu 22.04
* PincherX 100 robot arm along with its RViz simulation environment
* Python programming environment (VS code)
* Pen and paper

## Part 1: Yaml file manipulation and package creation

**Step 1.** Before writing down your code, we need to alter the yaml file that includes the control package settings, as we need to modify the joint operation modes from **position to velocity** in order to send velocity commands to the robot joints. Navigate to the control directory, then search for the modes.yaml file. You will need to remove the original content and replace it with the following code.

```yaml
groups:
  arm:
    operating_mode: velocity
    profile_type: time
    profile_velocity: 2000
    profile_acceleration: 300
    torque_enable: true

singles:
  gripper:
    operating_mode: PWM
    torque_enable: true
```

**Step 2.** We then need to create a **new package** for this project called vel_tut (or any appropriate name that you prefer) in the **interbotix_ws workspace** folder. We will then create an empty script in this package called px100_jac_ex.py (or any name that you like) and set its permission to become an **executable** file. Navigate to the interbotix workspace _interbotix_ws_ using your terminal. Once you are there, you can create your _vel_tut_ package using the following terminal command: `ros2 pkg create --build-type ament_python vel_tut`. Navigate into interbotix_ws/src/vel_tut/vel_tut. Recall that this directory is a Python package with the same name as the ROS 2 package it is nested in. You should be able to see an `__init__.py` file, next to which you will create another file (you can use VScode for that) called px100_jac_ex.py. For now, we will leave that script empty.

**Step 3.** Navigate one level back to the ros2_ws/src/vel_tut directory, where the setup.py, setup.cfg, and package.xml files have been created for you. Open package.xml with your text editor, you should add the following dependency lines to the dependency section of the xml code. These lines represent the dependencies (external libraries and packages) that we will need to use in the code (in your report explain what each of these are):

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>interbotix_xs_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
```

**Step 4.** Open the setup.py file. Again, match the maintainer, maintainer_email, description and license fields to your package.xml:

```xml
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```

Add the following line within the console_scripts brackets of the entry_points field (in your report explain what this line does):

```xml
entry_points={
        'console_scripts': [
                'px100_jac_ex = vel_tut.px100_jac_ex:main',
        ],
},
```

**Step 5.** Still in the root of your workspace, build your new package: `colcon build --packages-select vel_tut` that build only the selected package or `colcon build` will build all the existing packages again. 

More information on ROS package creation can be found here: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html. Also, you can find more information on creating Python node scripts in: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

## Part 2: Creating a transform listener to listen to the transformation of the end-effector frame

Additional resources to learn on how to create a transform listener from the ROS 2 humble wiki:

- https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
- https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html

**Step 1.** We need to create a **publisher** for **ground truth velocity**. In order to get this velocity, we need the derivative of the homogeneous transformation of the gripper frame with respect to the world frame, as well as its inverse. Remember that:

[![\\ \left[ V_s \right] = \dot{T}_{sb} T_{sb}^{-1} \\  \\ ](https://latex.codecogs.com/svg.latex?%5C%5C%20%5Cleft%5B%20V_s%20%5Cright%5D%20%3D%20%5Cdot%7BT%7D_%7Bsb%7D%20T_%7Bsb%7D%5E%7B-1%7D%20%5C%5C%20%20%5C%5C%20)](#_)

Our first step is to get the homogeneous transformation of the gripper frame relative to the world/base frame. Open your _px100_jac_ex.py_ script and include the following imports at the very beginning of the script. These are the external libraries/packages that we would need in our project:

```python
#!/usr/bin/env python3 
 
# Import the ROS client library for Python 
import rclpy 
 
# Enables the use of rclpy's Node class
from rclpy.node import Node 
 
# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

# We need the joint command message to send velcoity commands to joints
from interbotix_xs_msgs.msg import JointGroupCommand
 
# Import the 'Twist' message type from the 'geometry_msgs' package
# 'Twist' represents linear and angular velocities in a 3D space
from geometry_msgs.msg import Twist

# Include joint state message
from sensor_msgs.msg import JointState
 
# Math library
import math 
from math import sin, cos, pi

# Numpy library
import numpy as np
```
**Step 2.** Now we need to define the main function. The main function is the function that should execute immediately following the code run command from the terminal, and it invokes all the supporting classes/functions. We will create an object from a class called _FrameListener()_ which we are going to develop later. This class will contain all the logic and supporting functions for our project. The node will keep spinning till the user kills the node from the terminal. Copy the following snippet to your code below the imports:

```python
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = FrameListener()
  
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
```

**Step 3.** Now we will start developing our _FrameListener_ brick by brick. We first need to support it with **transform listening capabilities**, which will drive us to create two member functions inside the class called ___init__()_ and _on_timer()_. The init function is the class constructor and it makes the class inherit the ROS 2 node properties as well as initialize some internal parameters for each object. The on_timer function is a periodic function that is periodically invoked based on a predetermined rate. Let's start with the constructor.

The constructor is supposed to give the node this name _map_base_link_frame_listener_, declare the target frame (whose name in rviz is _px100/ee_gripper_link_), and set the timer period for the on_timer function to 0.1 seconds. Based on this data, copy the following class code into your script (right below the imports and right before the main function) and **fill the missing spaces**:

```python
class FrameListener(Node):
  """
  Subclass of the Node class.
  The class listens to coordinate transformations and 
  publishes the end-effector velocity at a specific time interval.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('                   ')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', '             ')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
 
    # Call on_timer function on a set interval
    timer_period = 
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Past variables' initialization (explain in your report why these are required)
    self.homogeneous_matrix_old = np.zeros((4, 4)); self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time
```

**Step 4.** We also need to create the on_timer function, which is going to get the transformation of the gripper frame relative to _world_ frame. Add the following member function to your class code and **fill the missing spaces**:

```python
def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = '       '
   
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
```

You can now **rebuild the package**.

**Step 5.** Launch the RViz simulation using the terminal. You can then run the node using a new terminal tab: `ros2 run vel_tut px100_jac_ex`. You can open a new terminal tab to view the published transformations from RViz using: `ros2 topic echo \tf`.

## Part 3: Computing the homogeneous transformation and end-effector velocity and publishing it

**Step 1.** For this part, we first need to convert the quaternion part of the **received transformation** to the **rotation matrix** convention. A quaternion is another form of rotation description that is often used in ROS. You should add one supporting function to your class to **convert the quaternion input to a rotation matrix** that would be used to construct the **homogeneous transformation matrix**. Learn the quaternion notation to express orientations in robotics and then complete the code below, where you will write a function that will convert the unit quaternion to a rotation matrix:

Unit Quaternion Lesson:
https://mecharithm.com/learning/lesson/unit-quaternions-to-express-orientations-in-robotics-14

Complete this function and add it to your class code:

```python
def [give it a name](self, q0, q1, q2, q3):
    """
    Convert a quaternion into a rotation matrix

    """
   
    # Calculate the rotation matrix

    rotation_matrix = 

    # Create a 4x4 homogeneous transformation matrix with zero translational elements (explain in the report why we should do this?)
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix

    return homogeneous_matrix
```

**Step 2.** We need to update our _init_ function in order to create a **velocity publisher**. The publisher message is of the type _Twist_ and we will call the topic _\end_eff_vel_. Copy the following updated code and **fill the missing spaces**:

```python
def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('          ')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', '             ')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    # Velocity publisher
   
    self.publisher_vel = self.create_publisher(
              , 
      '      ', 
      1)
 
    # Call on_timer function on a set interval
    timer_period = 
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Past variables' initialization
    self.homogeneous_matrix_old = np.zeros((4, 4)); self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time
```

**Step 3.** We need to update the on_timer function to incorporate the publishing utility. The function should compute the derivative of the transformation and its inverse. Copy the following updated code and **fill the missing spaces**: 

```python
def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = '     '
   
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
       
    # Get the homogeneous matrix (explain these in your report)
    homogeneous_matrix = self.[your function name](trans.transform.rotation.x,               ,                   ,         )   
    homogeneous_matrix[0,3] = trans.transform.translation.x; homogeneous_matrix[1,3] = 
    homogeneous_matrix[2,3] = 

    # Compute the time derivative of T using numerical differentiation (explain this in your report)
    homogeneous_matrix_deriv = (                          ) / 0.1 # Transformation derivative
    self.homogeneous_matrix_old = homogeneous_matrix # Update your old records
    homogeneous_matrix_inv = 
    # Compute the matrix form of the twist (write the math equations that you used to complete this in your report)
    vel_brack = 
    ang_vel =  # Angular velocity vector of gripper w.r.t world frame
    trans_vel =  # Translational velocity vector of a point on the origin of the {s} frame expressed w.r.t world frame

    # Publish the velocity message
    vel_msg = 
    vel_msg.linear.x = 
    vel_msg.linear.y = 
    vel_msg.linear.z = 
    vel_msg.angular.x = 
    vel_msg.angular.y = 
    vel_msg.angular.z = 
    self.publisher_vel.publish(vel_msg) 
```

**Step 4.** You can now rebuild the package and launch the Rviz simulation. Run the node and open a new terminal tab to view the \end_eff_vel topic using the echo command. The published velocity should be zero as the robot is still static.

## Part 4: Computing the velocity using the Jacobian and publishing the error between the Jacobian velocity and ground truth velocity

**Step 1.** We now need to create a subscriber that listens to the robot joint states, since the jacobian matrix is dependent on the robot joint angles. Remember that the end-effector velocity is the sum of _n_ spatial twists, which can be represented as follows:

$`\mathcal{V}_s = S_1 \dot{q}_1 + Ad_{e^{[S_1] q_1}}(S_2) \dot{q_2} + Ad_{e^{[S_1] q_1}e^{[S_2] q_2}}(S_3) \dot{q_3} + \ldots`$

Therefore, the Jacobian matrix can be written down as follows:

$`\mathcal{V}_s = J_{s1}(q) \dot{q}_1 + J_{s2}(q) \dot{q}_2 + ... + J_{sn}(q) \dot{q}_n = \begin{bmatrix}
J_{s1} & J_{s2} & . & . & . & J_{sn}
\end{bmatrix}\begin{bmatrix}
\dot{q}_1\\
\dot{q}_2\\
.\\
.\\
.\\
\dot{q}_n\\
\end{bmatrix} = J_s(q) \dot{q}`$

We, therefore, need to edit the _init_ function again. The subscriber will receive a message of the type _JointState_ and the topic to which the node would subscribe is called _/px100/joint_states_. Copy the following updated _init_ code and **fill the missing spaces**:

```python
def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('                 ')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', '             ')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    # Create velocity publisher

    self.publisher_vel = self.create_publisher(
               , 
      '       ', 
      1)
    # Velocity error publisher
    self.publisher_vel_err = self.create_publisher(
                , 
      '/vel_err', 
      1)
 
    # Call on_timer function on a set interval
    timer_period = 
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Past variables' initialization
    self.homogeneous_matrix_old = np.zeros((4, 4)); self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time

    # Define your jacobian matrix which is dependent on joint positions (angles) (make sure to show your calculation in your report)
    # all zero elements of the matrix should be calculated and entered in this matrix as a function of joint angles
    self.J = np.array([[0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0]]) # Iinitial jacobian
    
    # Create the subscriber
    self.subscription = self.create_subscription(
                       , 
      '               ', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning

    # Create joint position variable (initialization)
    self.angles = [0.0, 0.0, 0.0, 0.0]
```

**Step 2.** Update the _on_timer_ function to compute the Jacobian, the Jacobian derived velocity and publish the velocity error between the ground truth velocity and the jacobian derived velocity. Copy the following code and **fill in the missing spaces**:

```python
def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = '       '
   
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
       
    # Get the homogeneous matrix
    homogeneous_matrix = self.[Your function name](trans.transform.rotation.x,          ,                ,                    )   
    homogeneous_matrix[0,3] = trans.transform.translation.x; homogeneous_matrix[1,3] = 
    homogeneous_matrix[2,3] = 

    # Compute the time derivative of T using numerical differentiation
    homogeneous_matrix_deriv = (                         ) / 0.1 # Transformation derivative
    self.homogeneous_matrix_old = homogeneous_matrix # Update your old records
    homogeneous_matrix_inv = 
    # Compute the matrix form of the twist
    vel_brack = 
    ang_vel =  # Angular velocity vector of gripper w.r.t world frame
    trans_vel =  # Translational velocity vector of a point on the origin of the {s} frame expressed w.r.t world frame

    # Publish the velocity message
    vel_msg = 
    vel_msg.linear.x = 
    vel_msg.linear.y = 
    vel_msg.linear.z = 
    vel_msg.angular.x = 
    vel_msg.angular.y = 
    vel_msg.angular.z = 
    self.publisher_vel.publish(vel_msg) 

    # Compute twist using jacobian
    self.J = 
    vel_from_jac = self.J @ np.array([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])
    
    # Publish the velocity error message
    vel_err_msg = 
    vel_err_msg.linear.x = 
    vel_err_msg.linear.y =
    vel_err_msg.linear.z = 
    vel_err_msg.angular.x = 
    vel_err_msg.angular.y =
    vel_err_msg.angular.z = 
    self.publisher_vel_err.publish(vel_err_msg) 
```

**Step 3.** We also need to add the callback function that is invoked whenever a new joint state message is published from RViz. Add the following supporting callback function to your class:

```python
def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.angles = [data.position[0], , , ]
```

## Part 5: Publishing velocity commands to the robot and validation

**Step 1.** So far the robot is static and our work is not yet visible. We need to create a publisher to publish velocity commands to the robot joints. The publisher topic name is _/px100/commands/joint_group_ and it has a custom message type designated for interbotix robots called _JointGroupCommand_. We are going to send a velocity trajectory to the robot that makes the robot follow the following pattern for 10 seconds: wake up->freeze->dance->freeze (Feel free to design your own dance moves). As usual, we need to update the _init_ function and **fill the missing spaces**, as follows: 

```python
def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('                        ')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', '               ')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    # Create publisher(s)  
    # Create joint angular velocity publishers (after editing the yaml file)
    self.a_pub = self.create_publisher(               , '                 ', 1) # arm publisher
    # Velocity publisher
    self.publisher_vel = self.create_publisher(
              , 
      '      ', 
      1)
    # Velocity error publisher
    self.publisher_vel_err = self.create_publisher(
                , 
      '        ', 
      1)
 
    # Call on_timer function on a set interval
    timer_period = 
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Past variables' initialization
    self.homogeneous_matrix_old = np.zeros((4, 4)); self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time

    # Define your jacobian matrix
    self.J = np.array([[0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0]]) # Iinitial jacobian
    
    # Create the subscriber
    self.subscription = self.create_subscription(
                      , 
      '              ', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning

    # Create joint position variable
    self.angles = [0.0, 0.0, 0.0, 0.0]
```

**Step 2.** We also need to update the _on_timer_ function to include the new publisher and change the joint velocities from zero to the published velocity commands, as follows:

```python
def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = '            '
   
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
       
    # Get the homogeneous matrix
    homogeneous_matrix = self.[your function name](trans.transform.rotation.x,                      ,                  ,         )   
    homogeneous_matrix[0,3] = trans.transform.translation.x; homogeneous_matrix[1,3] = 
    homogeneous_matrix[2,3] = 

    # Compute the time derivative of T using numerical differentiation
    homogeneous_matrix_deriv = (                            ) / 0.1 # Transformation derivative
    self.homogeneous_matrix_old = homogeneous_matrix # Update your old records
    homogeneous_matrix_inv = 
    # Compute the matrix form of the twist
    vel_brack = 
    ang_vel =  # Angular velocity vector of gripper w.r.t world frame
    trans_vel = # Translational velocity vector of a point on the origin of the {s} frame expressed w.r.t world frame

    # Publish the velocity message
    vel_msg = 
    vel_msg.linear.x = 
    vel_msg.linear.y = 
    vel_msg.linear.z = 
    vel_msg.angular.x = 
    vel_msg.angular.y = 
    vel_msg.angular.z = 
    self.publisher_vel.publish(vel_msg) 

    # Publish velocity commands
    t = trans.header.stamp.sec # Time stamp in seconds
    a_msg =                    # Message type: JointGroupCommand
    a_msg.name = 'arm'
    
    # Robot motion commands
    # Feel free to design your own dance moves!
    if t-self.ti <= 1:
        # Stretch the arm a bit
        a_msg.cmd = [0.0, 1.0, -1.0, 0.0] # Initial velocity (rad/sec.)
    elif t-self.ti > 1 and t-self.ti <= 3:
        # Freeze
        a_msg.cmd = [0.0, 0.0, 0.0, 0.0] # Initial velocity (rad/sec.)
    elif t-self.ti > 3 and t-self.ti <= 10:
        # Dance --> Feel free to design your own dance moves
        a_msg.cmd = [0.3, 0.0, sin(2*pi*(self.get_clock().now().nanoseconds / 1e9)), 0.0] # Initial velocity (rad/sec.)
    else:
        # Freeze again
        a_msg.cmd = [0.0, 0.0, 0.0, 0.0] # Initial velocity (rad/sec.)

    # Publish velocity commands
    self.a_pub.publish(          )

    # Compute twist using jacobian
    self.J = 
    vel_from_jac = self.J @ np.array([[a_msg.cmd[0]],
                                      [a_msg.cmd[1]],
                                      [a_msg.cmd[2]],
                                      [a_msg.cmd[3]]])
    
    # Publish the velocity error message
    vel_err_msg = 
    vel_err_msg.linear.x = 
    vel_err_msg.linear.y = 
    vel_err_msg.linear.z = 
    vel_err_msg.angular.x = 
    vel_err_msg.angular.y = 
    vel_err_msg.angular.z = 
    self.publisher_vel_err.publish(vel_err_msg) 
```

**Step 3.** You are now ready to launch and run everything again after rebuilding the package. Feel free to echo the _\vel_err_ topic to see the magnitude of the error between **your calculated Jacobian** and **the ground truth Jacobian**. In an ideal scenario, the error should be zero, but because the ground truth velocity was calculated using a numerical method (which is not 100% accurate, but close enough), you should see very small values that almost approach zero.

**Step 4.** If everything worked fine in the simulation, go ahead and test your program on your robot arm.

## Guidelines for Project Report and Presentation

- Submit one report, code, and video per group. Make sure to include your math calculations in the report. Also, explain the parts of the code that I asked to explain within the code guides.
- The video should show the robot dancing (or maybe painting based on your imagination), and the end-effector twist and the error between the two methods should be published in the command line interface.  
- Prepare a presentation for the class as well (you can present it like you present your senior design projects). 

Good luck. 