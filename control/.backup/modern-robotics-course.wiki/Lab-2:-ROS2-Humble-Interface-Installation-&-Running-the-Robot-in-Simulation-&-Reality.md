## Objectives of Lab 2:
- Installing ROS Interface 
- Running the robot in both simulation and reality

**Important Note:** Take lots of screenshots, photos, and videos of your implementation. They will be handy when completing the lab report. 

Here are some implementations by students in the previous semesters in case you are interested:

https://youtu.be/wGofLHCHWyw

## ROS2 Humble Interface Installation

In this lab, our primary objective is to install both ROS and Python-ROS interfaces to be able to control our robot arm. As our intention is to utilize ROS2 humble, in the preceding lab session, we took the necessary steps to set up Ubuntu 22.04. It was essential to select this specific version of Ubuntu since ROS distributions are OS-specific, and ROS2 humble exclusively works with Ubuntu 22.04. Consequently, we will proceed with the installation of ROS2 Humble on our Ubuntu Linux 22.04 system.

PincherX 100 robot arm comes with ROS2 interface package to make our lives easier. The `robot_model` argument in the launch files for this arm is **px100**, so this is the code name that we will use for this robot arm. px stands for PincherX and 100 is the length of both the forearm and upper-arm links in millimeters (you can verify this with your robot arm). 

What you need:
- PincherX 100 robot arm 
- Computer running Ubuntu 22.04

Here are the steps to **install ROS2 Humble** on desktop and laptop computers running Ubuntu 22.04:

First, **open a terminal** by pressing Ctrl + Alt + T, then run codes `sudo apt update` and `sudo apt upgrade` to update and upgrade your Ubuntu operating system. After that copy, paste (Ctrl + Shift + V), and run the codes below to install ROS2 humble.

This code will **install the "curl" package** on Ubuntu. Curl is a command-line tool and library used for **transferring data with URLs**. When you run this command with administrative privileges (sudo), it will prompt you to enter your password to confirm the installation. Once you provide the password, the package manager (apt) will download and install the curl package and its dependencies on your system:

``` ubuntu
sudo apt install curl
```

In the code below we are using the curl command to download a shell script named xsarm_amd64_install.sh from a GitHub repository and then save it as xsarm_amd64_install.sh in the current directory:

``` ubuntu
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
```

The **chmod +x command** is used to **change the permissions of a file**, making it **executable**. By running chmod +x xsarm_amd64_install.sh, you are granting execute permission to the xsarm_amd64_install.sh script, allowing it to be executed as a program:

``` ubuntu
chmod +x xsarm_amd64_install.sh
```

Once you've executed the above command, you can run the script using the following command with the -d humble option or argument. We specified the **version of ROS2** that we want to install **using the `-d` flag**:

``` ubuntu
./xsarm_amd64_install.sh -d humble
```

After running this code it will ask if you want to install the **perception packages** and **MATLAB-ROS API** and type **y (yes)** for both. Then it will give you an installation summary to verify that everything is correct. Mine looked like this:

<figure>
<p align="center">
<img width="562" alt="installation overview" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/1cd0b735-3b2b-46ab-946a-369bf6bfeb2a">
</p>
</figure>

Check that the **ROS distro** is right and that it will install **perception and MATLAB modules**. The last line shows that it will create a **workspace** in my home directory by that name. If everything is correct, type y to confirm and continue. It can take up to **15 minutes** to install everything. 

#### Installation Checks

To **verify that everything is installed successfully** follow the steps below:

- First, check that the **udev rules are configured correctly** and that they are **triggered by the U2D2 controller**. For this **check that when U2D2 is plugged into a USB port**, the port name is `ttyDXL`:

``` ubuntu
ls /dev | grep ttyDXL
``` 

This command searches for any device files in the /dev directory that **contain the string "ttyDXL"** in their names. These device files are associated with **communication ports for Dynamixel servos**. As we learned in Lab 1, Dynamixel is a brand of smart servos commonly used in robotics and mechatronics projects. The communication with these servos is typically done through **serial communication (UART)** using a **USB-to-serial adapter**. The device files with "ttyDXL" in their names are created when the **adapter is connected to the system** and represent the virtual serial ports associated with the adapter.

2. Now check that the **ROS packages are installed** correctly. 

First, we should source the setup.bash file for the specified ROS distribution:

``` ubuntu
source /opt/ros/humble/setup.bash
```

Then, we should source the setup.bash file in our robot arm's workspace:

``` ubuntu
source ~/interbotix_ws/install/setup.bash
```

And finally, we **list the installed ROS 2 packages** and filter the output to only show the packages that contain the string "interbotix" in their names:

``` ubuntu
ros2 pkg list | grep interbotix
```

The fundamental core ROS 2 interface packages that you should confirm are installed are: **interbotix_xs_sdk**, **interbotix_xs_msgs**, **interbotix_common_modules**, and **interbotix_xs_modules**. 

If everything checks out and the installation is complete, **restart your computer** to ensure that all changes take effect.

## Running the Robot in Simulation & in Reality

In this section, you will learn how to **work with the ROS2 interface** and get the robot up and running.

Step 1: First we need to what we call **source our workspace**. This will **set up the necessary environment variables and configurations** for the packages installed in that workspace. When you run source, it executes the commands in the specified file, allowing you to access the packages and resources in the ROS workspace. Sourcing the workspace should be done in **every new terminal** to properly configure your ROS environment. 

``` ubuntu
source ~/interbotix_ws/install/setup.bash
```

But there is a way **to get around this** and that is adding the above line of code to the end of the `~/.bashrc` file so that **you do not have to do this every time**. Access the file in the home directory and make sure that you activate it to show the **hidden contents**. 

Step 2: Open up the robot's **virtual model in RViz** and play around with the **joint_state_publisher**. 

``` ubuntu
ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=px100 use_joint_pub_gui:=true
```
This code will **launch the xsarm_description.launch.py launch file** from the [package interbotix_xsarm_descriptions](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_descriptions.html) with specific parameters like the **robot model** and **the graphical user interface for publishing joint values**.

![robot_home_pose_simulation](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8d4ca723-fa0c-49d1-aeda-1908186b7d01)

Ctrl + C will **terminate** the session from the terminal. 

Step 3: Connect to the physical robot arm:

``` ubuntu
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```
This command will **launch the xsarm_control.launch.py launch file** from the [package interbotix_xsarm_control](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_control.html) with the robot_model parameter set to px100 which is the code for our robot arm. 

By executing this command you will see that all the **motors on the robot will be torqued on** and you **cannot manually manipulate it**. You can **torque off the motors** by executing the command below in **another** terminal window. **Note: Make sure that the robot is in the rest position or else it will collapse by running this command**. 

``` ubuntu
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
```

This command will **call the service** /px100/torque_enable with the interbotix_xs_msgs/srv/TorqueEnable service type and **send a request** to **disable torque (motors) for all joints** in a group. This group includes every Dynamixel motor in the manipulator. After this, you can **manipulate** the arm and the gripper **freely**. **Also make sure to note that by moving the physical robot, the virtual model in RViz also responds accordingly**. The command below can show you **the list of the ROS2 services** that this torque_enable service is one of them:

``` ubuntu
ros2 service list
```

There is also a **control panel in the GUI** where you can interact with **services and topics** instead of the command line in the terminal. First **torque on the robot again**. Then enter the **robot's code** in the **Robot Namespace** and hit "update". Home/sleep publishes a joint group command to the joint_group topic (see a list of topics by typing `ros2 topic list`). From the group name choose "all" for all joints including the gripper or arm for all joints but the gripper. Then, for example, hit **"Go to home pose" to go to the robot's zero position**, and after that **"go to sleep pose" to go back to the sleep position**. 

<figure>
<p align="center">
<img width="741" alt="home pose and sleep pose" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/50b53566-6cb9-40ac-baa3-1dda17a17482">
</p>
</figure>

You can also do other things from the GUI (interacting with other services) like **torquing on and off** (the torque that works with torque_enable service that you can see in the service list) the robot etc (for **torque off make sure that the robot is in sleep pose** or you are holding the robot arm **preventing it from falling**). The others are services that can be found in the ROS2 service list. 

If you want to **hold a robot in a certain pose**, **torque it off again** and **manually get the robot to that pose**, and execute the following command:

``` ubuntu
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```

By doing this, the motors are again **torqued on** and the robot can **hold that pose**. 

Step 4. Now that you feel comfortable working with the physical robot arm, hold onto the robot (**so that it does not fall**), and press `Ctrl + C` in the first terminal to shut down all nodes. You will see that the robot is **torqued off** and **manually** place back the robot in its sleep position. 

So by the end of this part, you should know:
- how to interact with the **virtual robot** through interbotix_xsarm_descriptions package 
- how to interact with the **real robot** through interbotix_xsarm_control package 
- become familiar with some of concepts of the the arm's ROS2 packages like different **topics and services** that you have available. 

## Guidelines for Lab 2 report

- Submit one report per person (disclose all your collaborators including the AI ones) through Canvas. 
- A good report for this lab that can get the whole points has these components: 1. A short summary of the activities done (90 points), 2. Challenges faced and how you solved them (10 points). Include photos/diagrams in the summary. Alternatively you can submit a short video that includes the short summary of activities done and explanation of the challenges faced and how you solved them.  
- Reports are due **one week after the class** that they are completed. You can submit the report one week later than the due date for half credit.

Pat yourself on the back for completing Lab 2! You've done a fantastic job.   