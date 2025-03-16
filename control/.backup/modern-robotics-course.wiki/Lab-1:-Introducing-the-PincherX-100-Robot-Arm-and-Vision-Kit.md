## Objectives of lab 1
- Become familiar with PincherX 100 robot arm and the vision kit
- Identify the robot's links, joints, actuators, and sensors
- Installing Ubuntu 22.04 

## PincherX 100 robot arm and the vision kit

For most of the labs and projects in this class, we will use the serial robot arm, <a href="https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx#overview">the PincherX 100</a> from Trossen Robotics with <a href="https://www.trossenrobotics.com/interbotix-arm-vision-kit.aspx">the vision kit</a> for vision-based control. This robot arm is controlled by Robot Operating System (ROS) and we will use <a href="https://docs.ros.org/en/humble/index.html">ROS2 Humble</a> which is the eighth release of ROS and it is a long-term support (LTS) release meaning that it will be updated and the bugs will be fixed until May 2027. We will use <a href="http://wiki.ros.org/rviz">RViz</a> as the visualization tool to visualize the state of the robot, including its position, orientation, and sensor data. Python is the preferred language and we will use the Python-ROS API which sits above the ROS so that even if you are not proficient with ROS, you can still control and program the robot arm. 

In this class, each group is assigned a robot arm, and a vision kit. The robot should be fastened to an 18"x24" wooden board for stability during operation. Here are the steps to do:
- Unbox the robot arm and the vision kit
- The robot arm comes pre-installed and all you have to do is to attach the gripper (note that you can always 3D print your own gripper for the specific needs of a task). Follow the instructions in the following video to do so:

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/0fd6e599-4834-45d3-a5b6-835b8622dd8f
<figcaption> <p align="center">How to install the gipper. Video credit: Trossen Robotics</figcaption> </p>

- Make sure to take into account the joint limits when turning the robot arm, because if you turn it beyond the joint limits, you will damage it (take this into account when programming the robot as well). The joint limits for the PincherX 100 robot arm to ensure a safe range of operation are:

<img width="645" alt="pincherx 100 joint limits" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/e07001cf-6402-498b-8b78-36964f546b1c">

- Now assemble the vision kit according to the box instructions which is so straightforward. The only place that you may need help, is installing the AprilTag which is a visual fiducial (artificial feature) system. Targets can be created from an ordinary printer, while the AprilTag detection software calculates accurate 3D position, orientation, and identity of the tags in relation to the camera. Learn more about the AprilTag at the link below:

https://april.eecs.umich.edu/software/apriltag

Follow the video below to install the AprilTag (make sure that the orientation of the tag is like the video):

https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/54ee78ce-64d5-41ec-b33c-3ad464f5ea22

- **TODO:** For the next class, go to the machine shop at Oliver Hall and make an 18"x24" wooden board. Then drill holes in it according to the template below:

[pincherx100-mounting template.pdf](https://github.com/madibabaiasl/modern-robotics-I-course/files/12423329/pincherx100-mounting.template.pdf)

Then use a set of M4 pem nuts and thumb screws or 3/4” #8 Rounded Pan Head Wood Screws to fasten the robot arm to the wooden board. 

<img width="745" alt="mounting instructions" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/7ffaa9ea-b089-45ee-98a6-d40c09c6e1ed">

- Now it's time to power on the robot and connect it to the computer. 

**Warning:** While the robot is powered on, exercise caution to avoid touching any cables, connectors, boards, or other electrical components onboard, as this could lead to the risk of electrical shock injury.

<img width="826" alt="connect the robot arm" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/646a44a2-bba3-4bc9-96a2-124760041f3d">

Our arm is now ready for ROS installation in the next lab. 

## PincherX 100's links, joints, actuators, and sensors

Question 1: Based on what we have learned in lesson 1, in terms of the kinematics design, what kind of a robot is the PincherX 100? 

Question 2: Identify the robot's links and joints. 

The links of the PincherX 100 robot arm are actuated using the Dynamixel X-Series smart servos by ROBOTIS where each smart servo has a motor, a control board, a gearbox, and an encoder that can fine-tune smooth joint motions. Learn more about smart servos at the link below:

https://youtu.be/U0v_0ZyX-SI

<img width="217" alt="smart servo" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d008a8d0-25e1-45aa-89a0-59b289c3124e">

These servos come with a small-size USB communication converter (it converts USB to TTL/RS-485) that enables to control and to operate the DYNAMIXEL with the computer. It has both 3Pin connectors for TTL communication and 4Pin connectors for RS-485 communication (two different methods used for data transmission in electronic systems). What is the communication used in the PincherX 100? 

![u2d2_separate_ttl_485](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/384fba32-35bd-4ead-a94b-f47a76dbaa58)
<figcaption> <p align="center">Photo credit: ROBOTIS</figcaption> </p>

If you need more information about the U2D2 controller, refer to the website below:

https://emanual.robotis.com/docs/en/parts/interface/u2d2/

Now let's talk about the sensors. As mentioned earlier, the smart servos have embedded encoders that provide feedback on the position, velocity, and torque of the motor. This feedback is used to control the motor and to ensure that it operates within its safe operating limits. 

Question 3: According to lesson 1, what kind of sensor this encoder is? 

The other sensor that we will use in this course is the camera and the vision kit that comes with it. This sensor will allow us to add a computer vision component to our robot setup to perform different tasks that need that extra eye. The camera that we will use is the Intel D415 RealSense Depth Camera which features 1920x1080 resolution at 30fps with a 65° × 40° field of view and an ideal range of 0.5m to 3m.

Question 4: According to lesson 1, what kind of sensor this camera is? 

Question 5: According to lesson 1, what is the main limitation of a single camera and how do you think this camera solves it? Tip: use the datasheet below to answer this question:

[Intel-RealSense-D400-Series-Datasheet-March-2023.pdf](https://github.com/madibabaiasl/modern-robotics-I-course/files/12423369/Intel-RealSense-D400-Series-Datasheet-March-2023.pdf)

## Installing Ubuntu 22.04 

**Video Tutorial:** https://youtu.be/C51S9C1KO1Q

In order to control our robot arm, we need to install Robot Operating System (ROS) and the distribution that we will use is ROS2 humble. ROS works better with Linux-based operating systems like Ubuntu and the version of the Ubuntu depends on the ROS distribution. ROS2 humble works with Ubuntu 22.04 and we need to install it on our computer. We will install it as a standalone operating system and not use virtual machines because of the speed and quality reduction following from using virtual machines. Ubuntu is an open-source operating system which is very suitable for robotics applications. 

What you need:
- a fast USB of at least 12 GB of space. 
- a computer running Windows

Steps to install Ubuntu 22.04

- First, check if your hardware is Ubuntu Certified (I used Dell Precision 5570 with Core i9 and 32 GB of RAM):

https://ubuntu.com/certified?q=&limit=20&category=Desktop&category=Laptop

- The first thing you should do is to go to this website and download Ubuntu’s desktop version and save it to your computer:
https://releases.ubuntu.com/jammy/
- Then you should create a live USB disk. For this, you will need software to flash. There are different software out there but I personally used balenaEtcher. Go to the following website and download and install it:
https://etcher.balena.io/
- Now insert your USB drive, and open up balenaEtcher and flash your disk from the file that you downloaded from the Ubuntu website. And Voila you now have a live USB.
- Now boot your system with this USB (if it did not start automatically, on Windows you can press the F12 key) and choose Install Ubuntu 22.04. Follow the instructions on the screen to install Ubuntu. 
- After installation is complete, do not forget to update. There are two ways to do this:

  - Do this through the software updater app or
  - Update it by opening the Terminal (Ctrl + Alt + T) and then typing: `sudo apt update`

- Then check for updates to apply them by typing: `sudo apt upgrade`

Ubuntu Terminal is a command-line interface (CLI) that comes pre-installed with the Ubuntu operating system and other Ubuntu-based Linux distributions. It allows users to interact with the system and execute various commands to perform tasks, manage files, configure settings, and more, all through text-based commands. You can open it by typing Ctrl + Alt + T. 

Key features and aspects of the Ubuntu Terminal include:

- File Navigation: Users can navigate the file system and access directories and files using commands like cd (change directory) and ls (list directory contents). For example, if we are in the home directory and we want to go to the music directory we type: `cd Music`, and `cd ..` goes back to the home directory. You can create a new folder in the selected directory by typing: `mkdir [name]`. You can also do this like what we do in other operating systems and using the GUI. You can also create a text file in the Music directory by typing: `touch mah.txt`.
- Package Management: Ubuntu Terminal utilizes package managers like APT (Advanced Package Tool) to install, update, and remove software packages. Users can use commands like apt-get or apt for package management tasks. We used this before to install and update the Ubuntu.

If you happen to mess up with Ubuntu and want to uninstall it to re-install it, follow the instructions in the video below:

https://youtu.be/oLksdgBj2fI

## Guidelines for Lab 1 report:

- Submit one report per person (disclose all your collaborators including the AI ones) through Canvas. 
- A good report for this lab that can get the whole points has these components: 1. A short summary of the activities done showcasing your results (50 points), 2. Answers to all questions within the lab manual (40 points), 3. Challenges faced and how you solved them (10 points). Include photos/diagrams. 
- Reports are due one week after the class that they are completed. You can submit the report one week later than the due date for half credit.

## Possible Challenges 

**Challenge:** Secure Boot issue in some laptops/desktops. For Dell laptops/desktops and some other brands we encountered an issue of difficulty installing Ubuntu and in some cases difficulty in restarting and shutting down the system.

**Solution:** For the problem of difficulty in restarting and shutting down, we noticed that these laptops have Secure Boot setup enabled in their BIOS, and that prevents the Ubuntu from booting properly. We disabled this option and some other boot options in the system BIOS. For example, for some Dell computers, the following article can be helpful:

https://www.dell.com/support/kbdoc/en-us/000131655/how-to-install-ubuntu-linux-on-your-dell-pc#Do_you_need_to_Install_Ubuntu_on_your_Dell_PC

For the problem of difficulty in installing Ubuntu, we reinstalled it using the "Safe Graphics" option, and that solved the issue. 

**Challenge:** The wrong time and date on the computer prevented the Ubuntu from updating and upgrading!

**Solution:** This was a weird problem and the solution as you may guess is to correct the system's time and date.

Pat yourself on the back for completing Lab 1! You've done a fantastic job. 