The Robot Operating System (ROS) is a freely available software framework used to control robots. It comprises a collection of software libraries and tools that facilitate the construction and sharing of code among various robotics applications. Additionally, ROS fosters a worldwide open-source community consisting of engineers, developers, and enthusiasts who collaborate to make robots better and make this field accessible to all. ROS is the backbone of a wide variety of systems like autonomous cars, and research & development systems to name a few. ROS is not an operating system in the conventional sense of process management and scheduling, but rather a structured communications layer that sits on top of the host operating system. 

In this course, we will utilize Robot Operating System 2 (ROS2) Humble in conjunction with the ROS-Python API to operate our robot arm. ROS2 Humble serves as the Long-Term Support (LTS) distribution of ROS, ensuring support for a minimum of 5 years. Our primary goal in this class is not to become ROS developers, but rather to leverage it to our advantage for controlling and programming our robot arm. The ROS-Python API will enable us to code the robot arm without needing to grasp all the intricate details of ROS. Instead, we will write our code in Python3, and this API will handle the underlying complexities. This prerequisite's objective is to provide you with an understanding of what ROS2 is and how it functions, enabling you to easily utilize it for your robot arm control.

<img width="826" alt="ros_ros-python_gazebo_our robot" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/58b38169-903a-4932-8527-09724208f86e">

In 2007, ROS originated with the intention of establishing a cooperative software framework dedicated to the advancement of robotics development. At that time, roboticists had to write all their operating code from scratch. Performing even a basic task, starting from writing the code, developing a framework controller, handling serial communication between the microcontroller and the main computer, and dealing with debugging, could consume several weeks. When dealing with more extensive projects involving lots of code, numerous sensors, and actuators, the complexity multiplies significantly. ROS was created to alleviate the complexity of this challenging situation by offering a multitude of pre-built tools and packages developed by other roboticists. These resources significantly reduce the amount of time required for software development, allowing you to get your robot operational more quickly and effortlessly. ROS also gives you the ability to scale your robotics systems. ROS2 fills in the gaps of ROS1 and is suitable not only for robotic arms but mobile robots, legged robots, drones, swarm robots, and autonomous cars. C++ & Python are officially supported and in this class, we will use Python3 to program our robot arm. 

The communication architecture of the ROS2 framework differs significantly from ROS1, and you can have a good concise overview of the architecture in the video below (Raymond did a great job of putting all this together):

https://www.youtube.com/watch?v=7TVWlADXwRw

After watching this video, answer the following questions to gauge your understanding:

1. What is Data Distribution Service (DDS) in the context of ROS 2, and how does it facilitate efficient communication and data sharing among different nodes in a robotic system?
2. How is communication achieved between nodes in ROS 2? Explain the fundamental methods nodes use to exchange information effectively within the ROS 2 framework.
3. How do publishers and subscribers interact through topics in ROS 2? Describe the role of messages in this communication process and how they enable the exchange of data between nodes.
4. How do the publisher-subscriber communication model and its use of topics contribute to enhancing the scalability of robotic systems in ROS 2? 
5. How do ROS 2 services facilitate a request-response communication pattern between nodes? Provide examples of scenarios where using services would be advantageous in a robotics application.
6. How do ROS 2 actions differ from services in terms of communication and functionality? Provide examples of scenarios where using actions would be more appropriate than using services in robotics applications.
7. How do ROS 2 bag files contribute to the analysis and development of robotic applications? Provide examples of situations where recording and replaying bag files would be advantageous for debugging, testing, and improving robotic systems.
8. What are ROS 2 packages, and how do they contribute to the modular organization and reusability of code in robotics development? Provide examples of how package management enhances collaboration and simplifies the creation of complex robotic systems.

## ROS2 Simulation and Visualization Tools 

ROS provides software packages scheduled to support robotics simulation and data visualization. 

[Gazebo](https://gazebosim.org/home) is a free robot simulator and can communicate data over ROS2. It has the ability to keep track of robot positions and replicate the state of a real robot. Additionally, it incorporates virtual sensors, enabling the simulation of authentic sensor data. This allows you to evaluate and test your code as if you were running it on an actual physical robotic system. 

[Rviz](http://wiki.ros.org/rviz) and [RQT](http://wiki.ros.org/rqt) are for visualization purposes. Rviz is a comprehensive software suite for 3D data visualization that allows seamless interaction with data using ROS2. It is equipped with various data visualization capabilities, enabling users to effectively interact with their robot. RQT on the other hand is a plugin-based GUI that can be used with ROS2. It comes with variable graphical plugins such as a topic publisher, image viewer, parameter updater, and node graph visualizer. RQT focuses most on user interaction with ROS2, whereas Rviz focuses on visualizing 3D data. 

<img width="823" alt="gazebo-rviz-rqt" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/f0874d55-9d07-438c-8438-0022dcf8cd93">

The robots can be modeled by creating a URDF (Unified Robot Description Format) model. URDF is an XML-based file format used to describe the geometry, kinematics, and visual properties of a robot. The URDF file includes information about robot joints, links, visual and collision geometry, joint limits, and more. Part of a URDF has a structure like this:

<figure>
<p align="center">
<img width="556" alt="A URDF file" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/169235d5-f01b-4988-83f6-a641d8ccccda">
</p>
</figure>

In summary, the Robot Operating System (ROS) stands as a versatile software framework for robot control, fostering collaboration and innovation within a global open-source community. Through ROS, we tap into a powerful toolset of libraries and utilities, simplifying code creation and sharing across a spectrum of robotics applications. ROS not only serves as the backbone for diverse systems but also provides a platform for efficient and modular development. As we start our journey with ROS 2 Humble and the ROS-Python API, our goal isn't to delve into intricate ROS development but rather to leverage these tools for effective robot arm control. 

Take the following quiz to test your understanding of basic concepts of ROS2:
https://docs.google.com/forms/d/e/1FAIpQLSfZ25tYswWyzPMrJm_bttLqoZEZPw-2AR1vVGKIstDnnJ0dvw/viewform?usp=sf_link

## More Resources
- http://wiki.ros.org/
- http://wiki.ros.org/ROS/Tutorials

Here you can learn more about our robot arm's ROS2 interface, the packages, etc:
- https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/overview.html
- https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages.html

If you like to know how you can be a ROS2 developer, this Udemy Course is a great resource to begin with (ignore the first couple of modules and do not use virtual machines because they can be quite a pain):
- https://www.udemy.com/course/ros2-robotics-developer-course-using-ros2-in-python