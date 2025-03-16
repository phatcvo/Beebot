## Video Version of the Lesson

https://youtu.be/4exT9R0lKLE

## Introduction

The Czech playwright Karel Capek is credited with coining the word "robot" in his 1920 play, R.U.R. (<a href="https://www.gutenberg.org/ebooks/59112">Rossum's Universal Robots</a>), but the concept of robots as goal-oriented machines that can sense, plan, and act has been around for centuries. For example, the ancient Greeks created stories about automatons, which were self-moving machines that could perform tasks. In science fiction, robots were machines that would rebel and end humanity. 

<figure>
<p align="center">
  <img width="500" height="187.5*2" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/c61754a4-6f1a-4637-ab43-dc3c8745fb31">
<figcaption> <p align="center">In popular culture, robots are machines that are meant to end human civilization.</figcaption> </p>
</p>
</figure>

But science fiction aside, a robot is a goal-oriented machine that can sense, plan, and act. A robot senses its environment using different sensors and uses that info together with a goal to plan an action (either move the tool of a robot manipulator arm to grasp an object or drive a mobile robot to some place).

<figure>
<p align="center">
<img width="793" alt="robot-moves-end-effector-mobile robot" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/100a90f2-1fc1-489c-be47-69aa1599f518">
<figcaption> <p align="center">A robot interacts with its environment by sensing, planning, and taking action like moving the tool to grab something or navigating to a goal.</figcaption> </p>
</p>
</figure>

## Robot Sensors 
 
The sensors can be proprioceptive or exteroceptive. Proprioceptive sensors measure the state of the robot itself (for ex. using an encoder to measure the angle of the joints on a robot arm or the number of wheel revolutions on a mobile robot). Exteroceptive sensors measure the state of the world with respect to the robot. For example, a bump sensor on a robot vacuum cleaner helps the robot to detect collisions. Another example is a camera that enables the robot to sense the world through vision. A camera creates images like the human eye and this along with computer vision algorithms extracts meaning from the images and the robot can recognize and manipulate objects.   

<figure>
<p align="center">
  <img width="500" height="500" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b5ca324e-36ed-4723-8671-3a665d461d6b">
<figcaption> <p align="center">Cheap cameras, new algorithms, and the massive parallel computing power of today’s computers made the vision a practical sensor for robots. Photo: Trossen Robotics Vision Kit.</figcaption> </p>
</p>
</figure>

An important limitation of a camera is that when taking a photo, the 3D structure of the scene is lost in the resulting 2D image. The solution here is to use stereo vision where information from two cameras is combined to estimate the 3D structure of the scene. This is how our eyes perceive depth. The camera that we have in our vision kit is a depth-sensing camera.

<figure>
<p align="center">
<img width="691" alt="sensing depth using two cameras" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/9d3f4919-88f7-4768-88c5-96e91eae80de">
<figcaption> <p align="center">Two cameras are used to estimate the depth from two images. Left: Image from LearnOpenCV, and Right: Intel D415 Realsense Depth Camera that we will use in this class. </figcaption> </p>
</p>
</figure>

## Robot Actuators 

So, when the sensors help the robot to perceive its environment, the actuators help it to move its links. Robot actuators can be electrically driven or they can be driven by pneumatic or hydraulic cylinders but electrically-driven actuators like AC/DC motors, stepper motors, and shape memory alloys are more common. In order for the robot to operate optimally, these actuators should be lightweight, should operate at low rotational speeds (in the range of hundreds of RPM), and should be able to generate large forces and torques. But, most available motors operate at low torque and at up to thousands of RPMs. Therefore, speed reduction and torque amplification using gears, cable drives, etc are needed. These devices should have zero or low slippage and backlash (backlash is the amount of rotation available at the output of the speed-reduction device without motion at the input). 

<figure>
<p align="center">
<img width="781" alt="actuators in robotics" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/6aea9c5a-1e15-4d7d-a28c-310ea55fecca">
<figcaption> <p align="center">Some of the actuators used in robots: electrically driven, pneumatic, and hydraulic cylinders</figcaption> </p>
</p>
</figure>

## Robot's Links & Joints

As mentioned above, actuators can move the links of a robot. A robot arm is usually constructed by connecting links using joints. The joints allow relative motion between links and actuating joints using motors make the robot move and exert forces in desired ways. Links can be connected to each other in a serial fashion to make open-chain robot arms like the PincherX 100 robot arm that we use for this course or they can make closed loops to make parallel robots where only a number of the joints are actuated like the famous Stewart-Gough platform.

<figure>
<p align="center">
<img width="658" alt="serial vs parallel robot" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/4ed7c1e5-d38b-4fa2-9158-37fa79695e7b">
<figcaption> <p align="center">(Left) Pincherx 100 robot arm (a serial manipulator), and (Right) a parallel robot (source: Wikipedia). </figcaption> </p>
</p>
</figure>
 
## Classifications of Robots 
 
The above classification divides robot arms into serial and parallel but robot arms themselves can be a part of a bigger taxonomy that categorizes the robots into stationery robots or basically the arm-type robots that cannot move and the mobile robots that can navigate around their environment using different types of mobility like moving around using wheels, or legs, or flying using wings, or moving through the water or sail over it. 

<figure>
<p align="center">
<img width="824" alt="stationery robots and mobile robots" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/b2ec8e72-3cb6-45f3-95da-10afbee416ac">
<figcaption> <p align="center">One taxonomy classifies the robots into stationery and mobile robots. </figcaption> </p>
</p>
</figure>

There are other classifications of robots based on their application that you can find these categories at the link below:

https://www.mecharithm.com/news/robotics/based-on-application 

In modern robotics, we will deep dive into the kinematics, dynamics, motion planning, and control of robots. You may ask why we call this modern robotics. The reason is that we are using a modern approach to robotics and that is using the techniques of classical screw theory in today's robotics problems that can largely simplify the complexity of these problems.  

<figure>
<p align="center">
<img width="698" alt="modern robotics i_classical screw theory" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/0e5bd643-1ba6-42dc-88de-00b5e5ddd460">
<figcaption> <p align="center">Modern Robotics is about the fundamentals of robotics, and we will use a modern approach to robotics using the methods of classical screw theory. </figcaption> </p>
</p>
</figure>

## Job Opportunities for Robotics Engineers 

You might also be curious about the various job opportunities available for robotics engineers. Presently, robotics engineers are actively engaged in diverse sectors of the industry. In the realm of industrial automation, their responsibilities involve working predominantly with robot arms, which are utilized in a wide range of industrial applications. These tasks encompass material handling, part spraying, assembling, manufacturing, and even performing hazardous tasks that are beyond human capability.

The primary objective of a robotics engineer in this field is to ensure the proper functioning of the robots, as any breakdown could lead to a halt in production. Although robot programs may not be highly sophisticated, they can be quite complex due to the multitude of elements present in an industrial environment. Moreover, each manufacturer utilizes its own set of programming languages and technologies that you should learn, the programs can be lengthy and a small bug can lead to system failure.

When dealing with industrial-type robots, robotics engineers work with industrial-grade controllers, often referred to as PLCs (Programmable Logic Controllers). The most commonly used programming language in this context is ladder logic, which differs from languages like Python or C++, but it is designed to be highly readable and easily debuggable by engineers.

Another aspect of industrial controls involves working with robot arms within cells, each designed to perform specific tasks in a factory setting. Here, robotics engineers must have a profound understanding of these robots, including their joint angles, motor functions, calibration methods, and maintenance requirements. Industrial robots are expected to be rigid, reliable, and designed for simplicity. The goal of the programs used for these robots is not to be groundbreaking but to be repeatable, enabling consistent product manufacturing and maintaining reliability throughout the production process.

Another aspect of robotics involves the more classic research-oriented approach, where advanced equipment is employed, and robotics engineers focus on navigation and algorithm development. In this realm, the emphasis is not solely on reliability since it revolves around research and experimentation, leading to varied outcomes with each trial. The primary goal here is to be innovative, pushing the boundaries of technology and advancing the entire field of robotics. This entails creating and exploring novel sensors, inventive programming methods, and expanding software libraries, with the Robot Operating System (ROS) playing a significant role. Languages like Python and C++ are frequently utilized in this domain.

Unlike the fast-paced problem-solving approach needed in dealing with industrial robot arms, this research-oriented context allows engineers to invest more time in tackling intricate challenges within robotics. The focus is on carefully crafting solutions that may not yield immediate results but contribute to groundbreaking advancements in the field. Currently, the majority of research positions in the industry demand a Ph.D. If you're wondering about the purpose of obtaining a Ph.D., simply examine the requirements for various high-demand roles in well-known technology and robotics companies, and you'll notice that many of these positions specifically require a Ph.D.

<figure>
<p align="center">
<img width="800" alt="robotics engineers in industry and research" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/4af5028a-8454-43e9-b346-3e9ad9bc5f3e">
<figcaption> <p align="center">Robotics Engineers and researchers can work in different sectors of the industry. </figcaption> </p>
</p>
</figure>

In modern robotics, we approach robotics from a research perspective. While many concepts we discuss apply to industrial robots, such as coordinate frames, rotation matrices, and forward kinematics, our goal is to leverage tools like Python and ROS to pave the way for future innovations.

Although robotics is an ever-evolving and actively researched field, the fundamental principles of robot design, including modeling, perception, planning, and control, are well-established. In this course, we will explore these fundamentals in theory and practice, focusing on arm-type manipulators. Through hands-on experience, you will have the opportunity to work with a real robotic arm, controlled by the Robot Operating System (ROS), to gain practical insights into these topics.


## Ethical Considerations

Let's conclude our introductory lesson by discussing the ethical considerations related to robotics and AI. There is much speculation about the impact of robots on the future job market and growing concerns that they may replace human workers. Currently, robots have limitations in terms of their skill level for everyday tasks, and they lack reliability, high speed, and come with high costs. However, it's important to recognize that these limitations can be addressed and improved over time.

<figure>
<p align="center">
<img width="800" alt="ethical considerations in robotics" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/8a20e3cf-e0dc-4e3f-ba4a-3c71edcf2796">
</p>
</figure>

It's essential to understand that having robotics and AI is not inherently negative. In fact, they can offer significant benefits. For instance, in the automotive industry, the implementation of robotic automation has led to increased productivity, making it economically viable in high-wage countries such as European countries, Japan, and the USA.

There are certain jobs that humans may prefer not to do due to their physically demanding nature, like outdoor labor, or because those jobs are low-skilled, such as fruit picking. Additionally, working in unhealthy or hazardous environments is another area where robots could potentially take on these tasks, safeguarding human well-being.

Another topic of discussion concerning robotics and AI revolves around the issue of assigning blame when a robot makes mistakes. For instance, in the context of self-driving cars, there is a dilemma regarding who should be held responsible in the event of an accident caused by the vehicle. Although self-driving cars have the potential to significantly reduce the number of fatalities, society tends to be more accepting of mistakes made by human drivers, even though they are responsible for over one million deaths annually.

Similarly, in the field of robotics in healthcare and surgery, the question arises about accountability in case of errors or unfavorable outcomes. As these technologies advance and become more prevalent, the issue of responsibility becomes more complex, and determining who should bear the blame for any mishaps becomes a matter of ethical consideration.

Another concern pertains to the use of robots for the care of elderly individuals and children. This raises questions about whether relying on robots for caregiving could diminish the quality of life for the elderly by eliminating human interaction, meaningful conversations, and companionship. Additionally, the issue of whether we should place our trust in robots to take care of children and even educate them also comes into play. This is one of the questions that I want to address in my research: can we equip robots with the capability to understand human intentions and adjust their behavior accordingly, providing a sense of control and dignity to the elderly? 

Another concern pertains to the utilization of robot armies for engaging in combat and causing harm to human beings. While robots may bring economic benefits to our society, we must question whether it is morally justifiable. Striking a balance between the collective welfare of society and the well-being of individuals poses profound ethical dilemmas and challenges. It is crucial not to blindly embrace the future simply because we have the capability to do so. Instead, we must engage in thoughtful discussions and act responsibly when addressing these complex ethical questions and issues.

What are some of the other ethical concerns that you can think of?

## References
- Modern robotics: Mechanics, planning and control, Kevin Lynch and Frank Park, 2019, Cambridge University Press
- Robotics, Vision and Control: Fundamental Algorithms in Python, Peter Corke, 2023, Springer Nature
- A mathematical introduction to robotic manipulation, Murray, Li and Sastry, 1994, CRC press
- https://www.trossenrobotics.com/
- https://learnopencv.com/
- https://www.intelrealsense.com/depth-camera-d415/
- https://www.youtube.com/@ieeespectrum
- https://www.youtube.com/@ddesignhub
- https://www.youtube.com/@mostafahassanalian2195
- https://seagrant.mit.edu/
- https://www.ros.org/