## Objectives
- Substituting the desired pose designed in part 1 with the **calculated cluster positions** from **camera feedback**
- Designing an experiment where you will use the **camera feedback** and the **numerical inverse kinematics** from part 1 to do something  

If you are curious about what my implementation looks like, here is my **not-so-creative-only-for-quick-testing** experiment (this is just for the blue cube):

https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/dbfa5d6d-3387-4738-ab39-dfd48a006c5d

And, here are some cool implementations by students in the previous semesters in case you are interested:

https://youtu.be/VquBxzu9jN8

https://youtu.be/M6aTDgBaiRg

https://youtu.be/SzFe-4Qi_HM

**This project is also featured in the prestigious Computer Vision Magazine by [RSIP Vision](https://www.rsipvision.com/):**

**Part 1:** https://rsipvision.com/ComputerVisionNews-2024March/6/

**Part 2:** https://www.rsipvision.com/ComputerVisionNews-2024April/6/

**Part 3:** https://rsipvision.com/ComputerVisionNews-2024May/6/

## Introduction and Some Useful Theory

In this project, we are going to merge the **numerical inverse kinematics** module that we developed before with the robot's perception package to create a **vision-aided inverse kinematics mission planner**. The **depth camera** will first capture the **AprilTag** attached to the robot's arm to help ROS figure out the homogeneous transformation of the robot's base frame relative to the camera and vice versa. This will help convert the camera's depth readings of scene objects to homogeneous transformations with respect to the robot's base. Later, the inverse kinematics module will map these transformations to **joint angle set-points** to make the robot catch and release these objects. You will be introduced to some image processing basics here to give you an intuition of what's going on **behind the scenes** (but we cannot go into a lot of details as machine vision needs a whole semester for its own).

The camera that we will use is the **Intel D415 RealSense Depth Camera**, which features 1920x1080 resolution at 30fps with a 65° × 40° field of view and an ideal range of 0.5m to 3m. Using the stereo vision technology and the infrared sensor will help us to estimate the 3D structure of the scene. For more information you can see [Robot Sensors](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lesson-1:-Introduction#robot-sensors) or the [Intel® RealSense™ website](https://www.intelrealsense.com/depth-camera-d415/).

### Digital Images
A **digital image** is a visual representation of an object, scene, or subject that has been captured, created, or processed in a digital format. For representing an image in computer systems, digital images are stored as an **array of numbers** (discrete elements called **pixels**). These numbers stand as **intensity** (gray level, or each color band), range, X-ray absorption coefficient, and so on.

<figure>
<p align="center">
<img width="466" alt="digital-image" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/97467ced-868b-43a3-9248-d0622486f2ec">
<figcaption> <p align="center"> Array of numbers for representing the image </figcaption> </p>
</p>
</figure>

### Camera vs Image Plane Coordinate

<img align="right" width="450" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/f7b6ed48-157a-4aac-9963-dbbe170c9f51">

**Camera Coordinate System {C}:**
* A 3D coordinate system (X,Y,Z) – units, say, in meters
* Origin at the center of projection
* Z axis points outward along the optical axis
* X points right, Y points down

**Image Plane Coordinate System {π}:**
* A 2D coordinate system (x,y) – units in mm
* Origin at the intersection of the optical axis with the image plane
* In real systems, this is where the CCD or CMOS plane is


<br clear="right"/>

### Image Buffer vs Image Plane
An image buffer is, in fact, a memory area or data structure used in computer graphics and imaging to temporarily store and manipulate image data during the rendering or post-processing process. Overall, the image buffer facilitates the conversion from the image plane (measured in millimeters) to pixel coordinates. To have a comparison of Image Plane and Image Buffer:

<figure>
<p align="center">
<img width="518" alt="Image plane and Image Buffer" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/2300fcc9-3b1e-47a6-bbd6-2e934d677f27">
<figcaption> <p align="center"> Image plane and Image Buffer </figcaption> </p>
</p>
</figure>

**Image Plane {π}:**
* The real image is formed on the CCD plane
* (x,y) units in mm
* Origin in center (principal point)

**Image Buffer {I}:**
* Digital (or pixel) image
* (row, col) indices
* We can also use ($`x_{im}`$, $`y_{im}`$)
* Origin in upper left

### AprilTag Technology

AprilTags are a type of **fiducial marker system** designed for computer vision applications. They are often used for **pose estimation** and **camera calibration**. Here's a basic overview of how AprilTags work:

**Design of Tags.** AprilTags consist of black squares on a white background, arranged in a specific pattern. The design is such that it allows for easy detection and identification by computer vision algorithms. Each tag has a **unique ID** encoded in the pattern.

**Detection.** The detection process involves capturing an image or video frame using a camera and then applying image processing techniques to identify and locate the AprilTags in the scene. The distinctive pattern of the tags makes them stand out, and the algorithm can identify the boundaries and corners of the tags.

<figure>
<p align="center">
<img width="250" alt="Apriltag" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/3ff7259f-bd16-44c7-88c2-105a6a958035">
</p>
</figure>

**Pose Estimation.** Once a tag is detected, the next step is to **estimate its pose**, which includes its **position and orientation** in 3D space relative to the camera. This process is crucial for tasks such as augmented reality, robot navigation, and other applications where understanding the object's position is essential.

**Decoding ID.** The unique ID encoded in the tag's pattern is then decoded. This ID helps to **distinguish between different tags**. Knowing the ID allows the system to associate specific information or actions with each detected tag.

**Calibration.** AprilTags are often used for camera calibration. By accurately determining the pose of the tags in the camera's field of view, the camera's intrinsic parameters (such as focal length, distortion coefficients, etc.) can be calibrated. This calibration is essential for accurate measurements and reconstructions.

**Applications.** AprilTags find applications in various fields, including robotics, augmented reality, camera calibration, and object tracking. They are lightweight, easy to detect, and computationally efficient, making them suitable for real-time applications.

#### AprilTag Image Processing Steps
The image processing techniques associated with AprilTags are designed to detect, identify, and estimate the pose (position and orientation) of the tags within a camera's field of view. Here are the key steps involved in the image processing pipeline for AprilTag detection:

1. **Image Acquisition.** The process begins with capturing images or video frames using a camera. The images are then processed to identify AprilTags within the scene. 

2. **Image Preprocessing.** Preprocessing steps may include converting the image to grayscale, adjusting contrast, and reducing noise. These steps help enhance the visibility of the tag patterns and improve the accuracy of detection.

<figure>
<p align="center">
<img width="400" alt="Image Preprocessing" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/3f92dbbb-772e-4d1b-af47-ad51081fc547">
</p>
</figure>

3. **Edge Detection.** AprilTags are characterized by black and white square patterns, and edge detection algorithms are commonly used to identify the boundaries of these patterns. Edges are points in the image where there is a significant change in intensity, and detecting them helps outline the contours of the tags.

![pg3-edge-detection](https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/a26af393-babb-4493-8e55-1d7dc0508379)

4. **Corner Detection.** Once edges are identified, corner detection algorithms locate the corners of the squares in the tag pattern. Corners are key features that define the tag's geometry, and detecting them is crucial for accurate pose estimation.

<figure>
<p align="center">
<img width="300" alt="Corner Detection" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/b484f8ad-e061-4528-bdb8-26cd572e1d67">
</p>
</figure>

5. **Segmentation.** Based on the detected edges and corners, the image is segmented to identify individual tags. This step helps isolate each AprilTag in the scene, making it easier to process and analyze.

<figure>
<p align="center">
<img width="350" alt="Segmentation" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/cce9add0-e33b-47fb-ae28-84a56a220f35">
</p>
</figure>

6. **Binary Thresholding.** AprilTags have a clear black-and-white pattern. Binary thresholding is often applied to convert the image into a binary (black and white) format to simplify the subsequent analysis. Pixels above a certain intensity threshold are set to white, while those below are set to black.

<figure>
<p align="center">
<img width="600" alt="Binary Thresholding" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/abaf4a7f-2274-4d65-b781-d98aa9641171">
</p>
</figure>

7. **Pattern Matching.** The binary image is then compared with predefined AprilTag patterns. The pattern-matching process involves checking for a match between the observed binary image and the known patterns associated with different AprilTag IDs.

<figure>
<p align="center">
<img width="400" alt="Pattern Matching" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/dc139d4a-cc3d-4ab5-8fb3-de365c2cb6aa">
</p>
</figure>

8. **Decoding ID.** Once a match is found, the unique ID encoded in the tag's pattern is decoded. This ID is used to identify the specific AprilTag among others in the scene.

9. **Pose Estimation.** Pose estimation involves determining the position and orientation of the detected AprilTag relative to the camera. This step is crucial for applications where the spatial relationship between the camera and the tag needs to be known.

10. **Calibration.** If camera calibration is part of the application, the detected AprilTag poses are used to calibrate the camera's intrinsic parameters.

The combination of these image processing techniques allows for robust and real-time detection of AprilTags, making them suitable for various computer vision applications. The efficiency of the algorithms is one of the reasons AprilTags are popular in robotics, augmented reality, and other fields.

## Physical Setup and Running the Perception Pipeline

First, you need to make the **physical setup** ready. To begin, construct your stand and fasten the RealSense camera onto the 1/4-inch screw located at the top. Subsequently, position the stand in your designated workspace and manipulate the goose-neck or ball/socket joint to orient the camera towards your tabletop. Then, arrange your target objects and the robot in a way that the objects and the AprilTag are clearly visible in the camera's view. Here is my setup (for your reference):

<figure>
<p align="center">
<img width="400" alt="my setup for the vision project" src="https://github.com/madibabaiasl/modern-robotics-course/assets/118206851/3e7f8970-d610-4294-9802-20c7d73e776a">
</p>
</figure>

There is nothing special about this setup. Just set up the camera in a way that the AprilTag and the objects are **clearly visible**. Also, choose objects that can fit into the robot's gripper and avoid reflective ones, as they may disrupt the depth camera's ability to detect them using infrared light. Also, the scene cannot have direct sunlight present for the same reason.

For the robot to detect the position of each object and pick them, first, it is necessary for us to know the position of the camera relative to the arm. We can do this by manually measuring the offset between the camera's color optical frame and the robot's _'base_link'_. However, using this method is extremely **time-consuming** and prone to errors. Instead, we utilize the [apriltag_ros](https://github.com/Interbotix/apriltag_ros/tree/ros2-port) ROS 2 package to determine the transformation of the AprilTag visual fiducial marker on the arm's end-effector relative to the camera's color optical frame. Afterward, the transformation from the camera's color optical frame to the arm's _'base_link'_ frame is computed and then published as a static transform.

To run the perception pipeline, run the following launch command in a terminal (for more info refer to the [documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/perception_pipeline_configuration.html)):

```
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=px100 use_pointcloud_tuner_gui:=true use_armtag_tuner_gui:=true
```

Here, you can play around with the GUI a bit to get familiar with the perception pipeline (we will code everything and will not use the GUI, but it is good to get familiar with it). The first command is `use_armtag_tuner_gui:=true`, which will open up a Graphical User Interface (GUI) that will allow you to figure out where the position of the arm is with respect to the camera. In the Armtag tuner GUI, you can see a field named "num samples." This can vary from 1 to 10, with 10 indicating the highest level of accuracy. When you press the 'snap pose' button, the system will capture the specified quantity of images as defined in the **num samples** setting. The AprilTag algorithm will then run on each of the images, probably producing slightly different poses for the AprilTag's location relative to the camera. Finally, the program behind the GUI will average all those poses to hopefully obtain the most accurate position. As you will see in the GUI message the snapped pose represents the transform from the 'camera_optical_frame' frame to the 'px_100/base_link' frame. Try to verify this by manually measuring the portion of the camera w.r.t the base.  

**NOTE:** This pose will be saved to a YAML file in the perception directory named 'static_transform.yaml' after ROS is shut down. And if you do not change the setup configuration of the camera and the robot, you can always use this static transformation. 

At this point, you should see a **pointcloud** version of your tabletop with the objects on it. The second GUI applies some filters on the image in a way that all the objects are clear in our image. This GUI obtains the raw point cloud from our depth camera and applies several filters to it in a manner that makes the objects (clusters) visible. This GUI contains several sections for **filtering** the point cloud. The description for each of these sections is provided in the GUI windows, and you can also see [this guide](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/rolling/interbotix_perception_toolbox/interbotix_perception_modules#gui-options) for how to go about doing this. In general, this GUI will employ a mathematical model to identify the plane on which the objects are positioned. Then, it applies a Radius Outlier Removal filter to omit 'noisy' points in the point cloud.

**NOTE:** If you save your configurations after tuning them, these settings will also be saved in a YAML file named 'filter_params.yaml' in the perception directory.

To understand how to create a perception pipeline, you can follow the link below:

https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html

Now, let's go ahead and **implement our own code** for cluster detection using the camera and picking and placing by solving the numerical inverse kinematics of the arm at the **desired poses detected by the camera**.  

## Vision-aided Inverse Kinematics Code Guides

**Guide 1**. By now and for part 1, your should have changed the yaml file content to 'position' mode, if you have not done so, change the robot settings by copying the following to the yaml file:

```yaml
groups:
  arm:
    operating_mode: position
    profile_type: time
    profile_velocity: 2500
    profile_acceleration: 300
    torque_enable: true

singles:
  gripper:
    operating_mode: PWM
    torque_enable: true
```

You also have the **inverse kinematics script** from part 1, which we will use for this part. For this part, use the **numerical method only**(no need for the geometric one). This time, instead of designing the desired poses yourself, you will use the **camera's feedback** to **detect objects** and **estimate those desired poses**.  

**Guide 2.** After we developed our low-level API's in part 1, now we need to write down a new script that contains our **mission planner**. Let's call this script px100_vision_IK (or any name that you like). We will **import** our **custom-defined library** in this file, as well as the **vision module** and some other **basic libraries**, by adding the following imports:

```python
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from IK import ourAPI
import numpy as np
from math import atan2, sin, cos, pi
# You can use the time library if you ever need to make some delay. For example: time.sleep(3) 
import time
```

**Guide 3**. We now need to define some constants required for the vision module operation as well as the standard homogeneous transformation for the basket (or any object based on your design) pose. Remember that the vision module needs to define a number of reference frames: the **camera's reference frame** (the vision module references the 3D coordinate data with respect to the camera's optical center), the **arm tag reference frame** (the frame of the AprilTag attached to the robot arm, which is captured by the camera in the very beginning to determine where the end-effector stands from the camera's optical center), and finally the **robot's base frame** (which is required to reference everything with respect to the robot's base frame instead of the camera's optical center). This can be done as follows:

```python
# Start by defining some constants such as robot model, visual perception frames, basket transform, etc.
ROBOT_MODEL = 'px100'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'
Td_release = np.array([[],
                    [],
                    [],
                    []]) # Basket (or the appropriate object in your design) end-effector transformation. 
                         # I designed it to be constant, you can design it as an object. 
```

**Guide 4**. We now have all the global variables and dependencies that we need to design our **mission planner**; it is time to create our main function and start to create a **robot object**, a **cloud interface object** (for 3D point cloud and depth data generation), an **arm tag interface object** (for AprilTag identification), and an **inverse kinematics object**:

```python
def main():
    # Initialize the arm module along with the point cloud, armtag modules and px100_IK_ex custom API
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        group_name='arm',
        gripper_name='gripper'
    )
    pcl = InterbotixPointCloudInterface(node_inf=bot.core.robot_node)
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME,
        arm_tag_frame=ARM_TAG_FRAME,
        arm_base_frame=ARM_BASE_FRAME,
        node_inf=bot.core
    )
    my_api = ourAPI()
```

**Guide 5**. In the beginning, we need to make sure the arm's gripper is in the **release position** and that **we start from the sleep pose**. Append the following lines to the main code:
```python
    # set initial arm and gripper pose
    bot.arm.go_to_sleep_pose()
    bot.gripper.release()
```

**Guide 6**. The robot arm AprilTag should now be in the field of view of the camera, so now we attempt to solve the problem of finding the homogeneous transformation of the base with respect to the camera frame. The point cloud object would later use this information to reference the cubes to the robot base frame directly. Append the following lines to the main code:
```python
    # get the ArmTag pose
    armtag.find_ref_to_arm_base_transform() 
```

**Guide 7**. Now, it is time to get the **homogeneous transformations of the cubes (clusters)** relative to the robot base frame using the _.get_cluster_positions_ method. The method will sort the cubes based on how far they are along the x-axis of the base frame. Append the following lines to the main code:

```python
    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the ARM_BASE_FRAME
    success, clusters = pcl.get_cluster_positions(
        ref_frame=ARM_BASE_FRAME,
        sort_axis='x',
        reverse=True
    )
```

**Guide 8**. Create **bounds for the RGB values of the clusters**. The camera would surely pick **unwanted objects** such as wires, stains, etc., as clusters (you can verify this by running your code in debug mode and seeing the variables for clusters on the left-hand side where you will see that there are other clusters than the object that you intend to pick up). Therefore, we need to consider only **some colors**. In this example, we use a blue cube and, therefore, pick **RGB thresholds** for different shades of blue. You repeat this for other colors of clusters in your scene.

**Note: The clusters have a color property; you can see those by putting a breakpoint after the above code and looking at the cluster variables on the left-hand side to see their RGB values. Those can also give you an idea of the color space that the camera is seeing.**

```python
# Create a blue color bound
    # Define a range for blue color
    lower_blue = (0, 0, 15)
    upper_blue = (55, 75, 255)
```

**Guide 9**. Now, it is time to write down our mission planner. We need to loop over the clusters/cubes one at a time and do the following in order:
1. Determine the cube's (x,y) coordinate and add a **slight offset** to the **z coordinate** to avoid pushing the cube away while approaching.
2. Adjust the end-effector orientation so that its **x-axis is perfectly aligned with the horizontal axis** (in other words, the end-effector frame should point forward). The first joint (waist) should be rotated by the angle $`\theta`$, where $`\theta`$ could be computed as follows: $`\theta = atan2(x,y)`$, where x, and y are the detected cluster position x and y coordinates. This will make sure that the end-effector is oriented towards the object. **NOTE: This is again my solution. Your experiment may need other adjustments.** 
3. The end-effector will go down to hold the cube and then grasp it.
4. The end-effector will go back to the original slightly higher position to avoid hitting other cubes on its way to the basket.
```python
    if success:
        bot.arm.go_to_home_pose()
        # pick up all the objects and drop them in a basket (note: you can design your own experiment)
        for cluster in clusters:
            if all(lower_blue[i] <= cluster['color'][i] <= upper_blue[i] for i in range(3)):
                # Get the first cube location
                x, y, z = cluster['position']; z = z + 0.05 # Fingers link offset (change this offset to match your experiment)
                print(x, y, z)

                # Go on top of the selected cube
                theta_base = atan2(y,x) # Waist angle offset
                new_x = x/cos(theta_base)-0.01 #adjust this also to your own experiment. This is just an example. 
                # desired pose
                Td_grasp = np.array([[1,  0,  0,  new_x],
                        [0,  1,  0,  0],
                        [0,  0,  1,  z],
                        [0,  0,  0,  1]])
                
                joint_positions = my_api.num_IK(Td_grasp, np.array([0,0,0,0])) # Numerical inverse kinematics
                
                # Here, I rotated the waist by theta_base. This positioned my end-effector towards the cluster. 
                bot.arm.set_joint_positions(np.append(theta_base,joint_positions[1:])) # Set position         
    else:
        print('Could not get cluster positions.')
```

**Guide 10**. Finally, you could go to the sleep pose. Don't forget to add the main run line as follows:

```python
    # Go to sleep
    bot.arm.go_to_sleep_pose()
    bot.shutdown()


if __name__ == '__main__':
    main()
```

## Guidelines for Project 2 - Part 2

- Each group will give a **live demonstration** of their designed task (I will also specify which colors the robot should pick up, and you should be able to tweak the code fast).
- Be **prepared** for questions (both simulation and implementation) during the presentation. 
- Submit one code and video per group.

## Possible Challenges

- Recently (12/24), the RealSense camera topics had changed and caused a mismatch between the topics published by the camera and the topics expected by the robot. To resolve the issue, we mapped the camera's topics to the robot's required topic. See the instructions here (the code is written by Guangping Liu and Jake Little, two students in Fall 2024 class):

[Bug_fixes.pdf](https://github.com/user-attachments/files/18114846/Bug_fixes.pdf)

Note: in order to download a folder from Github, paste the folder path to the link below:

https://download-directory.github.io/

- **Object detection relies on color.** Changes in lighting can affect color perception, which may require adjusting the defined color ranges. You can run your code in the debug mode and put a breakpoint right after getting the clusters, and in the variables, see the clusters and their color property. 

- **Visibility of AprilTag and object.** Both must be clearly visible to the camera for accurate detection and positioning.

- **Handling IK failures.** The script anticipates potential failures in the numerical solution of the IK, indicating either unreachable poses or failure in detecting clusters.

## More on Image Processing: Deep Learning
So far, we have handled the Apriltag processing by using conventional approaches. These conventional approaches rely on some handcrafted features by experts. However, as will become apparent in your trials, handcrafted features often produce errors when the environment slightly changes. Slight changes in light intensity might affect the color thresholding, leading to the identification of the wrong cubes. The camera will not be able to perfectly capture the clusters if you're sitting in a dark room. **Deep neural networks** can automatically learn hierarchical representations and features from raw data. This allows them to adapt to the inherent complexity and variability in images, potentially uncovering intricate patterns that may be challenging for manual feature engineering. This is why modern literature relies heavily on deep learning techniques. The detection problem is basically treated as an **optimization problem**, where engineers feed huge amounts of examples (called **training datasets**) to a neural network. The neural network uses these training examples to learn new features that are hard to handcraft. One good example is the **YOLO (You only look once)** pipeline, which can be trained to detect virtually any object, including cars, pedestrians, animals, and so on (https://github.com/ultralytics/ultralytics). In fact, YOLO itself has so many versions that each version surpasses its predecessor in accuracy and speed. Some of the pipelines that are used for different applications are:

**YOLO (You Only Look Once):**

- Task: Object Detection
- Description: YOLO is known for its **real-time object detection** capabilities. It divides an image into a grid and predicts bounding 
boxes and class probabilities directly, making it efficient for real-time applications.

**Faster R-CNN (Region-based Convolutional Neural Network):**

- Task: Object Detection
- Description: Faster R-CNN introduced region proposal networks to generate potential bounding box proposals. It is widely used for accurate object detection.

**SSD (Single Shot Multibox Detector):**

- Task: Object Detection
- Description: Similar to YOLO, SSD is designed for real-time object detection. It predicts bounding boxes and class probabilities at multiple scales, providing a good balance between speed and accuracy.

**Mask R-CNN:**

- Task: Instance Segmentation
- Description: An extension of Faster R-CNN, Mask R-CNN adds an additional branch for predicting segmentation masks alongside bounding boxes and class probabilities. It is widely used for detailed instance segmentation tasks.

**U-Net:**

- Task: Image Segmentation
- Description: U-Net is commonly used for biomedical image segmentation. Its architecture includes a contracting path, a bottleneck, and an expansive path, allowing it to capture fine details in segmented images.

**DeepLab:**

- Task: Semantic Segmentation
- Description: DeepLab employs atrous convolutions and dilated convolutions to capture multi-scale context information, making it effective for semantic segmentation tasks.

**CycleGAN**:

- Task: Image-to-Image Translation
- Description: CycleGAN is a type of Generative Adversarial Network (GAN) that can learn mappings between different image domains without paired data. It has been used for tasks like style transfer and domain adaptation.

**StyleGAN:**

- Task: Image Generation
- Description: StyleGAN is a GAN architecture designed for generating high-quality and diverse images. It has been notably used for generating realistic faces.

**DeepDream:**

- Task: Image Synthesis
- Description: While not a pipeline for a specific task, DeepDream is known for its ability to generate artistic and hallucinogenic images by enhancing patterns and features in existing images using neural networks.

**RetinaNet:**

- Task: Object Detection
- Description: RetinaNet addresses the problem of class imbalance in object detection by introducing a focal loss. It achieves high accuracy by focusing on hard-to-classify examples.

These are just a few examples, and there are many more architectures and pipelines designed for various image processing tasks. The choice of a specific pipeline depends on the requirements of the task at hand, such as speed, accuracy, and the nature of the data.

## References for the Theoretical Parts

* Richard Szeliski, [Computer Vision: Algorithms and Applications](http://szeliski.org/Book/2ndEdition.htm)
* [Introduction to computer vision](https://www.williamhoff.tech/intro-to-computer-vision) by Prof. William Hoff
* [OpenCV documentation](https://docs.opencv.org/3.4/d6/d00/tutorial_py_root.html)

Good luck