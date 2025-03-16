## Introduction to computer vision 

The goal of **computer vision** is to **create machines that can see**. Computer vision engineering aims to develop **mathematical models** that can **explain and emulate** how **sight/vision works**. These **mathematical models** can be used to **enable computers to process images**. An **image** is a **static view of the world at a particular instant of time**. Therefore, **images need to be converted to a mathematical format**, as well. In this section, we will go through the very basics of computer vision before applying some of these concepts ourselves.

### Image formation 

**Image formation** refers to the process by which an **image is created**, either in the **human eye** or in **optical devices such as cameras**. In the context of optics, image formation typically involves the **interaction of light with lenses** or **mirrors** to produce a **focused representation of an object**. The result of this is the **projection of a 3D scene onto a 2D plane** (like a wall, a movie theater silver screen, or a photosensitive material, etc.). One thing we know from elementary school physics is that **light travels in straight lines** and **bounces off objects**, creating a **reflection**. This reflection might not always be clear, given the nature of the surface. The blurry reflection is sometimes attributed to the fact that a single point in the physical world reflects many light rays and not just one in different directions, so when these light rays fall on an object they interfere with other light rays coming from other points in the physical scene. This led to the **"pin-hole" camera invention**, which allows **light to pass through a very small hole**. This means that, theoretically, only a single light ray coming from a single point in the scene could pass to the other side and hit the wall. This way a-somewhat-clear, inverted image of the scene could be formed on the wall.

<figure>
<p align="center">
<img width="391" alt="pinhole_model" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/dd36a8c6-4e79-4881-b9f3-2d36cb3449b5">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

The pinhole camera model is great, but the problem is since only **one light ray** passes to the other side, the energy contained within this single light ray is small. If one is to place a photosensitive material on the image plane, it would take a long time to record a clear image of the scene on the material, since the photosensitive material's chemical reaction is proportional to light intensity. The common solution proposed to this problem is to use **lenses** that could manipulate light in way that could make **multiple light rays** travel from the same point in the physical scene and hit the same point on the image plane. **A lens performs the same projection as a pinhole, but with more light**.

<figure>
<p align="center">
<img width="469" alt="image_formation_with_lens" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/c75a1c74-f263-43c5-83f0-1af664ac6b22">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

In the lens case, we refer to the **focal length (f) as the ability of the lens to bend the light ray and focus it at a particular distance from the lens center** and it can be defined using the following mathematical equation:

$`\frac{1}{f} = \frac{1}{i} + \frac{1}{o}`$

where, $`i`$ is the distance between the center of the lens and the image plane, and $`o`$ is the distance between the lens center and the physical plane containing the original point.

### Image sensing

Apart from image formation, there's another term called **image sensing**. In the old days, photographers used photosensitive materials which respond to incoming light rays through a chemical reaction. This was merely an **analog process** that **could not be computerized**. Nowadays, we have **digital cameras** that **divide the image plane into small square-like areas called 'pixels'**. This means that the image can now be described as an **'array' of pixels** (very small squares with particular locations on the image plane) and each square represents both the **intensity** of the incoming light and its **wavelength/colors**. A **numerical value** can now be assigned to every pixel in terms of **intensity**, giving a chance for **computers to comprehend the image**. 

<figure>
<p align="center">
<img width="563" alt="image_sensing_digitization" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/8050134b-b2b4-4820-a384-645da0321fae">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**Image digitization** involves the **conversion of analog images** into **digital format** so they can be **stored and manipulated by computers**. This process relies on dividing the continuous variation in brightness and color of an image into pixels. **Each pixel is assigned a numerical value** representing its **brightness or color intensity**. The **resolution** of a digital image is determined by the **number of pixels** it contains, often expressed as **width × height** (e.g., 1920 × 1080 pixels for Full HD). This resolution determines the level of detail and clarity of the digital image.

<figure>
<p align="center">
<img width="563" alt="digital-image" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/221619f8-cab1-455a-ade8-efe16868614f">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

**RGB, or Red, Green, Blue**, is a **color model** used to **represent colors in digital images**. In this model, **colors are generated by combining different intensities of the three primary colors: red, green, and blue**. **Each primary color is assigned an intensity value** ranging from **0 to 255**, where **0 represents no intensity (black)** and **255 represents full intensity (maximum brightness)**. By varying the intensity levels of these primary colors, a wide spectrum of colors can be represented. The combination of red, green, and blue intensities at each pixel determines the overall color of that pixel in the image. This RGB color model is fundamental in digital imaging devices such as computer monitors, digital cameras, and image editing software, providing a standardized way to represent and manipulate colors in digital images.

<figure>
<p align="center">
<img width="550" alt="RGB color model" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/08c8deb7-d85d-44ac-a753-e538881ec271">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

## Introduction to OpenCV

OpenCV, short for **Open Source Computer Vision Library**, is an **open-source computer vision and machine learning software library**. It's designed to provide a **common infrastructure for computer vision applications** and to accelerate the development of **vision-based projects**. OpenCV offers the following utilities:

- **Image Processing and Computer Vision Algorithms:** OpenCV provides a vast array of functions for image processing and computer vision tasks. These include basic operations like **reading**, **writing**, and **displaying images**, as well as more advanced functionalities such as **edge detection**, **image segmentation**, **feature detection**, **object recognition**, and **camera calibration**.

- **Cross-Platform Support:** OpenCV is **written in C++**, but it has **interfaces for Python**, **Java**, and **other languages**, making it accessible and usable across various platforms and programming environments.

- **Community and Documentation:** OpenCV has a large and active community of developers and users who contribute to its development and provide support through forums, documentation, tutorials, and sample code. The library is well-documented, with comprehensive API references and tutorials to help users get started and learn how to use its features.

- **Machine Learning and Deep Learning Integration:** OpenCV **integrates with popular machine learning and deep learning libraries** such as **TensorFlow**, **PyTorch**, and **scikit-learn**, enabling users to **combine traditional computer vision techniques** with **modern machine learning approaches**.

***

### Installing OpenCV

You have already **installed Python** (you can check the current version by typing this command in your terminal: `python3 --version`) and **Visual Studio Code**, which is a versatile editor that can be used for programming in various languages including **Python**. The next step here is to set up OpenCV. 

The standard Python libraries are pre-installed in your Python environment, but OpenCV is not one of those libraries. Copy the command `pip3 install opencv-contrib-python` into your command prompt (or terminal on MAC) and hit 'Enter'. Now OpenCV should be part of your Python environment. 

After that, create a **folder** to store the python scripts that you are going to test in this part, call it something like 'CV basics' or anything you want. You can place this folder in any directory you like as well.

***

### Reading images using OpenCV

- **Create a folder** on an appropriate folder on your computer and call it for example 'CV basics' (you can choose whatever name you like). 
- Open Visual Studio Code, **choose the folder that you created**, and **Create a new .py file** (make sure to use .py after the name for VS code to recognize it as a Python file). 
- You can **save it** under the name 'read_img.py' in the 'CV basics' folder that you created earlier. 
- **Place an image file** **(jpg/png)** in the CV basics folder. We are going to **read that image into the IDE**. As we said before, any image is stored in a digital format. Copy the following code into your IDE:

```python
import cv2 as cv

img = cv.imread('name.png') # Read the image from png format to an array of 8-bit integers 
cv.imshow('name', img) # Show the image in window, given the image variable that has been stored
cv.waitKey(0) # Keep showing the image till a key is pressed
cv.destroyAllWindows() # Close all show windows
```
This code imports the OpenCV python library `cv2` and uses an alias `cv` in order to **make the name a bit shorter** (that's just for ease of use). We then **read the jpg/png file** (in this case I had an image called 'name.png' stored in my basics folder, you can change the name given your own photo). The code then shows the image on a separate window and keeps it **shown till the user presses any button to close it**.

Now in order to see the size of the `img` variable in the code above do the following: You can use the `.shape` attribute to get the dimensions of the matrix. For example for the `img` matrix: `print(img.shape)` 

By printing the array shape, you will see that it has **3 numbers**. The first number indicates the **height of the image in pixels**. The second number indicates the **width of the image in pixels**, and the third number indicates the **number of color channels in the image** (you can even verify this by seeing the properties of the image on your computer). If you used a color image, this number should be 3 and it means that the image is composed of **three channels** (Blue, Green, and Red components). 

The figure below shows a **3x3 region from an image**, showing the individual color channels for each pixel. The 3x3 grids in red, green, and blue each represent the intensity values for the respective color channel of each pixel in that region. In a full-color image, each pixel has a combination of red, green, and blue intensities, typically ranging from **0 to 255 in an 8-bit image**. 

<figure>
<p align="center">
<img width="366" alt="three_color_channels" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/32083863-d88a-47ab-990c-488c94d0437b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

***

### Resizing and scaling using OpenCV

Sometimes in image processing, we would like to change the **size of the photo or compress it**. This is usually done in **machine learning applications**, when certain features are required and those features are kind of 'big picture' features that do not require wasting time on a high-resolution image. **Reducing the size of the photo can help reduce computation time**. This also applied to many other applications. Let's resize the photo from our last exercise. Copy the following code into your code editor and run.

```python
import cv2 as cv

def rescale(image, scale = 0.5):
    # Compute the new width of the image (no. of columns)
    width = int(image.shape[1] * scale)
    
    # Compute the new height of the image (no. of rows)
    height = int(image.shape[0] * scale)
    
    # Create a tuple for the new set of dimensions
    dimensions = (width, height)
    
    # Return a resized version of the image, given the new dimensions
    return cv.resize(image, dimensions, interpolation = cv.INTER_AREA)

img = cv.imread('name.png') # Read the image from png format to an array of 8-bit integers 
cv.imshow('name', img) # Show the image in window, given the image variable that has been stored
cv.imshow('name_rescaled', rescale(img))
cv.waitKey(0) # Keep showing the image till a key is pressed
cv.destroyAllWindows() # Close all show windows
```
What this code does is that it implements a rescaling function called `rescale()`. This function takes a **copy of the image**, **counts the number of rows and columns**, then **multiplies that number by the scale** (set to 0.5 by default). The new size is **rounded to an integer**, because all image sizes have to be integer. A tuple is a container which contains the **new dimensions (width, height)**. OpenCV's built-in function `resize()` to **resize the image given the new dimensions** was used. Then the interpolation argument `INTER_AREA` for resizing is used. For more information about this API, you can refer to OpenCV's documentation. 

***

### Drawing shapes and writing text

In this part, you will learn how to **draw simple geometric shapes** and **write text on an image**. Things you used to do on paint as a kid, but this time you understand the **computerized description of an image** and can draw these shapes more accurately. I used a picture 'name' and I used this code to draw a green rectangular mask on the face. Then I drew a bounding box on the face along with a text label that says: 'Face'. **The following code pieces will do these for you. Make sure to run the parts independently to see what each part actually does.**

```python
import cv2 as cv
import copy

# Let's read the image again
img = cv.imread('name.png')
cv.imshow('name', img) # Show the original image

# 1. Let's change the color for a range of pixels (a rectangle on a character's face)
# Note: Here, I'm covering the character's face by manually selecting face pixels
img_face_mask = copy.deepcopy(img) # Make a deep copy of the original image
img_face_mask[200:400, 200:350] = 0,255,0 # Change a range of pixels to green value
cv.imshow('name_Face_Mask', img_face_mask) # Show the image with the green face rectangular mask

# 2. Let's instead draw a bounding box on name's face
# Note: Here, we will use OpenCV's rectangle drawing utility
image_face_bound = copy.deepcopy(img) # Make a deep copy of the original image
cv.rectangle(image_face_bound, (200,200), (350, 400), (0,255,0), thickness=3)
        # Draw a rectangle around the character's face with a thickness of three pixels
cv.imshow('name_Face_Bounded', image_face_bound) # Show the image with the bounding box

# 3. Finally, let's write some text. For example, a label for the character's face
# Note: We use putText OpenCV API to achieve that (check the documentation)
cv.putText(image_face_bound, 'Face', (225,225), cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,255,0), 2)
cv.imshow('name_Face_Bounded_Text', image_face_bound)
  
cv.waitKey(0)
cv.destroyAllWindows() # Close all show windows
```
In this code, we start by importing OpenCV library for Python and another library called 'copy'. The copy library will be used to obtain deep copies of a variable that could be manipulated without changing the value of the original variable. First, we start by reading and showing the original image. Next, we manipulate the **values of a range of pixels** that forms a rectangle and change them to green (change the color if your like by changing the RGB values). In this code, I again used the image named "name", and I manually defined the indices and dimensions of that rectangle to cover the face (**from pixel number 199 to pixel number 399 (height), and pixel number 199 to pixel number 349 (width)**). Feel free to find the proper array indices for your own image of choice. I then displayed the face-covered image. Then I decided to draw a bounding box over the same region of thickness 3 pixels, instead of covering the whole face (This is what is done automatically in face detection scripts, but here I did it manually). Finally, I used the `puText()` API to write a label on a specific region in the image included within the bounding box. The index (225,225) defines where the first letter of the text should show up. You can check the documentation of the used API's here: https://www.geeksforgeeks.org/python-opencv-cv2-rectangle-method/ and here: https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/. 

***

### Basic functionalities in OpenCV

***

**Conversion from the BGR color representation to the grayscale representation:**

Now, let's introduce some of the very basic API's that are almost used in every OpenCV script. I will first explain the theory behind these operations, then write a code to achieve them. The most famous function is the **conversion from the BGR color representation to the grayscale representation**. This is done when we're not interested in the image colors, but mostly interested in other geometric features like **edges**. The **BGR color space** is a common representation of color images in computer vision and image processing, especially in libraries like OpenCV. In BGR, each pixel is represented by three color channels: Blue (B), Green (G), and Red (R). Converting from BGR to grayscale involves reducing the color information of each pixel to a **single intensity value**. This is typically achieved by combining the color channels in a weighted average, where the weights are chosen to reflect the human perception of brightness. The resulting intensity value represents the brightness of the pixel. One common method for **converting from BGR to grayscale** is to use the following formula:

$`Gray=0.299Red+0.587Green+0.114Blue`$

This formula is based on the **luminance formula**, which takes into account the **different sensitivities of the human eye to different colors**. The coefficients 0.299, 0.587, and 0.114 represent the **perceived brightness** of the red, green, and blue components, respectively. **Alternatively**, OpenCV provides a more efficient method for converting from BGR to grayscale using the `cv2.cvtColor()` function with the conversion code `cv2.COLOR_BGR2GRAY`. This function **internally performs the weighted average calculation** and handles any necessary data conversions.

***

**Blurring function:**

Another common function is the **blurring function** which relies on a **gaussian kernel**. Gaussian blur, a fundamental image processing technique, operates by **convolving an image with a Gaussian kernel**, an arrangement of numbers derived from the **Gaussian distribution**. Understanding the Gaussian distribution is crucial; it characterizes the **spread of intensity values in an image**. Its parameters, the mean $`(\mu)`$ and standard deviation $`(\sigma)`$, signify the distribution's center and spread, respectively.

To **create the Gaussian kernel**, one generates a matrix of values based on the Gaussian function. The kernel's size, typically odd and square (e.g., 3x3, 5x5), determines the **extent of blur**, with larger sizes producing more pronounced effects. Notably, the kernel's values adhere to the Gaussian distribution, peaking at the center and diminishing symmetrically towards the edges.

Convolving the Gaussian kernel with the input image involves **sliding** the kernel across the image's pixels and computing weighted sums within the kernel's neighborhood. The Gaussian kernel assigns higher weights to pixels closer to its center, reflecting the distribution's bell-shaped curve. Consequently, each output pixel is calculated as a weighted average of its neighbors, resulting in a blurred rendition of the original image.

The standard deviation, $`(\sigma)`$, significantly influences the **blur's intensity**. A larger $`(\sigma)`$ leads to wider spreads of intensity values in the Gaussian kernel, causing more aggressive smoothing. Conversely, a smaller $`(\sigma)`$ yields narrower spreads and less blur. Thus, adjusting the standard deviation enables fine-tuning of the blurring effect based on the desired level of image smoothing.

In essence, **Gaussian blur** **using kernels** harnesses the **Gaussian distribution's properties** to achieve noise reduction and detail smoothing in images. By modulating the Gaussian kernel's standard deviation, one can tailor the blur's intensity to suit specific image processing requirements.

The figure below shows the concept. You see that there's a 3x3 grid representing a small section of an image, with the central pixel highlighted. In the center, there's another 3x3 grid that represents the **convolution kernel that we sometimes call a filter or mask**. On the right, there's a single highlighted cell which shows the **new value** for the **central pixel** after **applying the convolution operation**. **The operation involves multiplying each element of the kernel with the corresponding pixel value from the image and then summing all these products together to produce a single output pixel value.**

<figure>
<p align="center">
<img width="366" alt="gaussian_kernel" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/95638a35-c615-4cc9-80fb-e7b5f3ae5ac9">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

***

**Image Features Extraction:**

Sometimes, we need to **extract strong features from an image**. A **feature** represents something special, such as a **sharp edge** or a **specific color**. We are mostly interested in **edges** because they **represent the boundaries** between different objects and can help us **separate objects** in the image. 

The **Canny edge detector**, developed by John F. Canny in 1986, is an **image processing technique widely used for edge detection** due to its effectiveness and robustness. It comprises several key steps, starting with **Gaussian smoothing** to reduce noise and preserve significant edges. This involves convolving the image with a Gaussian kernel, which is a matrix of numbers derived from the Gaussian distribution. Following Gaussian smoothing, the algorithm computes the **gradient magnitude and direction at each pixel**. The **gradient magnitude** represents the **rate of intensity change** in the image, while the **gradient direction** indicates the **direction of the most rapid change**. This information is crucial for **identifying edges**. Remember that the **gradient is a vectorized derivative that contains the magnitude of the rate of change as well as its strongest direction**. **Non-maximum suppression** is then applied to **thin out the edges** and retain only **local maxima**. At each pixel, the **gradient direction** determines the **direction perpendicular to the edge**. The pixel is compared with its neighbors in this direction, and if it is not the maximum, it is suppressed. Next, **double thresholding** is used to **classify pixels into strong edges, weak edges, and non-edges**. Pixels with **gradient magnitudes above a high threshold** are considered **strong edges**, while those **below a low threshold** are classified as **non-edges**. Pixels **between the thresholds** are considered **weak edges**. Finally, **edge tracking by hysteresis** links weak edges to strong ones and suppresses noise. Starting from strong edge pixels, neighboring weak edge pixels are recursively traced along the edge direction. If a weak edge pixel is connected to a strong edge pixel, it is classified as a strong edge. Otherwise, it is classified as a non-edge. This process completes **edge contours** and suppresses isolated weak edges caused by noise.

The image below depicts a **flowchart of the Canny edge detection algorithm** that outlines its **five key steps**: **applying a Gaussian filter**, **calculating gradients in the x and y directions**, **computing the intensity gradient** and **edge direction**, **performing non-maximal suppression**, and finally **applying hysteresis thresholding**.

<figure>
<p align="center">
<img width="366" alt="flowchart of the Canny edge detection" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/2be9f07c-bc15-410b-a3f5-535849d4a65e">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>

After **extracting features like edges**, we need to make the edges a bit **more clear and continuous**. We also need to **remove weak edges**. This can be done using **dilation and erosion**. Dilation and erosion are fundamental morphological operations in image processing used for **manipulating the shape and structure of objects** within an image. When applied to edge detection, dilation and erosion can enhance or refine the detected edges based on their structural characteristics.

**Erosion**:

Erosion is a morphological operation that **shrinks** or **thins** the boundaries of **foreground objects** in an image. When applied to edge detection, erosion can help to refine and sharpen the detected edges by removing small, spurious edges and smoothing out irregularities. This is achieved by sliding a **mask** (also known as a **kernel**) over the image and replacing each pixel with the minimum pixel value within the neighborhood defined by the kernel. As a result, pixels near the edges of objects are eroded away, causing the edges to retreat inwards and become thinner. Erosion is particularly useful for **cleaning up noisy edges** and eliminating small protrusions or irregularities that may have been introduced during the edge detection process.

**Dilation**:

Dilation, on the other hand, is a morphological operation that **expands** or **thickens** the **boundaries of foreground objects** in an image. When applied to edge detection, dilation can help to **enhance and strengthen the detected edges** by filling in gaps and extending the edges to connect disjointed segments. This is achieved by sliding a kernel over the image and replacing each pixel with the **maximum** pixel value within the neighborhood defined by the kernel. As a result, pixels near the edges of objects are dilated or expanded, causing the edges to grow outwards and become thicker. Dilation is particularly useful for **closing small gaps between edge segments**, connecting broken edges, and making the edges more continuous and robust.

**Combining Erosion and Dilation**:

In practice, erosion and dilation are often **used together** in sequence, a process known as **opening and closing**, to achieve specific effects on the edges. **Opening**, which consists of an **erosion followed by a dilation**, can help to **remove small objects** and **smooth out the edges**, while preserving the overall shape and structure of **larger objects**. **Closing**, which consists of a **dilation followed by an erosion**, can help to **fill in gaps** and **connect broken edges**, while preserving the overall size and shape of the objects. By carefully adjusting the size and shape of the kernel, as well as the number of iterations, erosion and dilation can be effectively tailored to **enhance the quality** and **robustness** of the **detected edges in an image**.

***

Now we need to apply all of these operations in OpenCV using one-line commands. Write the following code in your VS code (**remember to do it one function at a time to see what each function is doing**):

```python
import cv2 as cv

# Read in an image
img = cv.imread('name.png')
cv.imshow('name', img)

# Converting from BGR to grayscale
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('name_Gray', gray)

# Blur 
# Kernel size is 7*7, increasing size increases blurring
blur = cv.GaussianBlur(img, (7,7), cv.BORDER_DEFAULT)
cv.imshow('name_Blur', blur)

# Edge Cascade (documentation: https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga04723e007ed888ddf11d9ba04e2232de)
canny = cv.Canny(blur, 125, 175)
cv.imshow('name_Canny Edges', canny)

# Dilating the image (kernel size = 7*7, number of iterations = 3)
dilated = cv.dilate(canny, (7,7), iterations=3)
cv.imshow('name_Edges_Dilated', dilated)

# Eroding (kernel size = 7*7, number of iterations = 3)
eroded = cv.erode(dilated, (7,7), iterations=3)
cv.imshow('name_Edges_Dilated_Eroded', eroded)

cv.waitKey(0)
cv.destroyAllWindows() # Close all show windows
```
***

### Image transformations

Sometimes, we need to change the **view of the image using homogeneous transformations** such as **translation and rotation**. A **transformation matrix** is a mathematical concept used to **rotate/translate different objects** with respect to some frame and is heavily used in **robotics**.

***

**2D Translation Matrix**:

A **2D translation matrix** is used **to shift or move a point or object in a 2D coordinate system**. It is represented as a **3x3 matrix**, where the first two columns correspond to the x and y axes, and the third column represents the translation amounts in the x and y directions. The translation matrix is typically denoted as:

$`T = \begin{pmatrix}
1 & 0 & t_x \\
0 & 1 & t_y \\
0 & 0 & 1
\end{pmatrix}`$

Where $`(t_x)`$ and $`(t_y)`$ are the **translation** amounts along the x and y axes, respectively. To apply the **translation matrix** to a point $`(x,y)`$, we use **matrix multiplication**:

$`\begin{pmatrix}
x' \\
y'\\
1
\end{pmatrix} = \begin{pmatrix}
1 & 0 & t_x \\
0 & 1 & t_y \\
0 & 0 & 1
\end{pmatrix}\begin{pmatrix}
x \\
y\\
1
\end{pmatrix}`$

The resulting point $`(x',y')`$ represents the **translated coordinates of the original point** $`(x,y)`$ after applying the translation.

***

**2D Rotation Matrix**:

A **2D rotation matrix** is used to **rotate a point or object around the origin in a 2D coordinate system**. It is represented as a 2x2 matrix, where the elements of the matrix correspond to the **cosine and sine of the rotation angle**. The rotation matrix is typically denoted as:

$`R = \begin{pmatrix}
cos\theta & -sin\theta \\
sin\theta & cos\theta
\end{pmatrix}`$

Where $`(\theta)`$ is the **rotation angle in radians**. To apply the rotation matrix to a point $`(x,y)`$, we use **matrix multiplication**:

$`\begin{pmatrix}
x' \\
y'
\end{pmatrix} = \begin{pmatrix}
cos\theta & -sin\theta \\
sin\theta & cos\theta
\end{pmatrix} \begin{pmatrix}
x \\
y
\end{pmatrix}`$

The resulting point $`(x',y')`$ represents the **rotated coordinates** of the original point $`(x,y)`$ after applying the rotation. 

Write the following **image transformation code** to your VS code and view the output (**make sure to do one function at a time to see how they work and try to change the translation and rotation amounts**):

```python
import cv2 as cv
import numpy as np # This library is used for matrix operations

img = cv.imread('name.png')
cv.imshow('name', img) # original image

# Translation function
def translate(img, x, y):
    transMat = np.float32([[1,0,x],[0,1,y]]) # translation matrix as a numpy array of floats
    dimensions = (img.shape[1], img.shape[0]) # output image dimensions
    return cv.warpAffine(img, transMat, dimensions) # Affine transformation function applied translation 

# -x --> Left
# -y --> Up
# x --> Right
# y --> Down

translated = translate(img, -100, 100)
cv.imshow('Translated', translated)

# Rotation function
def rotate(img, angle, rotPoint=None):
    (height,width) = img.shape[:2]

    if rotPoint is None:
        # If no rotation point is specified, rotate about the center of the image
        rotPoint = (width//2,height//2)
    
    rotMat = cv.getRotationMatrix2D(rotPoint, angle, 1.0) # rotation matrix based on rotation point, angle and scale
    dimensions = (width,height)

    return cv.warpAffine(img, rotMat, dimensions) # Affine transformation API applied rotation on the image, given the rotation matrix

rotated = rotate(img, -45)
cv.imshow('Rotated', rotated)

rotated_rotated = rotate(img, -90)
cv.imshow('Rotated Rotated', rotated_rotated)

cv.waitKey(0)
cv.destroyAllWindows()
```

***

### Masking

One of the basic operations frequently used in OpenCV is to use a **mask image** to **cover some parts of an original image** while showing some other parts. This is called **masking**, because a mask basically covers some parts of the face while showing others. This operation can simply be done using **logic operations** such as `AND` and `OR`. Write the following code into your code editor and see how it functions:

```python
import cv2 as cv
import numpy as np

img = cv.imread('name.png')
cv.imshow('name', img) # original image

# Create a white circle in a blank image
blank = np.zeros(img.shape[:2], dtype='uint8') # new blank image (same size as original )
cv.imshow('Blank Image', blank)
circle = cv.circle(blank.copy(), (img.shape[1]//2 + 45,img.shape[0]//2), 100, 255, -1)

# Use masking API to show only the character's face (using the blank image mask)
masked = cv.bitwise_and(img,img,mask=circle)
cv.imshow('name_Masked', masked)

cv.waitKey(0)
cv.destroyAllWindows()
```

***

### More on edge detection

The **Laplacian operator**, often denoted as $`(\nabla^2)`$, is a **differential operator** used in image processing for edge detection and image enhancement. It calculates the **second derivative** of an image to highlight **regions of rapid intensity change**, which typically correspond to **edges**. Here's a brief overview of the theory behind the Laplacian operator in image processing:

- **Second Derivative**: The Laplacian operator computes the **second derivative of an image** with respect to spatial coordinates (x and y in 2D). Mathematically, it is defined as the **sum of the second partial derivatives of the image intensity function**:

$`\nabla^2 f(x, y) = \frac{\partial^2 f}{\partial x^2} + \frac{\partial^2 f}{\partial y^2}`$

This operation measures the **rate of change of intensity within the image**. High positive values indicate regions of rapid increase in intensity (e.g., edges), while high negative values indicate regions of rapid decrease.

- **Edge Detection**: By calculating the Laplacian of an image, areas with significant intensity variations (i.e., edges) are highlighted. Edge pixels typically have **large positive** or **negative** Laplacian values, making them easy to detect.

- **Image Enhancement**: In addition to edge detection, the Laplacian operator can be used for image enhancement tasks such as **sharpening**. By **adding the Laplacian of the image to the original image**, details and edges are emphasized, leading to a sharpened appearance.

- **Implementation**: In practice, the Laplacian operator is applied using **convolution** with a **Laplacian kernel**, such as the 3x3 or 5x5 **Laplacian filter**. This kernel approximates the Laplacian operator by sampling the **second derivative** at discrete points in the image. For example, a simple 3x3 Laplacian kernel is:

$`\begin{pmatrix}
0 & 1 & 0\\
1 & -4 & 1\\
0 & 1 & 0
\end{pmatrix}`$

This kernel calculates the Laplacian at each pixel by taking a weighted sum of its neighbors, with the center pixel having a negative weight to emphasize the central pixel's importance. **Write the following code** to the code editor to perform **image sharpening** (again make sure to write it one function at a time to understand what it does):

```python
import cv2 as cv
import numpy as np

img = cv.imread('name.png')
cv.imshow('name', img) # Original image

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('name_Gray', gray) # Gray scale image

# Apply Gaussian blur to reduce noise
blurred = cv.GaussianBlur(gray, (3, 3), 0)

# Laplacian
lap = np.uint8(np.absolute(cv.Laplacian(blurred, cv.CV_64F)))
cv.imshow('Laplacian', lap)

# Add the Laplacian image to the original image to perform image sharpening
sharpened = cv.addWeighted(gray, 1.0, lap, -1.0, 0)

# Display the original and sharpened images
cv.imshow('Original Image', gray)
cv.imshow('Sharpened Image', sharpened)
cv.waitKey(0)
cv.destroyAllWindows()
```
After this short introduction to machine vision, let's get down to business with our own wheeled mobile robot. 

## Face Detection Using Your Wheeled Robot's Camera

In this section, we are going to write some code to **interface with the robot's camera** and **get successive frames** for processing on our PC. 

- **First, make sure that you connect to your robot's WiFi network**. This can typically be done by finding the network name (SSID) specific to your robot in the WiFi settings of your computer. Your wheeled robot is equipped with an **ESP32 WiFi module** for remote control and data communication that is integrated into the camera module. 

- Then, write the following code to your code editor and play around with it like adding some image processing that you have learned earlier in this lesson. Also, pay attention to the IP used to connect to the camera's wifi network. 

```python
import cv2 as cv
import numpy as np
from urllib.request import urlopen

# Capture image from camera
cmd_no = 0 # Frame number

def capture():
    global cmd_no # Variable declared as global to be accessed inside function
    cmd_no += 1 # Increment the frame number
    print(str(cmd_no) + ': capture image')
    cam = urlopen('http://192.168.4.1/capture') # Connect to the camera WIFI network
    img = cam.read() # Read the current camera frame
    img = np.asarray(bytearray(img), dtype = 'uint8') # Convert the image numerical values to 8-bit integers
    img = cv.imdecode(img, cv.IMREAD_UNCHANGED)
    cv.imshow('Camera', img) # Show the image
    cv.waitKey(1)
    return img

# Infinite capturing till the user stops the program (usually Ctrl + C)
while 1:
    # Capture image
    img = capture()
    
    # Do some processing on the image (up to you!)
```

The provided code aims at **establishing a Wifi connection with the robot's camera**. The code's core is the `capture` function, which **converts the image to the 8-bit standard `numpy` array format and displays the frames one by one**. The code infinitely captures frames till you terminate the program (usually using Crtl + C). The while loop leaves some space for extra processing on the image. **Feel free to add any of the previous (or even new) OpenCV codes to process the image for decision making in your project**. 

I will shortly show you one OpenCV example using the robot's camera for **face detection**. **First you need to download the 'haar_face.xml' file from [HERE](https://drive.google.com/file/d/1Vg-7ErLFAvOXiNMI1i8M57PKraZef15-/view?usp=sharing) and put in your directory**. This xml file contains the **weights** for the **face classifier** that we're going to use for this exercise. Now, write the following code to your editor and stand in front of the camera with your face clear.

```python
import cv2 as cv
import numpy as np
from urllib.request import urlopen

# Capture image from camera
cmd_no = 0 # Frame number

def capture():
    global cmd_no # Variable declared as global to be accessed inside function
    cmd_no += 1 # Increment the frame number
    print(str(cmd_no) + ': capture image')
    cam = urlopen('http://192.168.4.1/capture') # Connect to the camera WIFI network
    img = cam.read() # Read the current camera frame
    img = np.asarray(bytearray(img), dtype = 'uint8') # Convert the image numerical values to 8-bit integers
    img = cv.imdecode(img, cv.IMREAD_UNCHANGED)
    return img

# Infinite capturing till the user stops the program
haar_cascade = cv.CascadeClassifier('haar_face.xml') # Load face classifier 
while 1:
    # Capture image
    img = capture()
    
    # Do some processing on the image (up to you!)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Convert to gray scale
    faces_rect = haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=1) # Detect faces using loaded classifier

    print(f'Number of faces found = {len(faces_rect)}') # Print the number of faces detected in the current frame

    for (x,y,w,h) in faces_rect:
        cv.rectangle(img, (x,y), (x+w,y+h), (0,255,0), thickness=2) # Draw a bounding box around detected faces

    cv.imshow('Detected Faces', img) # Show the image with the detected face
    cv.waitKey(1)
```

In this code, I used the processing region that we left blank earlier to add our face detection code. I started by **converting the image to grayscale**, then I used **the loaded classifier whose weights have been imported** from the previously mentioned .xml file to **detect faces** in each and every frame captured by the camera. I then displayed the **total number of detected faces** and use a for loop to draw a bounding box around each face detected in the frame.

## Suggestions for Ongoing Project

After successfully implementing face detection using the robot's camera, the next natural step is to **integrate this vision capability with robot control**. For this, there should be **a communication link established between the vision processing system on Python and the robot's control mechanism implemented on Arduino**. This can be achieved **using methods like WiFi, Bluetooth, or serial communication**, allowing the robot to receive and execute commands based on the visual data it receives. 

**Note:** Due to **limited processing power and memory capacity on Arduino boards**, we **had to process images on a computer**. The Arduino platform, while excellent for basic control tasks and interfacing with sensors and motors, lacks the necessary capabilities to **handle complex image processing tasks such as face detection**. Therefore, **image processing is performed on a more powerful computer, and the results are communicated back to the Arduino** to make decisions and control the robot accordingly. 

To facilitate **communication between the computer vision system and your robot**, you can use various **communication protocols** such as **Bluetooth, Serial Communication, or WiFi.** Each method has its advantages depending on the application's requirements in terms of range, data throughput, and ease of setup.

For the future, you can also experiment with **object detection and color tracking** to further enhance the robot's capabilities. By implementing object detection, the robot can recognize and interact with a variety of items **beyond just human faces**, such as pets, vehicles, or specific objects relevant to its environment. Additionally, color tracking allows the robot to follow or react to objects based on their color. 

## Guidelines for the part 3 report

- Screenshots for the machine vision part (put them in order in text editor with suitable titles for each figure), and a video of the face detection part is the submission for this part. 
- The network connection between the computer and Arduino is not required for this project due to time limitations but if you ended up doing that even with Serial communication and could control the robot movement based on the vision input, you can get **extra credit** for that. Here is a schematic of what to do for the Serial communication:

<figure>
<p align="center">
<img width="366" alt="python_to_arduino" src="https://github.com/madibabaiasl/mechatronics-course/assets/118206851/900e05b4-2b40-4248-bf3a-e55ed68a360b">
<figcaption> <p align="center"></figcaption> </p>
</p>
</figure>
  
Good luck!