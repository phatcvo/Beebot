## Video Version of the Lesson

https://youtu.be/hG2uOzL9Vc4

## Introduction

This lesson will refresh your knowledge of Python programming, particularly as it relates to robotics applications. Due to the fact that we will use Python to program our robot arm, I thought it would be nice if you refreshed your memory about the topic from a robotics perspective. So, let’s get started. 

Python is one of the most popular programming languages because it is easy to use and beginner-friendly. Python's core functionality can be extended with packages that can be downloaded, installed, and imported. Therefore, it is highly extensible through packages, giving the user instant gratification. 

[PyPI](https://pypi.org/) is a Python package index which is the main repository for 3rd party Python software. It has over 300k packages that can be simply installed using the pip command. 

<figure>
<p align="center">
<img width="678" alt="python for robotics_python packages" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/d49b2faa-cd62-445a-9434-2c586c9391e4">
<figcaption> <p align="center">(Left) In this lesson, we want to refresh our knowledge of Python programming with applications in robotics, (Right) Python's core functionality can be extended with packages.</figcaption> </p>
</p>
</figure>

## Installing Python and the Code Editor

Since we will be using Ubuntu 22.04 to run our ROS2 and to code, then first we make sure that we have Ubuntu on our computer (you should have installed it by now if not refer to [the first lab](https://github.com/madibabaiasl/modern-robotics-I-course/wiki/Lab-1:-Introducing-the-PincherX-100-Robot-Arm-and-Vision-Kit)). 

Then we need a code editor to write our Python programs. In the past, people were using just text editors to write their programs but thanks to the current code editors, we can get the most of their debugging properties. These code editors are called IDEs or integrated development environments. 

There are different IDEs and some are specially designed for Python. In this class, we are going to use <a href="https://code.visualstudio.com/">Visual Studio Code editor from Microsoft</a> and it has the advantage that you can organize your codes in different languages in one place and it is the most widely used code editor among programmers. Other options are PyCharm and Spyder.

Here is the Spyder tutorial in case you are interested (**but we will use VS code**):
https://www.youtube.com/watch?v=HOH6tMriJRc

<figure>
<p align="center">
<img width="797" alt="code-editers-ides" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/64c16519-6e3a-4cb4-bd6b-60ecf9ef5b90">
</p>
</figure>

Steps to install VS code and Python in Ubuntu 22.04:
- Open a terminal: `ctrl + alt + T`
- Type: `sudo snap install code - - classic` This is the same as going to the software center and typing visual studio code and then installing it.
- Head over to all apps and add the VS Code to your favorites and then run it.
- Go to the extensions tab and search for Python and get the recommended Python extension.
- Head over to the gear icon, then extension settings, then configure it as you want like setting your desired path for things like Conda virtual environments. 

**Note:** A virtual environment is like the Construct in the movie Matrix where Morpheus trained Neo and the reason to use them is that some packages that you may have developed for different projects have conflicting dependencies (for instance if you want to work on a ML project using Python3 and the web development project on Python2, this is not possible since the system can have only one default Python. In this case, you will need to install a virtual environment). Learn more about virtual environments and how to set them at the link below:

https://youtu.be/3YTGdJApYho

- You can also install a C/C++ extension if you need to program in those languages as well or if you want to use ROS packages that use C++.
- Write your very first program and test your IDE:
  - Open a new folder and give it a name. I named it Intro to Python
  - Create a new Python file and name it hello_world
  - Write:
``` Python
# This is the first program
print(“Hello World”)
```

And you will see that Hello World will be printed on the console. The first line that starts with # is just for human users to understand the code and it is called a comment. “ “ is used for strings. `print` will print anything that you pass to it on the console. 

## Variables and data types in Python

A variable is like a container that stores data. It can be a number, a text, or more complex data types. In most programming languages like C++, you need to declare a variable before using it but in Python, you just give it a name and assign a value to it. Different data types that can be stored in variables are:

- Numbers: different types like integers and floats
- Boolean value: True or False
- Strings: plain texts
- Lists
- Tuples 
- Dictionaries 

![different data types in python](https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/95367a60-4740-4601-a9b8-d654a5cac0a3)

### Numbers and Strings 

Let's first talk about the simplest data types in Python: numbers and strings. Suppose we want to create a variable named num_int and assign the value of 10 to that variable. Here is how we can do it:

``` Python
num_int = 10 # this is an integer
```

Or we can assign a float number to a variable:

``` Python
num_float = 0.5 # this number is a float
```

We can do basic arithmetic + - / * () **(power) with numbers:

``` Python
print(num_int * (num_int + num_float))
```

We can find the remainder of the division using the modulus operator n mod m = n % m:

``` Python
print(num_int % 3)
```

There are different math functions (Collections of code that we run and can perform a basic operation for us. Functions can either modify our data or give us some information about the data) that we can use on our numbers:

- max will give us the larger of the two numbers that we give to it
``` Python
print(max(num_int, 9))
```

- min is the opposite
- abs finds the absolute value of a number

``` Python
print(abs(-num_int))
```

- pow gets the first number to the power of the second number

``` Python
print(pow(num_int, 2))
```

- round will allow us to round a number either round down or up

``` Python
print(round(num_float))
```

There are some other math functions that should be imported to get access to them. `math` is a module and by the following code we can have access to those functions inside the `math`:

``` Python
from math import *
```
This means that you can now access all of the functions in the `math` library (a library of functions that are coded by other people and we don’t need to recreate them) without having to prefix them with `math.`.

- `floor` will chop off the decimal point
``` Python
print(floor(num_float))
```

- `ceil` will round the number up
``` Python
print(ceil(num_float))
```

- `sqrt` is the square root
``` Python
print(sqrt(num_int))
```

If you write `import math`, then to access the functions inside the math module, you need to, for example, say `math.exp()` to access the exponential function:

``` Python
print(math.exp(num_int))
```

You can also import a library as a preferred name for simplicity and of course if you’re lazy:

``` Python
import math as m
print(m.exp(num_int))
```

Now let’s combine strings and numbers. Type the following command in the VS code:

``` Python
print(“There are “ + str(num_int) + “ arm robots in the factory, “ + str(num_float) + “ of them are broken.”)
```

The `str()` function creates a string version of the object (any Python data type) passed as an argument to it. It can be used to convert an integer into a string. The output of the above code is:

There are 10 arm robots in the factory, 0.5 of them are broken. 

Note that here we concatenated different strings meaning that we added different strings together.

You can also write this like the following and it will give the same result:

``` Python
print(“There are “, num_int, “ arm robots in the factory, “, num_float, “ of them are broken.”)
```

Note: \n will create a new line inside a string. 


You can assign a string to a variable:

``` Python
factory_name = “ABC”
print(“The factory’s name is “ + factory_name)
```

Note that here you do not need to use the `str()` function since the `factory_name` variable is already of the data type string. 

You can access different characters in a string using [] notation:

``` Python
print(“The first character in the factory name is ” + factory_name[0])
print(“The third character in the factory name is ” + factory_name[2])
```

Note that indexes in Python start from 0.

We can use different functions with strings:

- Make the factory name all lower:

``` Python
print(factory_name.lower())
```

- Check to see if it is lower:

``` Python
print(factory_name.islower())
```

We can do similar with .upper() and isupper() functions. 

You can also combine different functions:

``` Python
print(factory_name.upper().isupper())
```

This will first convert the phrase to upper case and then we will get `True` value because it checks to see if it is all upper. 

We can figure out the length of the string using `len()` function:

``` Python
print(len(factory_name))
```

We can also access individual characters using the [] notations. Here I want to access several characters:

``` Python
print(factory_name[0:len(factory_name)-1])
```

This will show the characters from the first character up to one to the last character.

To know where a specific character is we use the `index` function and we can pass it a parameter:

``` Python
print(factory_name.index("A"))
```

We can give the `replace` function two parameters, the first one what you want to replace and the second one the thing that you want it to be replaced with:

``` Python
print(factory_name.replace("ABC", "DEF"))
```

**The whole code of this section is:**

``` Python
from math import * 

import math as m

num_int = 10 # this is an integer
num_float = 0.5 # this is a float

# we can do basic arithmitic + - / * ** (for power)
print(num_int * (num_int + num_float))

# % is the modulus operator --> gives the remainder of a division
print(num_int % 3)

# max and min --> larger or smaller of the two numbers
print(max(num_int,9))

# abs --> absolute value
print(abs(-num_int))

# pow gets the 1st num to the power of the second num
print(pow(num_int,2))

# round --> rounds a number down or up
print(round(num_float))

# floor will chop off the decimal point
print(floor(num_float))

# ceil will round the number up
print(ceil(num_float))

# sqrt --> square root
print(m.sqrt(num_int))

# combining numbers with strings
print("There are " + str(num_int) + " arm robots in the factory, "+ str(num_float) + " of them are broken.") 

# or you can write this like the following
print("There are",num_int,"arm robots in the factory,",num_float,"of them are broken.") 

# using string variables
factory_name = "ABC"
print("The factory's name is " + factory_name)
# accessing different characters in a string
print("The first character in the factory name is " + factory_name[0])
print("The third character in the factory name is " + factory_name[2])

# using different functions with strings
print(factory_name.lower()) # this will print the factory_name in all lower case
print(factory_name.islower()) # checks to see if the factory name is all lower case

# we can combine different functions
print(factory_name.upper().isupper()) # first makes the factory name all upper and then checks if it is all upper

print(len(factory_name)) # prints out the length of the factory_name string

print(factory_name[0:len(factory_name)-1]) # prints the characters of the factory_name from the first character up to the one to the last character

print(factory_name.index("A")) # prints the index of the character A 

print(factory_name.replace("ABC", "DEF")) # replaces the name of the factory with DEF

```

### Lists

A list is a data structure that allows you to store a collection of values. Lists are one of the most commonly used data structures in Python, and they are very versatile. You can use lists to store numbers, strings, objects, or any other type of data. Lists are created using square brackets []. The items in a list are separated by commas. 

For example, the following creates a list of robot manufacturers in the factory example:

``` Python
robot_brands= [“ABB”, “KUKA”, “FANUC”, “Omron” ] 
```

You can access the items in a list using their index. The index of the first item in a list is 0, the index of the second item is 1, and so on. For example, the following code prints the second item in the robot_brands list:

``` Python
print(robot_brands[1])
```

You can also use slices to access a range of items in a list. For example, the following code prints the first three items in the list:

``` Python
print(robot_brands[:3])
```

You can add items to a list using the append() method. For example, the following code adds another manufacturer with the number of robots to the above list:

``` Python
robot_brands.append(“UR”)
print(robot_brands)
```

You can remove items from a list using the `remove()` method. For example, the following code will remove the manufacturer that we just added:

``` Python
robot_brands.remove(“UR”)
print(robot_brands)
```

`insert` will also get the index where you want to insert another item:

``` Python
robot_brands.insert(1, “UR”)
```

This time it will insert it into the second index.

You can sort a list using the `sort()` method. The `sort()` method sorts the list in ascending order:

``` Python
robot_brands.sort()
```

You can reverse the order of a list using the `reverse()` method. The reverse() method reverses the order of the list in place:

``` Python
robot_brands.reverse()
```

You can find the length of a list using the `len()` function:

``` Python
print(len(robot_brands))
```

`extend` function will allow us to get a list and append another list to it:

``` Python
robot_brands1 = [“yaskawa”, “stabuli”]
robot_brands.extend(robot_brands1)
print(robot_brands)
```

`pop` gets rid of the last element in the list:

``` Python
robot_brands.pop()
```

Now I want to know if a certain value is in the list and get the index of the value:

``` Python
print(robot_brands.index("ABB"))
```

We can also count the number of times a value is repeated:

``` Python
print(robot_brands.count("ABB"))
```

`copy` function as the name suggest makes a copy of a list:

``` Python
robot_brands2 = robot_brands.copy()
print(robot_brands)
print(robot_brands2)
```

You can iterate through a list using a for loop. The for loop will iterate through the list, and it will assign each item in the list to the variable that you specify. For example, the following code iterates through the robot_brands list and prints each item:

``` Python
for robot_brand in robot_brands:
    print(robot_brand)
```

You can remove everything with `clear`:

``` Python
robot_brands.clear()
```

Lists are a very powerful data structure, and they can be used to store a wide variety of data.

The whole code:

``` Python
num_int = 10 # this is an integer
num_float = 0.5 # this is a float

# combining numbers with strings
print("There are " + str(num_int) + " arm robots in the factory, "+ str(num_float) + " of them are broken.") 

# using string variables
factory_name = "ABC"
print("The factory's name is " + factory_name)

# list of robot brands
robot_brands = ["ABB", "KUKA", "FANUC", "Omron"]

print(robot_brands)

# access the second robot brand in the above list
print(robot_brands[1])

# print the first three items in the list
print(robot_brands[:3])

# add another manufacturer "UR" to the above list
robot_brands.append("UR")
print(robot_brands)

# remove the manufacturer that we just added
robot_brands.remove("UR")
print(robot_brands)

# insert the manufacturer "UR" to the second index
robot_brands.insert(1, "UR")
print(robot_brands)

# sort() function --> sorts the list in assending order
robot_brands.sort()
print(robot_brands)

# reverse the order of the list
robot_brands.reverse()
print(robot_brands)

# length of the list
print(len(robot_brands))

# we now want to append another list to the robot_brands list
robot_brands_1 = ["Yaskawa", "Staubli"]
robot_brands.extend(robot_brands_1)
print(robot_brands)

# we want to get rid of the last element of the robot_brands list
robot_brands.pop()
print(robot_brands)

# find the index of the value "ABB"
print(robot_brands.index("ABB"))

# now let's find out how many times "ABB" is repeated 
print(robot_brands.count("ABB"))

# we want to make a copy of the robot_brands list
robot_brands2 = robot_brands.copy()
print(robot_brands)
print(robot_brands2)


# we can iterate through a list with a for loop
for robot_brand in robot_brands:
    print(robot_brand)


# we can remove everything in a list with clear function
robot_brands.clear()
print(robot_brands)
```

#### 2D Lists

In Python, a 2D list, also known as a nested list (or a list inside another list), is a list that contains other lists as its elements. It allows you to create a grid-like structure where each element represents a value in a row and column format.

To create a 2D list in Python, you can use the following syntax:

`matrix = [[element1, element2, ...], [element3, element4, ...], ...]`

Here's a breakdown of the components:

- matrix: This is the name of the 2D list. You can choose any valid variable name you prefer.
- element1, element2, element3, etc.: These are the individual elements within the 2D list. They can be any valid Python data type, such as numbers, strings, or even other lists.

Here's an example of creating a 2D list representing a 3x3 grid:

`matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]`

This creates a 2D list called "matrix" with three rows and three columns.

You can access individual elements within a 2D list using the row and column indices. The indices start from 0, so the element in the first row and first column can be accessed as `matrix[0][0]`, the element in the second row and third column as `matrix[1][2]`, and so on.

We can also modify an element in a 2D list:

`matrix[1][2] = 10`

2D lists are useful for representing grids, tables, and matrices in Python. They provide a flexible and convenient way to store and manipulate data in a tabular format.

The whole code:

``` Python
# this is an example of a 2D lists
# 2D lists --> lists inside lists

# lets create a 3*3 matrix using 2D lists

matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
print(matrix)
# this matrix has 3 rows and 3 columns

# we can access individual elements within a 2D list using the row and column indices 
# the indices start from 0 

# we want to access the element in the first row and first column:
print(matrix[0][0])

# how about in the second row and third column?
print(matrix[1][2])
```

### Tuple

Another data type that we can store in a variable is a Tuple.

In Python, a tuple is an ordered collection of values. Tuples are similar to lists, but they are immutable, which means that they cannot be changed once they are created. Tuples are created by enclosing a comma-separated list of values in parentheses. For example, the following code creates a tuple of robotic engineers in the factory:

``` Python
robotic_engineers = (“Hannah”, “Jim”, “Bella”, “John”)
```

Tuples can be indexed and sliced just like lists. For example, the following code prints the second value in the tuple:

``` Python
print(robotic_engineers[1])
```

Tuples can also be unpacked into multiple variables. For example, the following code unpacks the tuple into the variables e1, e2, e3, and e4:

``` Python
e1, e2, e3, e4 = robotic_engineers
print(e1, e2, e3, e4)
```

Tuples can be used to store any type of data, including strings, numbers, lists, and other tuples. They are often used to store data that does not need to be changed, such as the names of people in a group or the coordinates of a point on a map.

If we try to change anything inside the tuple, we'll get an error:

``` Python
Robotic_engineers[1] = “Bob”
```

We can also make a list of tuples:

``` Python
robotic_partners = [(“Hannah”, “John”), (“Jim”,”Bella”)]
print(robotic_partners)
```

The whole code that I wrote in the video:

``` Python
# this section is about tuples

robotic_engineers = ("Hannah", "Jim", "Bella", "John")
print(robotic_engineers)

# print the 2nd value in the tuple
print(robotic_engineers[1])

# unpack the tuple into the variables e1, e2, e3, and e4 that stand for engineers 1 through 4

e1, e2, e3, e4 = robotic_engineers
print(e1, e2, e3, e4)
print(e3)

# we cannot change anything inside the tuple
# robotic_engineers[1] = "Bob"

# we can make a list of tuples
robotic_partners = [(robotic_engineers[0],robotic_engineers[3]), (robotic_engineers[1], robotic_engineers[2])]
print(robotic_partners)
print(robotic_partners[0])
```
### Dictionaries 

The other data type is dictionaries.

A dictionary is a data structure that stores data in key-value pairs. We can create these pairs and when we want to access the info, we can refer it by its key. Keys are unique (cannot be repeated), case-sensitive, and they can be any immutable type, such as strings, numbers, or tuples. Values can be of any type.

You can access the values via their keys here. 

<img width="361" alt="image" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/334d463c-a414-4281-b132-56a8b959b1b9">

To create a dictionary in Python, we must enclose a sequence of elements inside curly brackets and separate them with commas. 

Example 1: A robot arm is trying to sort the objects in its environment based on color. Make a dictionary that maps the name of the object to its color. 

``` Python
# this dictionary maps the name of the object to its corresponding color

objects_colors = {
    "ball": "red",
    "cube": "green",
    "flower": "pink",
    "pyramid": "blue"

}

print("the ball is " + objects_colors["ball"])
print("the cube is " + objects_colors["cube"])

print(objects_colors.get("floer", "no such object"))
```

Here you see that I used .get() method to retrieve the value associated with a key. If the key is found in the dictionary, it returns the corresponding value. However, if the key is not found in the dictionary, it returns a default value provided as the second argument to the .get() method.

As another example, we want to create a dictionary for a robot arm with three revolute joints and this dictionary has three keys, corresponding to the three joints in a robot arm. Each key has three values: the minimum angle, the maximum angle, and the default angle. The minimum and maximum angles define the range of motion for each joint, while the default angle is the angle that the joint is set to when the robot arm is first turned on.

``` Python
# a dictionary that maps robot arm joints to their corresponding min, max, and default joint angles
import math as m

arm_dict = {
    "joint1": {
        "min" : -m.pi/2,
        "max" : m.pi/2,
        "default" : 0,
    },

        "joint2": {
        "min" : -m.pi,
        "max" : m.pi,
        "default" : 0,
    },

        "joint3": {
        "min" : -m.pi/2,
        "max" : m.pi/2,
        "default" : 0,
    },

}


# now we can access the keys and values in this dictionary
joint_angles_1 = arm_dict["joint1"]
print(joint_angles_1)

# you can also access to individual angles within each joint in the dictionary
# this gives the max angle of joint 2
print(arm_dict["joint2"]["max"])

# print the min angle of joint3
print(arm_dict["joint3"]["min"])
```
So, they were the most common data types in python. Now let's talk about functions and the return statement. 

## Functions and return statement

A function is a block of code that performs a specific task. It can be used to group related code together, make code more reusable, and make code easier to read and understand.

The `return` statement is used to exit a function and return a value to the caller. The value that is returned can be any Python object, such as a number, string, list, or dictionary. After the `return`, no code will be executed because when Python sees the return keyword it will break out of the code. 

Functions can be passed data in two ways:
- Arguments. Arguments are passed to a function when it is called. They can be any type of value, such as numbers, strings, lists, or dictionaries.
- Parameters. Parameters are variables that are declared in the function definition. They are used to store the values of the arguments that are passed to the function.

As a very basic example, we want to write a function that gets two numbers and returns their sum.

``` Python
def add(a,b):
return a+b
```

This function takes two numbers as input and returns their sum. It can be called like this:

``` Python
sum = add(3,-1)
print(sum)
```

a,b are parameters and they are used to store the values of the arguments that are passed to the function when it is called. 

As another simple example, suppose that we want to write a function that gets the radius of the wheel of the robot and returns the wheel area:

``` Python
import math as m

# this function will get the radius of the wheel and returns the area of the wheel
def calc_wheel_area(radius):
    area = m.pi * radius**2
    return area

# now we want to call this function
wheel_radius = 0.2 # 0.2 meters
wheel_area = calc_wheel_area(wheel_radius)
print("The area of the wheel is", wheel_area)
```

## Conditional statements: if statements

In Python, the if statement is used for conditional execution. It allows you to execute certain blocks of code based on whether a specified condition is true or false. Here's a brief explanation of the if statement syntax:

``` Python
if condition:
# code to be executed if the condition is True.
else:
# code to be executed if the condition is False
```

The condition is an expression that evaluates to be either True or False. If the condition is true, the code block under the if statement is executed. If the condition is false, the code block under the else statement (optional) is executed.

You can compare boolean values or numeric or string values. Let's see an example. 

Let's say you have a robot arm with two joints, and you want to control the movement of the arm based on the input from a joystick. If the joystick is tilted to the right, the first joint should rotate in a positive direction based on the RHR, and if the joystick is tilted to the left, the first joint should move in a negative direction based on the RHR. Similarly, tilting the joystick up means moving the second joint in the positive direction, and tilting the joystick down means moving the second joint in the negative direction. 

``` Python
# we want to control a two-joint robot arm with a joystick
# if the joystick is tilted to the right --> 1st joint rotates in + direction based on RHR
# if the joystick is tilted to the left --> 1st joint rotates in - direction based on RHR
# if the joystick is tilted up --> 2nd joint rotates in + direction based on RHR
# if the joystick is tilted down --> 2nd joint rotates in - direction based on RHR
# in neutral position --> the arm does not move

# we first get the joystick input
joystick_in = "up"

if joystick_in == "right":
    print("moving 1st joint in positive direction")
elif joystick_in == "left":
    print("moving 1st joint in negative direction")
elif joystick_in == "up":
    print("moving the 2nd joint in positive direction")
elif joystick_in == "down":
    print("moving the 2nd joint in the negative direction")
else:
    print("the joystick is in the neutral position, no movement")
```

In a real situation, you should read from the joystick input data and input the real data to the program and then command the joint motors to rotate accordingly. 

**Another example:**

Let's consider a mobile robot is equipped with two proximity sensors that detect obstacles. If any of the sensors detect an obstacle within a certain range, the robot should stop and display a warning message. Otherwise, it should continue moving forward. Here's an example implementation:

``` Python
# in this example code we have a mobile robot that is equipped with proximity sensors and can detect obstacles 
# if any of the sensors detect an obstacle within a certain range, the robot should stop and display a warning message
# otherwise it should move forward

sensor1_dist = 1.5 # distance read by sensor 1 in meters
sensor2_dist = 1.2 # distance read by sensor 2 in meters
threshold_dist = 1 # threshold distance for considering an obstacle in meters


if sensor1_dist < threshold_dist or sensor2_dist < threshold_dist:
    print("Warning: obstacle detected! Stopping!")
else:
    print("No obstacle, moving forward!")
```

If at least one sensor detects an obstacle (i.e., its reading is less than the safe distance), the code block under the if statement is executed, printing the warning message and performing additional actions to stop the robot. If none of the sensors detect an obstacle, the code block under the else statement is executed, printing "Moving forward" and performing additional actions for the robot to move forward.

For an actual scenario, you should read from your sensor data and command the motors to do the action that you want the robot to do. 

Let’s also see a general example: write a Python function that gives us the min number that we pass to it. 

Boolean table:

<img width="373" alt="boolean table min of three numbers" src="https://github.com/madibabaiasl/modern-robotics-I-course/assets/118206851/13be4879-0c15-4838-b15c-763a1edbc7f5">

``` Python
# in this example we want to write a function that gives us the minimum number of three numbers that we pass to it

def min_num(a,b,c):
    if a < b and a < c:
        return "The smallest number is " + str(a)
    elif b < a and b < c:
        return "The smallest number is " + str(b)
    elif c < a and c < b:
        return "The smallest number is " + str(c)
    else:
        return "They are equal"
    


print(min_num(2,-1,0))
```

## While loops

In Python, a while loop is a control flow statement that allows you to repeatedly execute a block of code as long as a certain condition is true. The general syntax of a while loop in Python is as follows:

``` Python
while condition:
    # code to be executed
```

Here's how a while loop works:
- The condition is evaluated. If it is true, the code block inside the loop is executed. If it is false, the loop is exited, and the program continues with the next statement after the loop.
- After executing the code block, the condition is checked again.
- If the condition is still true, the code block is executed again. This process continues until the condition becomes false.

It's important to be cautious while working with while loops to avoid infinite loops. An infinite loop occurs when the condition always remains true, causing the loop to execute indefinitely. To avoid this, ensure that the condition eventually becomes false. 

While loops are very flexible and can be used in various situations. They are particularly useful when the number of iterations is unknown or dependent on a certain condition.

As an illustrative example suppose that we have a robot that will move forward until it encounters an obstacle. 

``` Python
# in this example we want to simulate a situation where a robot moves forward until it encounters an obstacle
import time
import random 

# the move_robot() function simulates the robot's movement

def move_robot(distance):
    print("Moving robot forward ...")
    # now we want to simulate the time it takes for the robot to move forward by giving some delay
    time.sleep(1)
    print(f"The robot has moved {distance} units.")
    # note that f"..." is used to create formatted string literals aka f-strings 
    # it is introduced in python 3.6 and allows you to embed expressions inside string literals


# now we should write a function for obstacle detection
def detect_obstacle():
    # simulating obstacle detection
    obstacle = random.choice([True, False])
    return obstacle 
# the random.choice() function is part of the random module in python
# it allows you to select a random element from a sequence such as a list, tuple or string 
# because we are simulating a situation here and we do not have real obstacles we randomly choose whenever an obstacle is present

# robot's initial position
position = 0

# we will keep moving the robot until an obstacle is detected
while not detect_obstacle():
    # now we should generate a random distance to move fw (because we are simulating and it's not a real scenario)
    distance = random.randint(1, 5)
    # here we call the move_robot() function to move the robot fw by the randomly generated distance 
    move_robot(distance)
    # now we will add this distance to the position of the robot
    # here we update the position of the robot
    position += distance

# here the while loop continues to execute as long as the detect_obstacle() function returns False meaning that there is no obstacle. 
# if it becomes True, then it means that there is an obstacle and the loop is exited and a message will print showing that there is an obstacle
# and the control system should take action accordingly and commands the actuators to stop the robot


print("Obstacle is detected! Stopping the robot.")
# we also print the final position of the robot
print(f"The final position of the robot is {position}")
```

Note that in this example, obstacle detection and robot movement are simulated for illustration purposes. In a real robotic application, you would have actual sensors and control mechanisms to detect obstacles and move the robot accordingly.

## For loops

In Python, a for loop is a control flow statement used for iterative execution. It allows you to iterate over a sequence of elements, such as a list, tuple, string, or any other iterable object. The basic syntax of a for loop in Python is as follows:

``` Python
for element in iterable:
    # Code block to be executed
```

Here's a breakdown of the components:
- element: This is a variable that takes the value of the next item in the iterable for each iteration. You can choose any valid variable name you prefer.
- iterable: This is a sequence of elements that can be iterated over. It can be a list, tuple, string, range, or any other iterable object.

The code block under the for loop is indented and executed repeatedly for each element in the iterable. The loop continues until all elements in the iterable have been processed.

It's important to note that you can combine for loops with conditional statements (such as if-else) and other control flow statements to create more complex logic within the loop.

Here I will give you a supper simple example to get the idea.

``` Python
# this is for the for loops section
# these are very basic and easy examples
# more to come in the classes and objects section

# we can use for loop with strings
# a loop that iterates over each character in the string "Robotics" and prints each character on a separate line

for letter in "Robotics":
    print(letter)

# we can also use for loops with other data types like lists
# Here we defined a list called robotics_engineers containing the names of several robotics engineers, and then we used a for loop to # iterate through each engineer's name in the list and print each name on a separate line. 

robotics_engineers = ["Tim", "Hannah", "John", "Bella"]
for robotic_engineer in robotics_engineers:
    print(robotic_engineer)

# let's do another for loop with the indexes
# Here, we are using a for loop to iterate through the indices of the robotics_engineers list, and for each index, we are printing both the # index itself and the corresponding element (name of the robotic engineer) from the list.

for index in range(len(robotics_engineers)):
    print(index)
    print(robotics_engineers[index])

# you can loop through a series of numbers or range of numbers
# The range function in Python generates a sequence of numbers starting from the first argument and ending at one less than the second argument. So the numbers generated will be from 3 to 9.
for number in range(3, 10):
    print(number)
```

### Nested Loops

A nested loop means loops inside the loops. You can iterate over the elements of a 2D list using nested loops. The outer loop iterates over the rows, and the inner loop iterates over the columns. This allows you to perform operations on each element of the grid.

Here's an example that demonstrates iterating over a 2D list:

``` Python
# this is for the nested loop section
# nested loops --> loops inside the loops

# we can iterate over the elements of a 2D list using nested loops
# the outer loop iterates over the rows 
# the inner loop iterates over the columns 

# let's go back to the example of the 2D list
matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

for row in matrix:
    for element in row:
        print(element)
```
In this example, the outer loop iterates over each row of the matrix, and the inner loop iterates over each element in the current row, printing its value.

## Classes and objects

In previous sections, we saw different data types in Python like numbers, strings, and boolean values, and structures like lists or dictionaries that can store our data. 

But, there are lots of other things that cannot be represented by these data types like a person, a computer, a robot, etc. These cannot be covered by the data types that we have in Python. With classes and objects we can create our own data types for anything we want. For instance, a robot data type represents a robot. We can store all the information that we want to know about the robot inside that data type and Python will create a class for it. 

In Python, classes and objects are the fundamental concepts of object-oriented programming (OOP). Classes define the blueprint or template for creating objects, and objects are instances of those classes.

Let's start by understanding the basics of classes and objects:

Class: A class is a user-defined blueprint or prototype that defines the attributes (data) and methods (functions) that an object of that class can have. It serves as a template for creating objects. In Python, a class is defined using the class keyword. With class, we can define our own data type. 

Object: An object is an instance of a class. It is created using the class as a template and has its own unique set of attributes and can perform actions defined in the class's methods.

Once an object is created, you can access its attributes and call its methods using the dot notation.

Objects created from the same class can have different attribute values while sharing the same methods defined in the class.

Classes and objects allow you to organize and encapsulate data and functionality in a structured manner, making code more modular, reusable, and easier to maintain.

Let’s define a simplified robot arm class that has the attributes of name, length, weight, color, and position. We want to define two methods in this class: move that moves the robot’s position by certain angles and display_info that displays information about the robot including its new position. 

``` Python
# this is about creating a simple robot arm class

# here the RobotArm class has the attributes of name, length, weight, and color and also position
# we use the methods move and displaye_info to rotate the robot and display the information
# the __init__ method serves as the constructor and initializes the robot arm's attributes when an object is created from the class
# the move method takes an angle parameter and updates the robot arm's position by adding the angle to the current position
# the display_info method prints the robot arm's name, length, weight, color, and current position
# here suppost that our robot arm only has one link and one joint --> 1 degree of freedom
class RobotArm:
    def __init__(self, name, length, weight, color):
        self.name = name
        self.length = length
        self.weight = weight
        self.color = color
        self.position = 0

    def move(self, angle):
        self.position += angle

    def display_info(self):
        print(f"Name: {self.name}")
        print(f"Length: {self.length} cm")
        print(f"Weight: {self.weight} kg")
        print(f"Color: {self.color}")
        print(f"Position: {self.position} degrees")


"""
note that whatever I write between these three times quotation marks will not be executed
this is another way to write comments like using #
note that this is for the humans only to understand the code

"""

# now let's create an object of the RobotArm class and utilize its attributes and methods

# here we create an object arm1 of the RobotArm class 
# we provided it with arguments Armrob, 50, 10, and Black to the constructor 
arm1 = RobotArm("Armrob", 50, 10, "Black")
# here we call the move method to update the arm's position by 45 deg and display its information using the display_info method
arm1.move(45)
arm1.display_info()

# here we again call move method to further change the arm's position by -15 deg and display the updated information
arm1.move(-15)
arm1.display_info()
```

Accessing the functions in a class by an object:

``` Python
# we can also access to everything in the class from outside of the class using import function
# let's take a look at the RobotArm class again

class RobotArm:
    def __init__(self, name, length, weight, color):
        self.name = name
        self.length = length
        self.weight = weight
        self.color = color
        self.position = 0

    def move(self, angle):
        self.position += angle

    def display_info(self):
        print(f"Name: {self.name}")
        print(f"Length: {self.length} cm")
        print(f"Weight: {self.weight} kg")
        print(f"Color: {self.color}")
        print(f"Position: {self.position} degrees")
```

``` Python
# here we can import that class and all of the objects can access the methods in it

from RobotArm import RobotArm

# the first RobotArm refers to RobotArm.py and the second refers to the class name

arm1 = RobotArm("Armrob", 50, 10, "Black")
arm1.move(45)
arm1.display_info()

arm1.move(-15)
arm1.display_info()

# as you can see we can access the methods (functions) inside the class by importing it 
# here the arm1 object can access all the functions inside the RobotArm class
```
### Inheritance

Inheritance is a fundamental concept in object-oriented programming (OOP) that allows classes to inherit attributes and methods from other classes. It enables code reuse, promotes modularity, and facilitates the creation of hierarchies and relationships between classes.

In Python, you can create a new class by deriving it from an existing class. The existing class is called the "parent" or "base" class, and the new class is called the "child" or "derived" class. The child class inherits all the attributes and methods of the parent class and can also add its own unique attributes and methods.

Inheritance allows the child class to reuse the code and behavior defined in the parent class while extending or modifying it to suit its specific needs. It promotes code reusability, reduces redundancy, and allows for more flexible and modular code organization.

In other words, we can define a bunch of attributes and functions inside a class and then we can create another class and we can inherit all of those attributes. 

We can basically have one class that has all the functionality of another class without having to physically write out any of the same methods and attributes. 

Let's explore the concept of inheritance with an example:

Suppose we want to create a program to simulate different types of robots. We can start by creating a base class called Robot that defines common characteristics and behaviors of robots. Here's an example:

Robot.py:

``` Python
# this is about inheritance in Python

# suppose we want to create a program to simulate different types of robots
# first we will create a base class called Robot that defines common characteristics or robots

class Robot:
    def __init__ (self, name, DOFs):
        self.name = name 
        self.DOFs = DOFs

    def introduce(self):
        print(f"I am {self.name}, a {self.DOFs} DOF robot.")

    
    def move(self):
        print("I can move.")
```

In this Robot class, we have defined an __init__ method to initialize the name and dof of the robot, an introduce method to introduce the robot, and a move method to indicate that the robot can move.

Now, let's say we want to create specific types of robots, such as a WheeledRobot and a FlyingRobot, that inherit from the base Robot class. We can define these classes as follows:

WheeledRobot.py:

``` Python
# now we want to create specific types of robots such as a wheeled robot that can inherit from the base Robot class

# this means that from the Robot.py we import the Robot class

from Robot import Robot

class WheeledRobot(Robot):
    def __init__ (self, name, DOFs, num_wheels):
        super().__init__ (name, DOFs) # here we inheretited the name and DOFs from the Robot class
        self.num_wheels = num_wheels
    
    def move(self):
        print(f"I am a wheeled robot with {self.num_wheels} wheels.")
        super().move() # here we called the move method from the base class
```

FlyingRobot.py:

``` Python
# now we want to create another specific type of robots like a flying robot that will inherit from the Robot class

# frist we import Robot class from Robot.py

from Robot import Robot
# here we follow a similar pattern to the WheeledRobot class
class FlyingRobot(Robot):
    def __init__ (self, name, DOFs, max_altitude):
        super().__init__ (name, DOFs) # we inherit these attributes from the Robot class
        self.max_altitude = max_altitude
    
    def move(self):
        print(f"I am a flying robot with a max altitude of {self.max_altitude}.")
        super().move() # we also call the move method from the base class
```

main_inheritance.py:

``` Python
# let's now create some instances of these classes and see how they behave

from WheeledRobot import WheeledRobot
from FlyingRobot import FlyingRobot 

robot1 = WheeledRobot("Rover", 3, 4)
robot1.introduce() # this is the introduce function that we created in the Robot class and WheeledRobot class inherited from it
robot1.move() # this will provide its own definition of the move function and also the inherited move function from the Robot class


robot2 = FlyingRobot("DroneX", 6, 100)
robot2.introduce()
robot2.move()

# here you saw that the instance of WheeledRobot and FlyingRobot inherit the introduce and move methods from the Robot base class.
# they also have their own specialized implementations of the move method specific to them

# here you saw that by using inheritance we can easily extend and customize classes to represent different types of robots 
# while reusing common code from the base class
```

## Reading Files

Reading external files like a text file, or a CSV file in Python is a common task when working with data or text files. 

Let's say you have a file called "sensor_data.txt" that contains readings from a temperature sensor in different time stamps. Each line of the file represents a timestamped temperature reading. Here's the example content of the "sensor_data.txt" file:

[sensor_data.txt](https://github.com/madibabaiasl/modern-robotics-I-course/files/12465385/sensor_data.txt)

read_sensor_data.py:

``` Python
# to read this file, we should first open it

with open("intro to python/sensor_data.txt", "r") as file:
    # now iterate over each line
    for line in file:
        # split the line into timestamp and temprature:
        timestamp, temprature = line.strip().split(", ")

        # now we can process the data
        print("Timestamp: ", timestamp)
        print("Temprature: ", temprature)
```


In this example, we open the file using the open() function with the file name "sensor_data.txt" and mode "r" for reading. The with statement ensures that the file is properly closed after we finish reading.

Inside the loop, each line is split using the split() method with ", " as the separator. This separates the timestamp and temperature values. The strip() method removes any leading or trailing whitespace from the line.

You can then process the data as per your requirements. In this example, we simply print the timestamp and temperature for each line.

Note: Make sure the "sensor_data.txt" file is in the same directory as your Python script or provide the full path to the file if it's located elsewhere on your system.

There are also writing and appending modes onto files in Python that have less application for our cases so if you like to learn them you can learn them on your own. 

## Modules and pip command

In Python, a module is a file containing Python definitions, statements, and functions that can be imported and used in other Python programs. It provides a way to organize and reuse code, making your programs more modular and easier to manage. Modules are an essential concept in Python programming and are used extensively in large projects.

Here's a breakdown of the key aspects of modules in Python:

Creating a Module:

- A module is simply a Python file with a .py extension.
- To create a module, you can start by writing your Python code in a file and save it with a meaningful name and the .py extension.
- For example, if you create a module called my_module.py, it can be imported into other Python programs.

Importing Modules:

- To use a module in your program, you need to import it.
- You can import an entire module or specific objects (functions, classes, variables) from it.
- The import statement is used for importing modules. For example:

import my_module

Accessing Module Objects:

- Once a module is imported, you can access its objects using dot notation.

For example, if my_module contains a function called my_function, you can call it as follows:
my_module.my_function()

Module Aliases:

- You can assign an alias to a module to provide a shorter or more convenient name.
- Aliases are created using the as keyword during the import statement.

Here's an example:

``` Python
import my_module as mm
mm.my_function()
```
Importing Specific Objects:

- If you only need certain objects from a module, you can import them directly, rather than importing the entire module.
- This approach can save memory and improve code readability.

Here's an example:

``` Python
from my_module import my_function, my_variable
my_function()
print(my_variable)
```

Creating Your Own Modules:

- To create your own module, you can define functions, classes, and variables within a Python file.
- You can use these modules in other Python programs by importing them.
- It is good practice to include a brief description of the module's purpose using docstrings (a string at the beginning of a module or function) to provide documentation for users.

Standard Library Modules:

- Python comes with a vast standard library that provides many modules for common tasks.
- These modules offer functionalities such as file I/O, mathematical operations, networking, regular expressions, and more.
- You can import and use these modules in your programs without any additional installation.

Third-Party Modules:

- In addition to the standard library, Python has a rich ecosystem of third-party modules available for various purposes.
- These modules are created by the Python community and can be installed using package managers like pip.
- Popular third-party modules include NumPy for numerical computations, Pandas for data analysis, and Django for web development.

## Python Interpreter and getting input from users

Python interpreter is a little environment that we can use to execute Python commands. To use the Python interpreter you should first open the command prompt that for Ubuntu it is Ctrl+Alt+T.  

So first open the terminal
Then type python3 to see the latest version of python3 and get into the python interpreter
Now you can write quick Python code and quick test it. 

Example:

We want to get the user’s name and age, and then greet them using this information.

First, we take information from the user in Python using the input(“prompt”) command, then we store it in the variable and then we can do some operations with that variable. 

For this section, watch the video version of the lesson. 

## Try Except 

The try and except statements in Python are used to handle exceptions. An exception is an error that occurs during the execution of a program. If an exception is not handled, the program will crash.

The try block contains the code that you want to run and that might throw an exception. The except block contains the code that you want to run if an exception occurs.

Example: divide by zero

``` Python
def divide_numbers(x, y):
    try:
        return x / y
    except ZeroDivisionError:
        print("Division by zero error")

print(divide_numbers(10, 2))
print(divide_numbers(10, 0))
```

The divide_numbers() function takes two numbers as input and returns the result of dividing them. The try block tries to divide the two numbers. If the denominator is 0, a ZeroDivisionError exception will be thrown. The except block catches the ZeroDivisionError exception and prints a message.

The first line of output is the result of calling divide_numbers() with 10 and 2 as input. The second line of output is the result of calling divide_numbers() with 10 and 0 as input. The ZeroDivisionError exception is caught by the except block and the message "Division by zero error" is printed.

The None at the end is the result of the second print statement. This is because the divide_numbers function doesn't have a return statement for the case where a ZeroDivisionError occurs. In Python, a function that doesn't explicitly return a value implicitly returns None.

In order to handle other exceptions we can use more except blocks. For instance here, we also handled the invalid input error as well:

try-exept.py:

``` Python
def divide_numbers(x,y):
    try:
        return x/y
    except ZeroDivisionError:
        return "Division by zero error"
    except:
        return "Invalid"

# here we also handled the invalid input error as well. so depending on the situation we can handle the error

print(divide_numbers(5,2))
print(divide_numbers(5,0))
```

## Numpy

NumPy is a powerful Python library used for numerical computing. It provides efficient array operations and mathematical functions, making it widely used in robotics for tasks such as data processing, simulation, and control.

In order to install Numpy, first open up a terminal in Ubuntu: Ctrl + Alt + T. 

- first update the system
- then install pip if it is not already installed. Remember that pip is for installing packages in Python.
`sudo apt install python3-pip`

enter the password and wait for the pip to be installed. 
- check to see if pip is installed: `pip --version`
- then install numpy by typing `pip3 install numpy`
- now check numpy in the Python interpreter environment (type python3 to enter that environment) and create an array using the numpy library 

See the install and test numpy video for instructions

Now that it is installed, let's see how we can use it in our VS code IDE. 

See the codes and videos for numpy example

numpy-code.py

``` Python
# now let's test working with numpy

import numpy as np

v1 = np.array([1, 2, 3, 4, 5, 6])
print(v1)

# this way you can see that the v is saved as an array istead of a list 

# let's define another vector
v2 = np.array([7, 8, 9, 10, 11, 12])

# we can do several vector operations
print(v2)
print(v1 + v2)

# here the vectors should be of the same length

# element-wise, matrix multiplications
print (v1 * v2)

print(v1.shape) # it shows that v1 is 6 times nothing matrix 
# if we want to make it a 6 by 1 vector:
v1 = np.array([[1], [2], [3], [4], [5], [6]])
print(v1.shape)

# we do the same with v2
v2 = np.array([[7], [8], [9], [10], [11], [12]])
print(v2.shape)

# in order to do matrix multiplication, we should multiply one by the transpose of the other 

print(np.matmul(v1,v2.T))
print("or")
print(np.matmul(v1,np.transpose(v2))) # this matrix is 6 by 6
print(v2.T.shape) # see that it is a 1 by 6 matrix 

# @ also means matrix multiplication 
print(v1.T @ v2) # which is a 1 by 1 vector or just a number

# if we want to write a for example 2 by 6 matrix:

x = np.array([[[1], [2], [3], [4], [5], [6]],[[7], [8], [9], [10], [11], [12]]])
print(x.shape)
```

Explanation of the above code:

We first began by importing the NumPy library and giving it an alias 'np'. Then we created two NumPy arrays, v1 and v2. After that, we did basic element-wise vector operations. It prints the values of v2 and the result of adding v1 and v2 element-wise. Then we demonstrated the element-wise multiplication of v1 and v2. The shape attribute of the NumPy arrays is used to display their dimensions. Initially, the shape of v1 is (6,), indicating a 1-dimensional array with 6 elements. To make it a column vector, it's reshaped to have dimensions (6, 1) using double square brackets. After that, we showed how to perform matrix multiplication using the np.matmul function. Matrix multiplication between v1 and the transpose of v2 is demonstrated which results in a 6x6 matrix. Next, the transpose of a matrix is obtained using the .T attribute. The shape of v2 transposed is (1, 6), indicating a row vector. We also showed matrix multiplication using the @ operator (Python 3.5 and later). The result of v1.T @ v2 is a 1x1 matrix, essentially a scalar (a single number).

In the next part, we showed the creation of a 2 by 6 matrix using NumPy. We created a matrix as a NumPy array called x. This array is constructed using nested lists, where each sublist represents a column of the matrix. The matrix is created with 2 columns and 6 rows. Again we used the .shape attribute of the NumPy array x to display the dimensions of the matrix. The result of x.shape is (2, 6), indicating that the matrix has 2 rows and 6 columns.

This was an introduction to Python programming for robotics applications that covered the basics to help you get started. It can be a stepping stone for you to learn more advanced topics, especially in object-oriented programming.
