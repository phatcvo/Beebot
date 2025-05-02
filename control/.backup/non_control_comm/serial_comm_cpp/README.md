## Easy PID Motor Controller (EPMC) C++ Library
This library helps communicate with the **`Easy PID Motor Controller Module`** (i.e **`L298N EPMC MODULE`** or a **`CUSTOM EPMC INTERFACE BOARD`**) in your PC or microcomputer linux c++ robotic project (as it depends on the libserial-dev package), with the [epmc_setup_application](https://github.com/samuko-things-company/epmc_setup_application).

> you can use it in your microcomputer robotics project **running on linux** (e.g Raspberry Pi, PC, etc.)

A simple way to get started is simply to try out and follow the example code in the example folder

## How to Use the Library
- install the libserial-dev package
  > sudo apt-get update
  >
  > sudo apt install libserial-dev

- Ensure you have the **`L298N EPMC MODULE`** or a **`CUSTOM EPMC INTERFACE BOARD`** interfaced with your preferred motors, setup the encoder and PID parameters with the **`epmc_setup_application`**.

- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
> ```git clone https://github.com/samuko-things-company/epmc_cpp.git```

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on

- A simple way to get started is simply to try out and follow the example `motor_control.cpp` code.

- make, build and run the example code.
  > cd into the root directory
  >
  > mkdir build (i.e create a folder named build)
  >
  > enter the following command in the terminal in the root folder:
    ````
    cmake -B ./build/
    ````
    ````
    cmake --build ./build/
    ````
    ````
    ./build/motor_control
    ````

- You can follow the pattern used in the example `motor_control.cpp` in your own code.


## Basic Library functions and usage

- connect to the **`L298N EPMC MODULE`** or a **`CUSTOM EPMC INTERFACE BOARD`**
  > .connect("port_name or port_path")

- send target angular velocity command
  > .sendTargetVel(motorA_TargetVel, motorB_TargetVel)

- set motor command timeout in ms
  > .setCmdTimeout(timeout_ms) // sets command timeout in ms

- set motor command timeout in ms
  > .getCmdTimeout(&timeout_ms) // sets command timeout in ms

- send PWM commands
  > .sendPwm(motorA_PWM, motorB_PWM)

- read motors angular position
  > .getMotorsPos(&angPosA, &angPosB) // copies the motors angular position into angPosA, angPosB

- read motors angular velocity
  > .getMotorsVel(&angVelA, &angVelB) // copies the motors ang vel angVelA, angVelB

- read motorA maximum commandable angular velocity
  > .getMotorAMaxVel(&maxVelA)

- read motorB maximum commandable angular velocity
  > .getMotorBMaxVel(&maxVelB)