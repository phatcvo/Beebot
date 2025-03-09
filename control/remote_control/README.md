# XBox Gamepad

### Install & Run

- Installing the connecting package
  `sudo apt-get install ros-indigo-joy`

- Configuring the Joystick
  Connect your joystick to your computer. Now let's see if Linux recognized your joystick.
  `ls /dev/input/`

As you can see above, the joystick devices are referred to by `jsX` ; in this case, our joystick is `js0`. Let's make sure that the joystick is working.

`sudo jstest /dev/input/jsX`

- Now let's make the joystick accessible for the ROS joy node. Start by listing the permissions of the joystick:
  `ls -l /dev/input/jsX`
- [Optional] the js device is not configured properly and you need to:
  `sudo chmod a+rw /dev/input/jsX`
- Run
  `roslaunch joy_teleop joy_teleop.launch`

#### Control Key

- Table of buttons index of functions:
  Index Function
  X Use cross-button for spd control
  A Use joystick for spd control
  B E-stop
  Y Cancel E-stop
  LB Linear Speed Down
  RB Linear Speed Up
  LT Angular Speed Down
  RT Angular Speed Up
  back Mode: AUTO
  start Start the joy controller / Mode: MANUAL
  Button stick left (Not Use)
  Button stick right (Not Use)

- Table of axes index of functions:
  Index Function
  L/R Axis stick left (Not Use)
  U/D Axis stick left Move front/back
  L/R Axis stick right Move left/right
  U/D Axis stick right (Not Use)
  cross key L/R Control linear speed
  cross key U/D Control angular speed

  ref: https://www.david-amador.com/2012/04/xbox-360-controller-input-in-c-via-xinput/
