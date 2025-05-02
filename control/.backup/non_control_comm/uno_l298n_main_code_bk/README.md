# L298N EPMC Driver Code (Easy PID Motor Controller - EPMC)
This is the code running on the microcontroller on the **Easy PID Motor Controller** (**L298N EPMC Module**).

## How to Use the Driver Code
- Ensure you have the Arduino IDE up and running on your PC

- Download (by clicking on the green Code button above) or clone the repo into your PC

- Open the **`l298n_epmc_driver_code`** folder

- Then open the **`l298n_epmc_driver_code.ino`** in your Arduino IDE.

- select the board - `Arduino NANO` - and choose PORT. 

- choose for the processor option :`ATmega328P`

- verify and upload the code the code to the **`L298N EPMC Module`** (i.e. **`Easy PID Motor Controller`**).


## Routes and their functions
> please check the `serial_i2c_comm_api.h` file to the routes
- **`"/pos"`** : access the function to return the **angular positions** of `motorA` and `motorB` respectively.
- **`"/vel"`** : access the function to return the **filtered angular speeds** of `motorA` and `motorB` respectively.
- **`"/velA"`** : access the function to return respectively the **raw angular speed** and **filtered angular speed** of `motorA`.
- **`"/velB"`** : access the function to return respectively the **raw angular speed** and **filtered angular speed** of `motorB`.
- **`"/pVelA"`** : access the function to return respectively the **pid target angular speed** and **filtered angular speed** of `motorA`.
- **`"/pVelB"`** : access the function to return respectively the **pid target angular speed** and **filtered angular speed** of `motorB`.
- **`"/dataA"`** : access the function to return respectively the **angular position** and **filtered angular speed** of `motorA`.
- **`"/dataB"`** : access the function to return respectively the **angular position** and **filtered angular speed** of `motorB`.
- **`"/pwm"`** : access the function to send **pwm value** to `motorA` and `motorB` respectively.
- **`"/tag"`** : access the function to send **target angular velocities** to `motorA` and `motorB` respectively.
- **`"/mode"`** : access the function to get and send **operation mode value [0 or 1]**  where 0 - PWM MODE and 1 - PID MODE.
- **`"/pprA"`** : access the function to get and send **pulse per rev** value of `motorA` encoder.
- **`"/pprB"`** : access the function to get and send **pulse per rev** value of `motorB` encoder.
- **`"/kpA"`** : access the function to get and send **Kp (proportional) gain** value of `motorA`.
- **`"/kiA"`** : access the function to get and send **Ki (integral) gain** value of `motorA`.
- **`"/kdB"`** : access the function to get and send **Kd (differential) gain** value of `motorA`.
- **`"/kpB"`** : access the function to get and send **Kp (proportional) gain** value of `motorB`.
- **`"/kiB"`** : access the function to get and send **Ki (integral) gain** value of `motorB`.
- **`"/kdB"`** : access the function to get and send **Kd (differential) gain** value of `motorB`.
- **`"/ordA"`** : access the function to get and send **lowpass filter order [1 or 2]**  for `motorA` velocity filter.
- **`"/ordB"`** : access the function to get and send **lowpass filter order [1 or 2]**  for `motorB` velocity filter.
- **`"/f0A"`** : access the function to get and send **lowpass filter cut-off frequency**  for `motorA` velocity filter.
- **`"/f0B"`** : access the function to get and send **lowpass filter cut-off frequency**  for `motorB` velocity filter.
- **`"/sfA"`** : access the function to get and send **stopping frequency** (i.e min frequency at which the motor is seen as stopped)  for `motorA` velocity calculation.
- **`"/sfB"`** : access the function to get and send **stopping frequency** (i.e min frequency at which the motor is seen as stopped)  for `motorB` velocity calculation.
- **`"/rdirA"`** : access the function to get and send **default direction [-1 or 1]**  for `motorA`. where -1 -> REVERSE and 1 -> FORWARD.
- **`"/rdirB"`** : access the function to get and send **default direction [-1 or 1]**  for `motorB`. where -1 -> REVERSE and 1 -> FORWARD.
- **`"/i2c"`** : access the function to get and send **i2c comm address**  for the `smc_l298n_pid_driver` module.
- **`"/reset"`** : access the function to send a **reset**  command to reset all parameters to their default values.
- **`"/freq"`** : access the function that send or set the **allowable operation frequency**. Its default value 2000Hz.
- **`"/maxVelA"`** : access the function to send or set the **maximum commandable velocity of Motor A**. w _allowableA = (2 * PI * 2000[gotten from the `"/freq"` route]) / PPRA
- **`"/maxVelB"`** : access the function to send or set the **maximum commandable velocity of Motor B**. w _allowableA = (2 * PI * 2000[gotten from the `"/freq"` route]) / PPRB