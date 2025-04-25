# MPU9250 EIMU (Easy IMU) Driver Code
This is the code running on the microcontroller on the Easy IMU Module.
> [!NOTE]  
> It makes use of the `invensense-imu` library by bolder flight and `arduino_matrix_vector_lab` library by samuko-things and `imu_madgwick_filter` library by samuko-things. But all have been used in a concised way to accomodate the microcontroller code space. The license of the code is GPL3 due to license of the midgwick filter

## How to Use the Driver Code
- Ensure you have the Arduino IDE up and running on your PC

- Download (by clicking on the green Code button above) or clone the repo into your PC

- Open the **`mpu9250_eimu_driver_code`** folder

- Then open the **`mpu9250_eimu_driver_code.ino`** in your Arduino IDE.

- select the board - `Arduino NANO` - and choose PORT. 

- choose for the processor option :`ATmega328P`

- verify and upload the code the code to the **`MPU9250 EIMU Module`** (i.e. **`Easy IMU`**).