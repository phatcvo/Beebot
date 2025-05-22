#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <iostream>

#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

class PCA9685 {
public:
    int i2c_fd;

    PCA9685(const char* i2c_dev = "/dev/i2c-1") {
        i2c_fd = open(i2c_dev, O_RDWR);
        if (i2c_fd < 0) {
            perror("Failed to open I2C device");
            exit(1);
        }

        if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
            perror("Failed to set I2C address");
            exit(1);
        }

        reset();
        setPWMFreq(50);  // 50 Hz for servos
    }

    ~PCA9685() {
        close(i2c_fd);
    }

    void reset() {
        writeRegister(MODE1, 0x00);
    }

    void setPWMFreq(float freq) {
        float prescaleval = 25000000.0;
        prescaleval /= 4096.0;
        prescaleval /= freq;
        prescaleval -= 1.0;
        uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));

        uint8_t oldmode = readRegister(MODE1);
        uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
        writeRegister(MODE1, newmode);
        writeRegister(PRESCALE, prescale);
        writeRegister(MODE1, oldmode);
        usleep(5000);
        writeRegister(MODE1, oldmode | 0xa1); // auto-increment
    }

    void setPWM(int channel, int on, int off) {
        writeRegister(LED0_ON_L + 4 * channel, on & 0xFF);
        writeRegister(LED0_ON_L + 4 * channel + 1, on >> 8);
        writeRegister(LED0_ON_L + 4 * channel + 2, off & 0xFF);
        writeRegister(LED0_ON_L + 4 * channel + 3, off >> 8);
    }

    void setServoPulse(int channel, float pulse_us) {
        float pulse_length = 1000000.0 / 50.0 / 4096.0; // 50Hz, 12-bit resolution
        int ticks = static_cast<int>(pulse_us / pulse_length);
        setPWM(channel, 0, ticks);
    }

private:
    void writeRegister(uint8_t reg, uint8_t data) {
        uint8_t buf[2] = {reg, data};
        if (write(i2c_fd, buf, 2) != 2) {
            perror("Failed to write to I2C");
        }
    }

    uint8_t readRegister(uint8_t reg) {
        if (write(i2c_fd, &reg, 1) != 1) {
            perror("Failed to write register address");
        }

        uint8_t data;
        if (read(i2c_fd, &data, 1) != 1) {
            perror("Failed to read from I2C");
        }
        return data;
    }
};

// Global pointer to PCA9685 driver
PCA9685* pwm_driver = nullptr;

// Callback to receive microsecond servo pulse values
void servoCallback(const std_msgs::UInt16::ConstPtr& msg) {
    int pulse_us = msg->data;
    if (pulse_us < 500 || pulse_us > 2500) {
        ROS_WARN("Pulse out of range (500–2500us): %d", pulse_us);
        return;
    }

    pwm_driver->setServoPulse(0, pulse_us);  // channel 0
    ROS_INFO("Set pulse width: %dus", pulse_us);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pca9685_node");
    ros::NodeHandle nh;

    pwm_driver = new PCA9685();

    ros::Subscriber sub = nh.subscribe("servo_angle_us", 10, servoCallback);

    ros::spin();

    delete pwm_driver;
    return 0;
}
