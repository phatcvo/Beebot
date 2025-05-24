#ifndef L298N_MOTOR_DRIVER_H
#define L298N_MOTOR_DRIVER_H
#include <Arduino.h>

class L298NMotorDriver {
  public:
    // L298NMotorDriver(int IN1_pin, int IN2_pin, int en_pin);
    L298NMotorDriver(int DR_pin, int PWM_pin, int EN_pin);

    void sendPWM(int pwmVal, bool invert);
    int getDirection();
    void test();

  private:
    // int in1Pin, in2Pin, enPin;
    int drPin, pwmPin, enPin;
    int dir = 1;

    void setForwardDirection();
    void setReverseDirection();
    void setHalt();
    void delayMs(int ms);

};



#endif