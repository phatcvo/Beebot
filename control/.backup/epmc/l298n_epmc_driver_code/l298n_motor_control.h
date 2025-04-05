#ifndef L298N_MOTOR_CONTROL_H
#define L298N_MOTOR_CONTROL_H
#include <Arduino.h>

class L298NMotorControl {
  public:
    L298NMotorControl(int IN1_pin, int IN2_pin, int en_pin);

    void sendPWM(int pwmVal);
    int getDirection();
    void test();

  private:
    int in1Pin, in2Pin, enPin;
    int dir = 1;

    void setForwardDirection();
    void setReverseDirection();
    void setHalt();
    void delayMs(int ms);

};



#endif