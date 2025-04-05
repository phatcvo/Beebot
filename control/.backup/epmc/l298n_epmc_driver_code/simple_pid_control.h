#ifndef SIMPLE_PID_CONTROL_H
#define SIMPLE_PID_CONTROL_H
#include <Arduino.h>


class SimplePID {
  public:
    SimplePID(float Kp, float Ki, float Kd, float out_min, float out_max);

    void setParameters(float Kp, float Ki, float Kd, float out_min, float out_max);
    void setGains(float Kp, float Ki, float Kd);
    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
    void setOutLimit(float out_max, float out_min);
    void begin();
    float compute(float target, float actual);

  private:
    float error, errorPrev, errorInt, errorDot;
    unsigned long lastTime = 0;
    float kp, ki, kd;
    float outMax, outMin, outSat, outUnsat;
    bool outputIsClamped = false;
    bool integratorIsOn = true;

    void reset();
};


#endif
