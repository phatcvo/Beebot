#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include <util/atomic.h>


class QuadEncoder {
public:
  int clkPin, dirPin;
  float pulsePerRev;
  volatile long tickCount;
  float prevAngPos, angPos, angVel, freqPerTick, frequency;
  volatile unsigned long oldFreqTime, checkFreqTime, freqSampleTime=2000;
  long testTickCount = 0;

  QuadEncoder(int clk_pin, int dir_pin, float ppr);

  void setPulsePerRev(float ppr);
  float getAngPos();
  float getAbsAngPosDeg();
  float getAngVel();
  void setStopFreqInUs(unsigned long freq);
  void resetFrequency();

private:
  float absAngDeg(float incAngRad);
};


#endif


