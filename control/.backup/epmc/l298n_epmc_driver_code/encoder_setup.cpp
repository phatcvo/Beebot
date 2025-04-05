#include "encoder_setup.h"

QuadEncoder::QuadEncoder(int clk_pin, int dir_pin, float ppr)
{
  clkPin = clk_pin;
  dirPin = dir_pin;
  pulsePerRev = ppr;

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dirPin, INPUT_PULLUP);

  oldFreqTime = micros();
  checkFreqTime = micros();
}

void QuadEncoder::setPulsePerRev(float ppr)
{
  pulsePerRev = ppr;
}

float QuadEncoder::getAngPos()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    return (2.00 * PI * (float)tickCount) / pulsePerRev;
  }
}

float QuadEncoder::getAbsAngPosDeg()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    return absAngDeg((2.00 * PI * (float)tickCount) / pulsePerRev);
  }
}

float QuadEncoder::getAngVel()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    return 2.00 * PI * frequency;
  }
}

void QuadEncoder::setStopFreqInUs(unsigned long freq)
{
  freqSampleTime = freq;
}

void QuadEncoder::resetFrequency()
{
  if (micros() - checkFreqTime >= freqSampleTime)
  {
    frequency = 0;
  }
}

float QuadEncoder::absAngDeg(float incAngRad)
{
  float incAngDeg = incAngRad * 180.0 / PI;
  return (int)incAngDeg % 360;
}