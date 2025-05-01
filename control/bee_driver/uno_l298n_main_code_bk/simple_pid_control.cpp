#include "simple_pid_control.h"

SimplePID::SimplePID(float Kp, float Ki, float Kd, float out_min, float out_max)
{
  reset();

  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;

  errorPrev = error;
  lastTime = micros();
}

void SimplePID::setParameters(float Kp, float Ki, float Kd, float out_min, float out_max)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::setGains(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void SimplePID::setKp(float Kp)
{
  kp = Kp;
}

void SimplePID::setKi(float Ki)
{
  ki = Ki;
}

void SimplePID::setKd(float Kd)
{
  kd = Kd;
}

void SimplePID::setOutLimit(float out_max, float out_min)
{
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::begin()
{
  reset();
  lastTime = micros();
}

float SimplePID::compute(float target, float actual)
{
  float sampleFreq = 1.0e6 / (float)(micros() - lastTime);

  error = target - actual;

  errorDot = (error - errorPrev) * sampleFreq;

  if (integratorIsOn)
  {
    errorInt += (error / sampleFreq);
  }
  else
  {
    errorInt += 0.0;
  }

  outUnsat = (kp * error) + (ki * errorInt) + (kd * errorDot);

  if (outUnsat > outMax)
  {
    outSat = outMax;
    outputIsClamped = true;
  }
  else if (outUnsat < outMin)
  {
    outSat = outMin;
    outputIsClamped = true;
  }
  else
  {
    outSat = outUnsat;
    outputIsClamped = false;
  }

  //      if(!outputIsClamped && !((error*outUnsat)<0)){
  //        integratorIsOn = true;
  //      }
  if (!outputIsClamped)
  {
    integratorIsOn = true;
  }
  else
  {
    integratorIsOn = false;
  }

  //      Serial.println(integratorIsOn);
  //      Serial.println();

  errorPrev = error;
  lastTime = micros();

  return outSat;
}

void SimplePID::reset()
{
  error = 0.0;
  errorPrev = 0.0;
  errorInt = 0.0;
  errorDot = 0.0;
  outSat = 0.0;
  outUnsat = 0.0;
  lastTime = 0;
}
