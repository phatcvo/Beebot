#include "l298n_motor_driver.h"

L298NMotorDriver::L298NMotorDriver(int DR_pin, int PWM_pin, int EN_pin)
{
  drPin = DR_pin;
  enPin = EN_pin;
  pwmPin = PWM_pin;

  pinMode(drPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  digitalWrite(drPin, LOW);
  digitalWrite(enPin, LOW);
}

void L298NMotorDriver::sendPWM(int pwmVal, bool invert)
{
  if(!invert){
    if (pwmVal > 0)
    {
      analogWrite(pwmPin, abs(pwmVal));
      setForwardDirection();
    }
    else if (pwmVal < 0)
    {
      analogWrite(pwmPin, abs(pwmVal));
      setReverseDirection();
    }
    else
    {
      analogWrite(pwmPin, 0);
      setHalt();
    }
  }else{
    if (pwmVal > 0)
    {
      analogWrite(pwmPin, abs(pwmVal));
      setReverseDirection();
    }
    else if (pwmVal < 0)
    {
      analogWrite(pwmPin, abs(pwmVal));
      setForwardDirection();
    }
    else
    {
      analogWrite(pwmPin, 0);
      setHalt();
    }
  }
}

int L298NMotorDriver::getDirection()
{
  return dir;
}

void L298NMotorDriver::setForwardDirection()
{
  dir = 1;
  digitalWrite(drPin, LOW);
  digitalWrite(enPin, HIGH);
}

void L298NMotorDriver::setReverseDirection()
{
  dir = 0;
  digitalWrite(drPin, HIGH);
  digitalWrite(enPin, HIGH);
}

void L298NMotorDriver::setHalt()
{
  dir = 0;
  digitalWrite(enPin, LOW);
}
