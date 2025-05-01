#include "l298n_motor_driver.h"

L298NMotorDriver::L298NMotorDriver(int IN1_pin, int IN2_pin, int en_pin)
{
  in1Pin = IN1_pin;
  in2Pin = IN2_pin;
  enPin = en_pin;

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}

void L298NMotorDriver::sendPWM(int pwmVal)
{
  if (pwmVal > 0)
  {
    analogWrite(enPin, abs(pwmVal));
    setForwardDirection();
  }
  else if (pwmVal < 0)
  {
    analogWrite(enPin, abs(pwmVal));
    setReverseDirection();
  }
  else
  {
    analogWrite(enPin, 0);
    setHalt();
  }
}

int L298NMotorDriver::getDirection()
{
  return dir;
}

void L298NMotorDriver::setForwardDirection()
{
  dir = 1;
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
}

void L298NMotorDriver::setReverseDirection()
{
  dir = 0;
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
}

void L298NMotorDriver::setHalt()
{
  dir = 0;
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}
