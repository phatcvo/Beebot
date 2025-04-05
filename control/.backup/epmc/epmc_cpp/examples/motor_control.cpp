
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "epmc.hpp"

EPMC motorControl;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{

  bool sendHigh = false;

  float lowTargetVel = -4.0; // in rad/sec
  float highTargetVel = 4.0; // in rad/sec

  float angPosA, angPosB;
  float angVelA, angVelB;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.02;

  auto ctrlPrevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> ctrlDuration;
  float ctrlSampleTime = 5.0;

  // std::string port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0";
  std::string port = "/dev/ttyACM0";
  motorControl.connect(port);

  // wait for the motorControl to fully setup
  for (int i = 1; i <= 5; i += 1)
  {
    delay_ms(1000);
    std::cout << "configuring controller: " << i << " sec" << std::endl;
  }
  motorControl.sendTargetVel(0.0, 0.0); // targetA, targetB
  std::cout << "configuration complete" << std::endl;

  int motor_cmd_timeout_ms = 3000;
  motorControl.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motorControl.getCmdTimeout(motor_cmd_timeout_ms);
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  motorControl.sendTargetVel(lowTargetVel, lowTargetVel); // targetA, targetB
  sendHigh = true;

  prevTime = std::chrono::system_clock::now();
  ctrlPrevTime = std::chrono::system_clock::now();

  while (true)
  {

    ctrlDuration = (std::chrono::system_clock::now() - ctrlPrevTime);
    if (ctrlDuration.count() > ctrlSampleTime)
    {
      if (sendHigh)
      {
        motorControl.sendTargetVel(highTargetVel, highTargetVel); // targetA, targetB
        sendHigh = false;
      }
      else
      {
        motorControl.sendTargetVel(lowTargetVel, lowTargetVel); // targetA, targetB
        sendHigh = true;
      }

      ctrlPrevTime = std::chrono::system_clock::now();
    }

    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      try
      {
        motorControl.getMotorsPos(angPosA, angPosB); // gets angPosA, angPosB
        motorControl.getMotorsVel(angVelA, angVelB); // gets angVelA, angVelB
      }
      catch (...)
      {
        // std::cout << "motorA_readings: [" << angPosA << std::fixed << std::setprecision(4) << "," << angVelA << std::fixed << std::setprecision(4) << "]" << std::endl;
        // std::cout << "motorB_readings: [" << angPosB << std::fixed << std::setprecision(4) << "," << angVelB << std::fixed << std::setprecision(4) << "]" << '\n' << std::endl;
      }

      std::cout << "motorA_readings: [" << angPosA << std::fixed << std::setprecision(4) << "," << angVelA << std::fixed << std::setprecision(4) << "]" << std::endl;
      std::cout << "motorB_readings: [" << angPosB << std::fixed << std::setprecision(4) << "," << angVelB << std::fixed << std::setprecision(4) << "]" << '\n'
                << std::endl;

      prevTime = std::chrono::system_clock::now();
    }
  }
}