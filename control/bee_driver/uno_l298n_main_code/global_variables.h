#ifndef GLOBAL_VARIABLES
#define GLOBAL_VARIABLES
#include "encoder_setup.h"
#include "adaptiveLPF.h"
#include "l298n_motor_driver.h"
#include "simple_pid_control.h"
#define ALPHA 0.2 
///////////////////////////////////////////////////
// store encoder pulsePerRev needed by encoder
float encA_ppr = 1000.0;
float encB_ppr = 1000.0;
unsigned long encA_stopFreq = 10000; // in us
unsigned long encB_stopFreq = 10000; // in us

int encA_clkPin = 2, encA_dirPin = 8; // encA_ppr parameter is decleared globally in the global_variables.h file.
int encB_clkPin = 3, encB_dirPin = 9; // encB_ppr parameter is decleared globally in the global_variables.h file.

QuadEncoder encA(encA_clkPin, encA_dirPin, encA_ppr);
QuadEncoder encB(encB_clkPin, encB_dirPin, encB_ppr);

/////////////////////////////
float rdirA = 1.00;
float rdirB = 1.00;
/////////////////////////////

// adaptive lowpass Filter
int orderA = 1;
float cutOffFreqA = 1.0;

int orderB = 1;
float cutOffFreqB = 1.0;

// Filter instance
AdaptiveLowPassFilter lpfA(orderA, cutOffFreqA);
AdaptiveLowPassFilter lpfB(orderB, cutOffFreqB);

// motor A H-Bridge Connection
int IN1 = 6, IN2 = 7, enA = 5;
L298NMotorDriver motorA(IN1, IN2, enA);

// motor B H-Bridge Connection
int IN3 = 10, IN4 = 11, enB = 12;
L298NMotorDriver motorB(IN3, IN4, enB);

///////////////////////////////////////////////
float outMin = -255.0, outMax = 255.0;

// motorA pid control global params needed by pid
float kpA = 0.0, kiA = 0.0, kdA = 0.0;
float targetA = 0.00, filteredAngVelA;
float outputA;

// motorB pid control global params needed by pid
float kpB = 0.0, kiB = 0.0, kdB = 0.0;
float targetB = 0.00, filteredAngVelB;
float outputB;

// motorA pid control
SimplePID pidMotorA(kpA, kiA, kdA, outMin, outMax);

// motorA pid control
SimplePID pidMotorB(kpB, kiB, kdB, outMin, outMax);

// check if in PID or PWM mode
bool pidMode = true; // true-PID MODE, false-SETUP MODE

// initial i2cAddress
int i2cAddress = 1;

// calcute allowable maximum angular velocity (for overall smooth operation of the whole system)
float freq_per_tick_allowable = 2000.0; // Hz
float wA_allowable = 10.00;
float wB_allowable = 10.00;

float calc_wA_allowable()
{
  wA_allowable = (2 * PI * freq_per_tick_allowable) / encA_ppr;
  return wA_allowable;
}

float calc_wB_allowable()
{
  wB_allowable = (2 * PI * freq_per_tick_allowable) / encB_ppr;
  return wB_allowable;
}

// maximum motor velocity that can be commanded
float maxVelA = calc_wA_allowable(); // in radians/sec
float maxVelB = calc_wB_allowable(); // in radians/sec

// for command timeout.
unsigned long cmdVelTimeout, cmdVelTimeoutSampleTime = 0; // ms -> (1000/sampleTime) hz

const int voltagePin = A0; 
const int goPin = A1;
const float R1 = 2000.0;  // 10kΩ
const float R2 = 1000.0;   // 4.7kΩ
const float scaleFactor = (R1 + R2 + 15) / R2;
// yaw pitch roll
float angles[3]; 
float roll, pitch, yaw;
float filteredBattery = 0;
int batteryPercent = 0;
int go_btn;
unsigned long previousBatteryMillis = 0;  
const long batteryInterval = 1000;

#endif
