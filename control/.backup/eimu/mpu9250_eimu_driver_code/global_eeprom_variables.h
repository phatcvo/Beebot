#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES

#include <Wire.h>
#include "madgwick_filter.h"

MadgwickFilter madgwickFilter;

//------------- ACC global variables ---------------//
float axRaw = 0.00;
float ayRaw = 0.00;
float azRaw = 0.00;

float axCal = 0.00;
float ayCal = 0.00;
float azCal = 0.00;

float axOff = 0.00;
float ayOff = 0.00;
float azOff = 0.00;

float axVar = 0.00;
float ayVar = 0.00;
float azVar = 0.00;
//---------------------------------------------------//

//-------------GYR Global Variables -----------------//
float gxRaw = 0.00;
float gyRaw = 0.00;
float gzRaw = 0.00;

float gxCal = 0.00;
float gyCal = 0.00;
float gzCal = 0.00;

float gxOff = 0.00;
float gyOff = 0.00;
float gzOff = 0.00;

float gxVar = 0.00;
float gyVar = 0.00;
float gzVar = 0.00;
//---------------------------------------------------//

//------------- MAG Global Variables ----------------//
// magCal = A*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
float mxRaw = 0.00;
float myRaw = 0.00;
float mzRaw = 0.00;

float mxCal = 0.00;
float myCal = 0.00;
float mzCal = 0.00;

float A_mat[3][3];
float b_vect[3];
float mag_vect[3];
//---------------------------------------------------//

//------------- RPY Global Variables ----------------//
float roll = 0.00;
float pitch = 0.00;
float yaw = 0.00;

float roll_variance = 0.00;
float pitch_variance = 0.00;
float yaw_variance = 0.00;

float qw = 0.00;
float qx = 0.00;
float qy = 0.00;
float qz = 0.00;

void resetOrientationValues(){
  roll = 0.00;
  pitch = 0.00;
  yaw = 0.00;
  
  qw = 0.00;
  qx = 0.00;
  qy = 0.00;
  qz = 0.00;
}
//---------------------------------------------------//

//---------------------------------------------------//
int i2cAddress = 104; // 0x68
float filterGain = 1.0;
int worldFrameId = 1; // 0 - NWU,  1 - ENU,  2 - NED
//---------------------------------------------------//

#endif