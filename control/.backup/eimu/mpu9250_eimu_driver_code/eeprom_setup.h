#ifndef EEPROM_SETUP_H
#define EEPROM_SETUP_H

#include <EEPROM.h>
#include "global_eeprom_variables.h"

//--------- EEPROM ADDRESS ---------------//
const byte FIRST_TIME_EEPROM_ADDRESS = 0;

const byte AX_OFFSET_EEPROM_ADDRESS = 4;
const byte AY_OFFSET_EEPROM_ADDRESS = 8;
const byte AZ_OFFSET_EEPROM_ADDRESS = 12;

const byte GX_OFFSET_EEPROM_ADDRESS = 16;
const byte GY_OFFSET_EEPROM_ADDRESS = 20;
const byte GZ_OFFSET_EEPROM_ADDRESS = 24;

const byte B_VECT_X_EEPROM_ADDRESS = 28;
const byte B_VECT_Y_EEPROM_ADDRESS = 32;
const byte B_VECT_Z_EEPROM_ADDRESS = 36;

const byte A_MAT_00_EEPROM_ADDRESS = 40;
const byte A_MAT_01_EEPROM_ADDRESS = 44;
const byte A_MAT_02_EEPROM_ADDRESS = 48;

const byte A_MAT_10_EEPROM_ADDRESS = 52;
const byte A_MAT_11_EEPROM_ADDRESS = 56;
const byte A_MAT_12_EEPROM_ADDRESS = 60;

const byte A_MAT_20_EEPROM_ADDRESS = 64;
const byte A_MAT_21_EEPROM_ADDRESS = 68;
const byte A_MAT_22_EEPROM_ADDRESS = 72;

const byte R_VAR_EEPROM_ADDRESS = 76;
const byte P_VAR_EEPROM_ADDRESS = 80;
const byte Y_VAR_EEPROM_ADDRESS = 84;

const byte GX_VAR_EEPROM_ADDRESS = 88;
const byte GY_VAR_EEPROM_ADDRESS = 92;
const byte GZ_VAR_EEPROM_ADDRESS = 96;

const byte AX_VAR_EEPROM_ADDRESS = 100;
const byte AY_VAR_EEPROM_ADDRESS = 104;
const byte AZ_VAR_EEPROM_ADDRESS = 108;

const byte I2C_EEPROM_ADDRESS = 112;
const byte GAIN_EEPROM_ADDRESS = 116;
const byte FRAME_ID_EEPROM_ADDRESS = 120;
//-----------------------------------------//

float get_EEPROM_data(byte EEPROM_ADDRESS)
{
  float val;
  EEPROM.get(EEPROM_ADDRESS, val);
  return val;
}

void set_EEPROM_data(byte EEPROM_ADDRESS, float input_data, float &global_data)
{
  EEPROM.put(EEPROM_ADDRESS, input_data);
  float data;
  EEPROM.get(EEPROM_ADDRESS, data);
  global_data = data;

  if (EEPROM_ADDRESS == GAIN_EEPROM_ADDRESS)
    madgwickFilter.setAlgorithmGain(global_data);
}

void set_EEPROM_data(byte EEPROM_ADDRESS, int input_data, int &global_data)
{
  EEPROM.put(EEPROM_ADDRESS, (float)input_data);
  float data;
  EEPROM.get(EEPROM_ADDRESS, data);
  global_data = (int)data;

  if (EEPROM_ADDRESS == I2C_EEPROM_ADDRESS){
    Wire.begin(global_data);
  }
  else if (EEPROM_ADDRESS == FRAME_ID_EEPROM_ADDRESS){
    madgwickFilter.setWorldFrameId(global_data);
    resetOrientationValues();
  }
}

//////////////////////////////////////////////////////
void setFIRST_TIME(int val)
{
  EEPROM.put(FIRST_TIME_EEPROM_ADDRESS, (float)val);
}
int getFIRST_TIME()
{
  float firstTime;
  EEPROM.get(FIRST_TIME_EEPROM_ADDRESS, firstTime);
  return (int)firstTime;
}
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////

void resetAllParams()
{

  set_EEPROM_data(AX_OFFSET_EEPROM_ADDRESS, 0.00, axOff);
  set_EEPROM_data(AY_OFFSET_EEPROM_ADDRESS, 0.00, ayOff);
  set_EEPROM_data(AZ_OFFSET_EEPROM_ADDRESS, 0.00, azOff);

  set_EEPROM_data(GX_OFFSET_EEPROM_ADDRESS, 0.00, gxOff);
  set_EEPROM_data(GY_OFFSET_EEPROM_ADDRESS, 0.00, gyOff);
  set_EEPROM_data(GZ_OFFSET_EEPROM_ADDRESS, 0.00, gzOff);

  set_EEPROM_data(B_VECT_X_EEPROM_ADDRESS, 0.00, b_vect[0]);
  set_EEPROM_data(B_VECT_Y_EEPROM_ADDRESS, 0.00, b_vect[1]);
  set_EEPROM_data(B_VECT_Z_EEPROM_ADDRESS, 0.00, b_vect[2]);

  set_EEPROM_data(A_MAT_00_EEPROM_ADDRESS, 0.00, A_mat[0][0]);
  set_EEPROM_data(A_MAT_01_EEPROM_ADDRESS, 0.00, A_mat[0][1]);
  set_EEPROM_data(A_MAT_02_EEPROM_ADDRESS, 0.00, A_mat[0][2]);

  set_EEPROM_data(A_MAT_10_EEPROM_ADDRESS, 0.00, A_mat[1][0]);
  set_EEPROM_data(A_MAT_11_EEPROM_ADDRESS, 0.00, A_mat[1][1]);
  set_EEPROM_data(A_MAT_12_EEPROM_ADDRESS, 0.00, A_mat[1][2]);

  set_EEPROM_data(A_MAT_20_EEPROM_ADDRESS, 0.00, A_mat[2][0]);
  set_EEPROM_data(A_MAT_21_EEPROM_ADDRESS, 0.00, A_mat[2][1]);
  set_EEPROM_data(A_MAT_22_EEPROM_ADDRESS, 0.00, A_mat[2][2]);

  set_EEPROM_data(R_VAR_EEPROM_ADDRESS, 0.00, roll_variance);
  set_EEPROM_data(P_VAR_EEPROM_ADDRESS, 0.00, pitch_variance);
  set_EEPROM_data(Y_VAR_EEPROM_ADDRESS, 0.00, yaw_variance);

  set_EEPROM_data(GX_VAR_EEPROM_ADDRESS, 0.00, gxOff);
  set_EEPROM_data(GY_VAR_EEPROM_ADDRESS, 0.00, gyOff);
  set_EEPROM_data(GZ_VAR_EEPROM_ADDRESS, 0.00, gzOff);

  set_EEPROM_data(AX_VAR_EEPROM_ADDRESS, 0.00, axOff);
  set_EEPROM_data(AY_VAR_EEPROM_ADDRESS, 0.00, ayOff);
  set_EEPROM_data(AZ_VAR_EEPROM_ADDRESS, 0.00, azOff);

  set_EEPROM_data(GAIN_EEPROM_ADDRESS, 1.0, filterGain);
  set_EEPROM_data(I2C_EEPROM_ADDRESS, 104, i2cAddress);    // 0x68
  set_EEPROM_data(FRAME_ID_EEPROM_ADDRESS, 1, worldFrameId); // 0 - NWU,  1 - ENU,  2 - NED
}

void initEEPROMparamsStorage()
{
  int isFirstTime, setupCode = 11111; // please do not change
  isFirstTime = getFIRST_TIME();
  if (isFirstTime != setupCode)
  { // if not equal to 11111
    setFIRST_TIME(setupCode);
    resetAllParams();
    // Serial.println("reset all eeprom paramas");
  }
}

void updateGlobalParamsFromEERPOM()
{
  initEEPROMparamsStorage();

  axOff = get_EEPROM_data(AX_OFFSET_EEPROM_ADDRESS);
  ayOff = get_EEPROM_data(AY_OFFSET_EEPROM_ADDRESS);
  azOff = get_EEPROM_data(AZ_OFFSET_EEPROM_ADDRESS);

  gxOff = get_EEPROM_data(GX_OFFSET_EEPROM_ADDRESS);
  gyOff = get_EEPROM_data(GY_OFFSET_EEPROM_ADDRESS);
  gzOff = get_EEPROM_data(GZ_OFFSET_EEPROM_ADDRESS);

  b_vect[0] = get_EEPROM_data(B_VECT_X_EEPROM_ADDRESS);
  b_vect[1] = get_EEPROM_data(B_VECT_Y_EEPROM_ADDRESS);
  b_vect[2] = get_EEPROM_data(B_VECT_Z_EEPROM_ADDRESS);

  A_mat[0][0] = get_EEPROM_data(A_MAT_00_EEPROM_ADDRESS);
  A_mat[0][1] = get_EEPROM_data(A_MAT_01_EEPROM_ADDRESS);
  A_mat[0][2] = get_EEPROM_data(A_MAT_02_EEPROM_ADDRESS);

  A_mat[1][0] = get_EEPROM_data(A_MAT_10_EEPROM_ADDRESS);
  A_mat[1][1] = get_EEPROM_data(A_MAT_11_EEPROM_ADDRESS);
  A_mat[1][2] = get_EEPROM_data(A_MAT_12_EEPROM_ADDRESS);

  A_mat[2][0] = get_EEPROM_data(A_MAT_20_EEPROM_ADDRESS);
  A_mat[2][1] = get_EEPROM_data(A_MAT_21_EEPROM_ADDRESS);
  A_mat[2][2] = get_EEPROM_data(A_MAT_22_EEPROM_ADDRESS);

  roll_variance = get_EEPROM_data(R_VAR_EEPROM_ADDRESS);
  pitch_variance = get_EEPROM_data(P_VAR_EEPROM_ADDRESS);
  yaw_variance = get_EEPROM_data(Y_VAR_EEPROM_ADDRESS);

  gxVar = get_EEPROM_data(GX_VAR_EEPROM_ADDRESS);
  gyVar = get_EEPROM_data(GY_VAR_EEPROM_ADDRESS);
  gzVar = get_EEPROM_data(GZ_VAR_EEPROM_ADDRESS);

  axVar = get_EEPROM_data(AX_VAR_EEPROM_ADDRESS);
  ayVar = get_EEPROM_data(AY_VAR_EEPROM_ADDRESS);
  azVar = get_EEPROM_data(AZ_VAR_EEPROM_ADDRESS);

  filterGain = get_EEPROM_data(GAIN_EEPROM_ADDRESS);

  float address = get_EEPROM_data(I2C_EEPROM_ADDRESS);
  i2cAddress = (int)address;

  float world_frame_id = get_EEPROM_data(FRAME_ID_EEPROM_ADDRESS);
  worldFrameId = (int)world_frame_id;
}
/////////////////////////////////////////////////////////////

#endif