/////////////////// STORING AND READING PARAMETERS FROM EEPROM /////////////////
#include <EEPROM.h>
#include <Wire.h>
#include "global_variables.h"

int KP_A_ADDRESS = 0;
int KP_B_ADDRESS = 4;

int KI_A_ADDRESS = 8;
int KI_B_ADDRESS = 12;

int KD_A_ADDRESS = 16;
int KD_B_ADDRESS = 20;

int PPR_A_ADDRESS = 24;
int PPR_B_ADDRESS = 28;

int I2C_ADDRESS = 32;

int FIRST_TIME_ADDRESS = 36;

int FILTER_ORDER_A_ADDRESS = 40;
int FILTER_ORDER_B_ADDRESS = 44;

int FILTER_F0_A_ADDRESS = 48;
int FILTER_F0_B_ADDRESS = 52;

int STOP_FREQ_A_ADDRESS = 56;
int STOP_FREQ_B_ADDRESS = 60;

int RDIR_A_ADDRESS = 64;
int RDIR_B_ADDRESS = 68;

int MAX_VEL_A_ADDRESS = 72;
int MAX_VEL_B_ADDRESS = 76;

int FREQ_ALLOWABLE_ADDRESS = 80;








/////////////////////////////////////////
float getAllowableFreq(){
  float freq_allowed;
  EEPROM.get(FREQ_ALLOWABLE_ADDRESS, freq_allowed);
  return freq_allowed;
}
void setAllowableFreq(float freq_allowed){
  EEPROM.put(FREQ_ALLOWABLE_ADDRESS, freq_allowed);
  freq_per_tick_allowable = getAllowableFreq();
}
//////////////////////////////////////////////////////


//////////////////////////////////////////
float getMAXVEL_A(){
  float max_vel;
  EEPROM.get(MAX_VEL_A_ADDRESS, max_vel);
  return max_vel;
}
void setMAXVEL_A(float max_vel){
  EEPROM.put(MAX_VEL_A_ADDRESS, max_vel);
  maxVelA = getMAXVEL_A();
}
//////////////////////////////////////////


/////////////////////////////////////////
float getMAXVEL_B(){
  float max_vel;
  EEPROM.get(MAX_VEL_B_ADDRESS, max_vel);
  return max_vel;
}
void setMAXVEL_B(float max_vel){
  EEPROM.put(MAX_VEL_B_ADDRESS, max_vel);
  maxVelB = getMAXVEL_B();
}
//////////////////////////////////////////



//////////////////////////////////////
float getKP_A(){
  float KpA;
  EEPROM.get(KP_A_ADDRESS, KpA);
  return KpA;
}
void setKP_A(float KpA){
  EEPROM.put(KP_A_ADDRESS, KpA);
  kpA = getKP_A();
  pidMotorA.setKp(kpA);
}


float getKP_B(){
  float KpB;
  EEPROM.get(KP_B_ADDRESS, KpB);
  return KpB;
}
void setKP_B(float KpB){
  EEPROM.put(KP_B_ADDRESS, KpB);
  kpB = getKP_B();
  pidMotorB.setKp(kpB);
}
//////////////////////////////////////////



//////////////////////////////////////////
float getKI_A(){
  float KiA;
  EEPROM.get(KI_A_ADDRESS, KiA);
  return KiA;
}
void setKI_A(float KiA){
  EEPROM.put(KI_A_ADDRESS, KiA);
  kiA = getKI_A();
  pidMotorA.setKi(kiA);
}


float getKI_B(){
  float KiB;
  EEPROM.get(KI_B_ADDRESS, KiB);
  return KiB;
}
void setKI_B(float KiB){
  EEPROM.put(KI_B_ADDRESS, KiB);
  kiB = getKI_B();
  pidMotorB.setKi(kiB);
}
////////////////////////////////////////



////////////////////////////////////////
float getKD_A(){
  float KdA;
  EEPROM.get(KD_A_ADDRESS, KdA);
  return KdA;
}
void setKD_A(float KdA){
  EEPROM.put(KD_A_ADDRESS, KdA);
  kdA = getKD_A();
  pidMotorA.setKd(kdA);
}


float getKD_B(){
  float KdB;
  EEPROM.get(KD_B_ADDRESS, KdB);
  return KdB;
}
void setKD_B(float KdB){
  EEPROM.put(KD_B_ADDRESS, KdB);
  kdB = getKD_B();
  pidMotorB.setKd(kdB);
}
/////////////////////////////////////////




/////////////////////////////////////////
float getPPR_A(){
  float pprA;
  EEPROM.get(PPR_A_ADDRESS, pprA);
  return pprA;
}
void setPPR_A(float pprA){
  EEPROM.put(PPR_A_ADDRESS, pprA);
  encA_ppr = getPPR_A();
  encA.setPulsePerRev(encA_ppr);

  float wA_allowable = calc_wA_allowable(); // in radians/sec
  setMAXVEL_A(wA_allowable);
}


float getPPR_B(){
  float pprB;
  EEPROM.get(PPR_B_ADDRESS, pprB);
  return pprB;
}
void setPPR_B(float pprB){
  EEPROM.put(PPR_B_ADDRESS, pprB);
  encB_ppr = getPPR_B();
  encB.setPulsePerRev(encB_ppr);

  float wB_allowable = calc_wB_allowable(); // in radians/sec
  setMAXVEL_B(wB_allowable);
}
/////////////////////////////////////////




/////////////////////////////////////////
int getI2CADDRESS(){
  float address;
  EEPROM.get(I2C_ADDRESS, address);
  return (int)address;
}
void setI2CADDRESS(int address){
  EEPROM.put(I2C_ADDRESS, (float)address);
  i2cAddress = getI2CADDRESS();
  Wire.begin(i2cAddress);
}
///////////////////////////////////////////





////////////////////////////////////////////////
void setFIRST_TIME(int val){
  EEPROM.put(FIRST_TIME_ADDRESS, (float)val);
}
int getFIRST_TIME(){
  float firstTime;
  EEPROM.get(FIRST_TIME_ADDRESS, firstTime);
  return (int)firstTime;
}
/////////////////////////////////////////////////





////////////////////////////////////////////////////////////
float getFilterOrder_A(){
  float filterOrder;
  EEPROM.get(FILTER_ORDER_A_ADDRESS, filterOrder);
  return (int)filterOrder;
}
void setFilterOrder_A(int filterOrder){
  EEPROM.put(FILTER_ORDER_A_ADDRESS, (float)filterOrder);
  orderA = getFilterOrder_A();
  lpfA.setOrder(orderA);
}


float getFilterOrder_B(){
  float filterOrder;
  EEPROM.get(FILTER_ORDER_B_ADDRESS, filterOrder);
  return (int)filterOrder;
}
void setFilterOrder_B(int filterOrder){
  EEPROM.put(FILTER_ORDER_B_ADDRESS, (float)filterOrder);
  orderB = getFilterOrder_B();
  lpfB.setOrder(orderB);
}
/////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////// 
float getFilterCutOffFreq_A(){
  float filterCutOffFreq;
  EEPROM.get(FILTER_F0_A_ADDRESS, filterCutOffFreq);
  return filterCutOffFreq;
}
void setFilterCutOffFreq_A(float filterCutOffFreq){
  EEPROM.put(FILTER_F0_A_ADDRESS, filterCutOffFreq);
  cutOffFreqA = getFilterCutOffFreq_A();
  lpfA.setCutOffFreq(cutOffFreqA);
}


float getFilterCutOffFreq_B(){
  float filterCutOffFreq;
  EEPROM.get(FILTER_F0_B_ADDRESS, filterCutOffFreq);
  return filterCutOffFreq;
}
void setFilterCutOffFreq_B(float filterCutOffFreq){
  EEPROM.put(FILTER_F0_B_ADDRESS, filterCutOffFreq);
  cutOffFreqB = getFilterCutOffFreq_B();
  lpfB.setCutOffFreq(cutOffFreqB);
}
///////////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////
float getStopFreq_A(){
  float stopFreq;
  EEPROM.get(STOP_FREQ_A_ADDRESS, stopFreq);
  return (unsigned long)stopFreq;
}
void setStopFreq_A(float stopFreq){
  EEPROM.put(STOP_FREQ_A_ADDRESS, stopFreq);
  encA_stopFreq = getStopFreq_A();
  encA.setStopFreqInUs(encA_stopFreq);
}


float getStopFreq_B(){
  float stopFreq;
  EEPROM.get(STOP_FREQ_B_ADDRESS, stopFreq);
  return (unsigned long)stopFreq;
}
void setStopFreq_B(float stopFreq){
  EEPROM.put(STOP_FREQ_B_ADDRESS, stopFreq);
  encB_stopFreq = getStopFreq_B();
  encB.setStopFreqInUs(encB_stopFreq);
}
///////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////
float getRDIR_A(){
  float rDirA;
  EEPROM.get(RDIR_A_ADDRESS, rDirA);
  return rDirA;
}
void setRDIR_A(float rDirA){
  EEPROM.put(RDIR_A_ADDRESS, rDirA);
  rdirA = getRDIR_A();
}


float getRDIR_B(){
  float rDirB;
  EEPROM.get(RDIR_B_ADDRESS, rDirB);
  return rDirB;
}
void setRDIR_B(float rDirB){
  EEPROM.put(RDIR_B_ADDRESS, rDirB);
  rdirB = getRDIR_B();
}
//////////////////////////////////////////////////////////
















void resetAllParams(){
  setKP_A(0.00);
  setKP_B(0.00);

  setKI_A(0.00);
  setKI_B(0.00);

  setKD_A(0.00);
  setKD_B(0.00);

  setPPR_A(1000.00);
  setPPR_B(1000.00);

  setI2CADDRESS(1);

  setFilterOrder_A(1);
  setFilterOrder_B(1);

  setFilterCutOffFreq_A(1.00);
  setFilterCutOffFreq_B(1.00);

  setStopFreq_A(10000);
  setStopFreq_B(10000);

  setRDIR_A(1.00);
  setRDIR_B(1.00);

  float wA_allowable = calc_wA_allowable(); // in radians/sec
  float wB_allowable = calc_wB_allowable(); // in radians/sec
  setMAXVEL_A(wA_allowable);
  setMAXVEL_B(wB_allowable);

  setAllowableFreq(2000.00);
}



void initEEPROMparamsStorage(){
  int isFirstTime, setupCode = 11111; // please do not change
  isFirstTime = getFIRST_TIME();
  if(isFirstTime != setupCode){ //if not equal to 11111
    setFIRST_TIME(setupCode);
    resetAllParams();
  }
}



void updateGlobalParamsFromEERPOM(){
  initEEPROMparamsStorage();

  kpA = getKP_A();
  kpB = getKP_B();

  kiA = getKI_A();
  kiB = getKI_B();

  kdA = getKD_A();
  kdB = getKD_B();

  encA_ppr = getPPR_A();
  encB_ppr = getPPR_B();

  i2cAddress = getI2CADDRESS();

  orderA = getFilterOrder_A();
  orderB = getFilterOrder_B();

  cutOffFreqA = getFilterCutOffFreq_A();
  cutOffFreqB = getFilterCutOffFreq_B();

  encA_stopFreq = getStopFreq_A();
  encB_stopFreq = getStopFreq_B();

  rdirA = getRDIR_A();
  rdirB = getRDIR_B();

  maxVelA = getMAXVEL_A();
  maxVelB = getMAXVEL_B();

  freq_per_tick_allowable = getAllowableFreq();
}
/////////////////////////////////////////////////////////////