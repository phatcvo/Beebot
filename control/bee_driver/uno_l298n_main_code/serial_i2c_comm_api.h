#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include <Arduino.h>
#include <Wire.h>
#include "eeprom_setup.h"

float maxFloat = 99999.888, minFloat = -99999.888;
long maxLong = 2147000000, minLong = -2147000000;

void initLed0()
{
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
}
void onLed0()
{
  digitalWrite(A0, HIGH);
  digitalWrite(A1, LOW);
}
void offLed0()
{
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
}

void initLed1()
{
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
}
void onLed1()
{
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
}
void offLed1()
{
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

///////// DIFFERENT TASK FOR SERIAL AND I2C COMMUNICATION //////////

/////////////////////////////////////////////////////////////////////////////////////
String sendMotorsPos()
{
  float angPosA = encA.getAngPos();
  float angPosB = encB.getAngPos();
  String data = String(constrain(rdirA * angPosA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB * angPosB, minFloat, maxFloat), 4);
  return data;
}
String sendMotorsPos3dp()
{
  float angPosA = encA.getAngPos();
  float angPosB = encB.getAngPos();
  String data = String(constrain(rdirA * angPosA, minFloat, maxFloat), 3);
  data += ",";
  data += String(constrain(rdirB * angPosB, minFloat, maxFloat), 3);
  return data;
}

String sendMotorsVel()
{
  String data = String(constrain(rdirA * filteredAngVelA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB * filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
String sendMotorAVel()
{
  float rawAngVelA = encA.getAngVel();
  String data = String(constrain(rdirA * rawAngVelA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirA * filteredAngVelA, minFloat, maxFloat), 4);
  return data;
}

String sendMotorBVel()
{
  float rawAngVelB = encB.getAngVel();
  String data = String(constrain(rdirB * rawAngVelB, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB * filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
String sendMotorAVelPID()
{
  String data = String(constrain(rdirA * targetA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirA * filteredAngVelA, minFloat, maxFloat), 4);
  return data;
}

String sendMotorBVelPID()
{
  String data = String(constrain(rdirB * targetB, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB * filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
String sendMotorAData()
{
  float angPosA = encA.getAngPos();
  String data = String(constrain(rdirA * angPosA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirA * filteredAngVelA, minFloat, maxFloat), 4);
  return data;
}

String sendMotorBData()
{
  float angPosB = encB.getAngPos();
  String data = String(constrain(rdirB * angPosB, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB * filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
String setMotorsPwm(int valA, int valB)
{
  if (!pidMode)
  {
    motorA.sendPWM((int)rdirA * valA);
    motorB.sendPWM((int)rdirB * valB);
    cmdVelTimeout = millis();
    return "1";
  }
  else
    return "0";
}

String setMotorsTarget(float valA, float valB)
{
  float tVelA = constrain(valA, -1.00 * maxVelA, maxVelA);
  float tVelB = constrain(valB, -1.00 * maxVelB, maxVelB);

  targetA = rdirA * tVelA;
  targetB = rdirB * tVelB;

  cmdVelTimeout = millis();
  return "1";
}
/////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
String setPidModeFunc(int mode)
{
  if (mode == 0)
  {
    pidMode = false;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    pidMotorA.begin();
    pidMotorB.begin();
  }
  else if (mode == 1)
  {
    pidMode = true;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    pidMotorA.begin();
    pidMotorB.begin();
  }
}
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
String setEncAppr(float ppr)
{
  setPPR_A(ppr);
  encA_ppr = getPPR_A();
  return "1";
}
String sendEncAppr()
{
  return String(encA_ppr);
}

String setEncBppr(float ppr)
{
  setPPR_B(ppr);
  encB_ppr = getPPR_B();
  return "1";
}
String sendEncBppr()
{
  return String(encB_ppr);
}
//////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
String setMotorAkp(float kp)
{
  setKP_A(kp);
  kpA = getKP_A();
  return "1";
}
String sendMotorAkp()
{
  return String(kpA, 4);
}

String setMotorBkp(float kp)
{
  setKP_B(kp);
  kpB = getKP_B();
  return "1";
}
String sendMotorBkp()
{
  return String(kpB, 4);
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
String setMotorAki(float ki)
{
  setKI_A(ki);
  kiA = getKI_A();
  return "1";
}
String sendMotorAki()
{
  return String(kiA, 4);
}

String setMotorBki(float ki)
{
  setKI_B(ki);
  kiB = getKI_B();
  return "1";
}
String sendMotorBki()
{
  return String(kiB, 4);
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
String setMotorAkd(float kd)
{
  setKD_A(kd);
  kdA = getKD_A();
  return "1";
}
String sendMotorAkd()
{
  return String(kdA, 4);
}

String setMotorBkd(float kd)
{
  setKD_B(kd);
  kdB = getKD_B();
  return "1";
}
String sendMotorBkd()
{
  return String(kdB, 4);
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
String setFilterOrderA(int order)
{
  if (order > 2)
    setFilterOrder_A(2);
  else if (order < 1)
    setFilterOrder_A(1);
  else
    setFilterOrder_A(order);
  orderA = getFilterOrder_A();
  return "1";
}
String sendFilterOrderA()
{
  return String(orderA);
}

String setFilterOrderB(int order)
{
  if (order > 2)
    setFilterOrder_B(2);
  else if (order < 1)
    setFilterOrder_B(1);
  else
    setFilterOrder_B(order);
  orderB = getFilterOrder_B();
  return "1";
}
String sendFilterOrderB()
{
  return String(orderB);
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
String setCutOffFreqA(float f0)
{
  setFilterCutOffFreq_A(f0);
  cutOffFreqA = getFilterCutOffFreq_A();
  return "1";
}
String sendCutOffFreqA()
{
  return String(cutOffFreqA);
}

String setCutOffFreqB(float f0)
{
  setFilterCutOffFreq_B(f0);
  cutOffFreqB = getFilterCutOffFreq_B();
  return "1";
}
String sendCutOffFreqB()
{
  return String(cutOffFreqB);
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
String setStopFreqA(float freq)
{
  setStopFreq_A(freq);
  encA_stopFreq = getStopFreq_A();
  return "1";
}
String sendStopFreqA()
{
  return String(encA_stopFreq);
}

String setStopFreqB(float freq)
{
  setStopFreq_B(freq);
  encB_stopFreq = getStopFreq_B();
  return "1";
}
String sendStopFreqB()
{
  return String(encB_stopFreq);
}
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
String setRdirA(float rDirA)
{
  if (rDirA >= 0.0)
  {
    setRDIR_A(1.00);
    rdirA = getRDIR_A();
  }
  else
  {
    setRDIR_A(-1.00);
    rdirA = getRDIR_A();
  }
  return "1";
}
String sendRdirA()
{
  return String(rdirA);
}

String setRdirB(float rDirB)
{
  if (rDirB >= 0.0)
  {
    setRDIR_B(1.00);
    rdirB = getRDIR_B();
  }
  else
  {
    setRDIR_B(-1.00);
    rdirB = getRDIR_B();
  }
  return "1";
}
String sendRdirB()
{
  return String(rdirB);
}
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
String setI2Caddress(int address)
{
  setI2CADDRESS(address);
  i2cAddress = getI2CADDRESS();
  Wire.begin(i2cAddress);
  return "1";
}
String sendI2Caddress()
{
  return String(i2cAddress);
}

String resetEEPROM()
{
  setFIRST_TIME(0);
  // updateGlobalParamsFromEERPOM();
  return "1";
}
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////
String setMaxVelA(float vel)
{
  if (!pidMode)
  {
    float max_vel = abs(vel);
    setMAXVEL_A(constrain(max_vel, -1.00 * wA_allowable, wA_allowable));
    maxVelA = getMAXVEL_A();
    return "1";
  }
  else
    return "0";
}
String sendMaxVelA()
{
  return String(maxVelA, 0);
}

String setMaxVelB(float vel)
{
  if (!pidMode)
  {
    float max_vel = abs(vel);
    setMAXVEL_B(constrain(max_vel, -1.00 * wB_allowable, wB_allowable));
    maxVelB = getMAXVEL_B();
    return "1";
  }
  else
    return "0";
}
String sendMaxVelB()
{
  return String(maxVelB, 0);
}

String setAllowedFreq(float freq)
{
  if (!pidMode)
  {
    setAllowableFreq(freq);
    freq_per_tick_allowable = getAllowableFreq();
    return "1";
  }
  else
    return "0";
}
String sendAllowedFreq()
{
  return String(freq_per_tick_allowable, 2);
}

String setCmdTimeout(int timeout)
{
  int cmdTimeout = timeout;
  if (cmdTimeout < 10)
  {
    cmdVelTimeoutSampleTime = 0;
  }
  else
  {
    cmdVelTimeoutSampleTime = cmdTimeout;
  }

  cmdVelTimeout = millis();
  return "1";
}
String sendCmdTimeout()
{
  return String(cmdVelTimeoutSampleTime);
}
////////////////////////////////////////////

///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[3];

void serialReceiveAndSendData()
{
  int indexPos = 0, i = 0;

  if (Serial.available() > 0)
  {
    while (Serial.available())
    {
      serMsg = Serial.readStringUntil('\n');
      // serMsg = Serial.readString();
    }
    serMsg.trim();
    if (serMsg != "")
    {
      do
      {
        indexPos = serMsg.indexOf(',');
        if (indexPos != -1)
        {
          serMsgBuffer = serMsg.substring(0, indexPos);
          serMsg = serMsg.substring(indexPos + 1, serMsg.length());
          serDataBuffer[i] = serMsgBuffer;
          serMsgBuffer = "";
        }
        else
        {
          if (serMsg.length() > 0)
            serDataBuffer[i] = serMsg;
        }
        i += 1;
      } while (indexPos >= 0);
    }

    if (serDataBuffer[0] != "")
    {

      onLed1();

      if (serDataBuffer[0] == "/pos")
      {
        ser_msg = sendMotorsPos();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/vel")
      {
        ser_msg = sendMotorsVel();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/velA")
      {
        ser_msg = sendMotorAVel();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/velB")
      {
        ser_msg = sendMotorBVel();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pVelA")
      {
        ser_msg = sendMotorAVelPID();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pVelB")
      {
        ser_msg = sendMotorBVelPID();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/dataA")
      {
        ser_msg = sendMotorAData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/dataB")
      {
        ser_msg = sendMotorBData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pwm")
      {
        int pwm_a = serDataBuffer[1].toInt();
        int pwm_b = serDataBuffer[2].toInt();
        ser_msg = setMotorsPwm(constrain(pwm_a, -255, 255), constrain(pwm_b, -255, 255));
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/tag")
      {
        ser_msg = setMotorsTarget(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pprA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendEncAppr();
        else
          ser_msg = setEncAppr(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pprB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendEncBppr();
        else
          ser_msg = setEncBppr(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kpA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMotorAkp();
        else
          ser_msg = setMotorAkp(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kpB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMotorBkp();
        else
          ser_msg = setMotorBkp(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kiA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMotorAki();
        else
          ser_msg = setMotorAki(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kiB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMotorBki();
        else
          ser_msg = setMotorBki(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kdA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMotorAkd();
        else
          ser_msg = setMotorAkd(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kdB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMotorBkd();
        else
          ser_msg = setMotorBkd(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/ordA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendFilterOrderA();
        else
          ser_msg = setFilterOrderA(serDataBuffer[1].toInt());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/ordB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendFilterOrderB();
        else
          ser_msg = setFilterOrderB(serDataBuffer[1].toInt());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/f0A")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendCutOffFreqA();
        else
          ser_msg = setCutOffFreqA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/f0B")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendCutOffFreqB();
        else
          ser_msg = setCutOffFreqB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/sfA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendStopFreqA();
        else
          ser_msg = setStopFreqA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/sfB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendStopFreqB();
        else
          ser_msg = setStopFreqB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/rdirA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendRdirA();
        else
          ser_msg = setRdirA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/rdirB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendRdirB();
        else
          ser_msg = setRdirB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/i2c")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendI2Caddress();
        else
        {
          int i2c_address = serDataBuffer[1].toInt();
          ser_msg = setI2Caddress(constrain(i2c_address, 0, 127));
        }
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/reset")
      {
        ser_msg = resetEEPROM();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/freq")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendAllowedFreq();
        else
          ser_msg = setAllowedFreq(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/maxVelA")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMaxVelA();
        else
          ser_msg = setMaxVelA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/maxVelB")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendMaxVelB();
        else
          ser_msg = setMaxVelB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/timeout")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendCmdTimeout();
        else
          ser_msg = setCmdTimeout(serDataBuffer[1].toInt());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      offLed1();
    }
    else
    {
      ser_msg = "0";
      Serial.println(ser_msg);
      ser_msg = "";
    }
  }

  serMsg = "";
  serMsgBuffer = "";
  serDataBuffer[0] = "";
  serDataBuffer[1] = "";
  serDataBuffer[2] = "";
}
//////////////////////////////////////////////////////////////////////////

//----------------- I2C COMMUNICATION -------------------//

String i2c_msg = "";

String i2cMsg = "", i2cMsgBuffer, i2cDataBuffer[3];

void i2cSlaveSendData()
{
  String msg = "";
  if (i2c_msg != "")
  {
    msg = i2c_msg;
    i2c_msg = "";
  }
  else
  {
    msg = "0";
    i2c_msg = "";
  }
  char charArray[msg.length() + 1];
  msg.toCharArray(charArray, msg.length() + 1);
  Wire.write(charArray, msg.length());
}

void i2cSlaveReceiveData(int dataSizeInBytes)
{

  onLed0();

  int indexPos = 0, i = 0;

  for (int i = 0; i < dataSizeInBytes; i += 1)
  {
    char c = Wire.read();
    i2cMsg += c;
  }

  i2cMsg.trim();

  if (i2cMsg != "")
  {
    do
    {
      indexPos = i2cMsg.indexOf(',');
      if (indexPos != -1)
      {
        i2cMsgBuffer = i2cMsg.substring(0, indexPos);
        i2cMsg = i2cMsg.substring(indexPos + 1, i2cMsg.length());
        i2cDataBuffer[i] = i2cMsgBuffer;
        i2cMsgBuffer = "";
      }
      else
      {
        if (i2cMsg.length() > 0)
          i2cDataBuffer[i] = i2cMsg;
      }
      i += 1;
    } while (indexPos >= 0);
  }

  if (i2cDataBuffer[0] == "/pos")
  {
    i2c_msg = sendMotorsPos3dp();
  }

  else if (i2cDataBuffer[0] == "/vel")
  {
    i2c_msg = sendMotorsVel();
  }

  else if (i2cDataBuffer[0] == "/dataA")
  {
    i2c_msg = sendMotorAData();
  }

  else if (i2cDataBuffer[0] == "/dataB")
  {
    i2c_msg = sendMotorBData();
  }

  else if (i2cDataBuffer[0] == "/pwm")
  {
    int pwm_a = i2cDataBuffer[1].toInt();
    int pwm_b = i2cDataBuffer[2].toInt();
    i2c_msg = setMotorsPwm(constrain(pwm_a, -255, 255), constrain(pwm_b, -255, 255));
  }

  else if (i2cDataBuffer[0] == "/tag")
  {
    i2c_msg = setMotorsTarget(i2cDataBuffer[1].toFloat(), i2cDataBuffer[2].toFloat());
  }

  else if (i2cDataBuffer[0] == "/maxVelA")
  {
    i2c_msg = sendMaxVelA();
  }

  else if (i2cDataBuffer[0] == "/maxVelB")
  {
    i2c_msg = sendMaxVelB();
  }

  else if (i2cDataBuffer[0] == "/timeout")
  {
    if (i2cDataBuffer[1] == "")
      i2c_msg = sendCmdTimeout();
    else
      i2c_msg = setCmdTimeout(i2cDataBuffer[1].toInt());
  }

  offLed0();
  i2cMsg = "";
  i2cMsgBuffer = "";
  i2cDataBuffer[0] = "";
  i2cDataBuffer[1] = "";
  i2cDataBuffer[2] = "";
}
//-----------------------------------------------------//

//////////////////////////////////////////////////////////

#endif
