#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include <Arduino.h>
#include <Wire.h>
#include "eeprom_setup.h"

////////////////////////////////////////////////

void initLed0()
{
  pinMode(A0, OUTPUT);
}
void onLed0()
{
  digitalWrite(A0, HIGH);
}
void offLed0()
{
  digitalWrite(A0, LOW);
}

void initLed1()
{
  pinMode(A1, OUTPUT);
}
void onLed1()
{
  digitalWrite(A1, HIGH);
}
void offLed1()
{
  digitalWrite(A1, LOW);
}

////////////////////////////////////////////////////////

//-------------for SERIAL COMM ------------------------//

String resetEEPROM()
{
  setFIRST_TIME(0);
  // updateGlobalParamsFromEERPOM();
  return "1";
}

String sendSerialData1(float data1, byte dp)
{
  String data = String(data1, dp);
  return data;
}

String updateSerialData1(byte EEPROM_ADDRESS, float data1, float &global_data)
{
  set_EEPROM_data(EEPROM_ADDRESS, data1, global_data);
  return "1";
}

String updateSerialData1(byte EEPROM_ADDRESS, int data1, int &global_data)
{
  set_EEPROM_data(EEPROM_ADDRESS, data1, global_data);
  return "1";
}

String sendSerialData3(float data1, float data2, float data3, byte dp)
{
  String data = String(data1, dp);
  data += ",";
  data += String(data2, dp);
  data += ",";
  data += String(data3, dp);
  return data;
}

String updateSerialData3(byte EEPROM_ADDRESS_1, byte EEPROM_ADDRESS_2, byte EEPROM_ADDRESS_3,
                         float data1, float data2, float data3,
                         float &global_data1, float &global_data2, float &global_data3)
{
  set_EEPROM_data(EEPROM_ADDRESS_1, data1, global_data1);
  set_EEPROM_data(EEPROM_ADDRESS_2, data2, global_data2);
  set_EEPROM_data(EEPROM_ADDRESS_3, data3, global_data3);
  return "1";
}

String sendSerialData4(float data1, float data2, float data3, float data4, byte dp)
{
  String data = String(data1, dp);
  data += ",";
  data += String(data2, dp);
  data += ",";
  data += String(data3, dp);
  data += ",";
  data += String(data4, dp);
  return data;
}
//---------------------------------------------------//

///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[4];

void serialReceiveAndSendData()
{
  int indexPos = 0, i = 0;

  if (Serial.available() > 0)
  {
    while (Serial.available())
    {
      serMsg = Serial.readString();
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

      /////////////// FUNCTION CALLS /////////////////////
      if (serDataBuffer[0] == "/reset")
      {
        ser_msg = resetEEPROM();
      }
      else if (serDataBuffer[0] == "/gain")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData1(filterGain, 4);
        else
          ser_msg = updateSerialData1(GAIN_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(),
                                      filterGain);
      }
      else if (serDataBuffer[0] == "/i2c")
      {
        if (serDataBuffer[1] == ""){
          ser_msg = sendSerialData1(i2cAddress, 0);
        }
        else{
          int i2c_address = serDataBuffer[1].toInt();
          ser_msg = updateSerialData1(I2C_EEPROM_ADDRESS,
                                      constrain(i2c_address, 0, 127),
                                      i2cAddress);
        }
      }
      else if (serDataBuffer[0] == "/frame-id")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData1(worldFrameId, 0);
        else
          ser_msg = updateSerialData1(FRAME_ID_EEPROM_ADDRESS,
                                      serDataBuffer[1].toInt(),
                                      worldFrameId);
      }
      else if (serDataBuffer[0] == "/acc-raw")
      {
        ser_msg = sendSerialData3(axRaw, ayRaw, azRaw, 6);
      }
      else if (serDataBuffer[0] == "/acc-off")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(axOff, ayOff, azOff, 10);
        else
          ser_msg = updateSerialData3(AX_OFFSET_EEPROM_ADDRESS, AY_OFFSET_EEPROM_ADDRESS, AZ_OFFSET_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      axOff, ayOff, azOff);
      }
      else if (serDataBuffer[0] == "/acc-cal")
      {
        ser_msg = sendSerialData3(axCal, ayCal, azCal, 6);
      }

      else if (serDataBuffer[0] == "/gyro-raw")
      {
        ser_msg = sendSerialData3(gxRaw, gyRaw, gzRaw, 6);
      }
      else if (serDataBuffer[0] == "/gyro-off")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(gxOff, gyOff, gzOff, 10);
        else
          ser_msg = updateSerialData3(GX_OFFSET_EEPROM_ADDRESS, GY_OFFSET_EEPROM_ADDRESS, GZ_OFFSET_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      gxOff, gyOff, gzOff);
      }
      else if (serDataBuffer[0] == "/gyro-cal")
      {
        ser_msg = sendSerialData3(gxCal, gyCal, gzCal, 6);
      }

      else if (serDataBuffer[0] == "/mag-raw")
      {
        ser_msg = sendSerialData3(mxRaw, myRaw, mzRaw, 6);
      }
      else if (serDataBuffer[0] == "/mag-cal")
      {
        ser_msg = sendSerialData3(mxCal, myCal, mzCal, 6);
      }

      else if (serDataBuffer[0] == "/bvect")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(b_vect[0], b_vect[1], b_vect[2], 10);
        else
          ser_msg = updateSerialData3(B_VECT_X_EEPROM_ADDRESS, B_VECT_Y_EEPROM_ADDRESS, B_VECT_Z_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      b_vect[0], b_vect[1], b_vect[2]);
      }

      else if (serDataBuffer[0] == "/amatR0")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(A_mat[0][0], A_mat[0][1], A_mat[0][2], 10);
        else
          ser_msg = updateSerialData3(A_MAT_00_EEPROM_ADDRESS, A_MAT_01_EEPROM_ADDRESS, A_MAT_02_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      A_mat[0][0], A_mat[0][1], A_mat[0][2]);
      }

      else if (serDataBuffer[0] == "/amatR1")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(A_mat[1][0], A_mat[1][1], A_mat[1][2], 10);
        else
          ser_msg = updateSerialData3(A_MAT_10_EEPROM_ADDRESS, A_MAT_11_EEPROM_ADDRESS, A_MAT_12_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      A_mat[1][0], A_mat[1][1], A_mat[1][2]);
      }

      else if (serDataBuffer[0] == "/amatR2")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(A_mat[2][0], A_mat[2][1], A_mat[2][2], 10);
        else
          ser_msg = updateSerialData3(A_MAT_20_EEPROM_ADDRESS, A_MAT_21_EEPROM_ADDRESS, A_MAT_22_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      A_mat[2][0], A_mat[2][1], A_mat[2][2]);
      }

      else if (serDataBuffer[0] == "/rpy")
      {
        ser_msg = sendSerialData3(roll, pitch, yaw, 6);
      }
      else if (serDataBuffer[0] == "/quat")
      {
        ser_msg = sendSerialData4(qw, qx, qy, qz, 6);
      }

      else if (serDataBuffer[0] == "/rpy-var")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(roll_variance, pitch_variance, yaw_variance, 10);
        else
          ser_msg = updateSerialData3(R_VAR_EEPROM_ADDRESS, P_VAR_EEPROM_ADDRESS, Y_VAR_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      roll_variance, pitch_variance, yaw_variance);
      }

      else if (serDataBuffer[0] == "/gyro-var")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(gxVar, gyVar, gzVar, 10);
        else
          ser_msg = updateSerialData3(GX_VAR_EEPROM_ADDRESS, GY_VAR_EEPROM_ADDRESS, GZ_VAR_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      gxVar, gyVar, gzVar);
      }

      else if (serDataBuffer[0] == "/acc-var")
      {
        if (serDataBuffer[1] == "")
          ser_msg = sendSerialData3(axVar, ayVar, azVar, 10);
        else
          ser_msg = updateSerialData3(AX_VAR_EEPROM_ADDRESS, AY_VAR_EEPROM_ADDRESS, AZ_VAR_EEPROM_ADDRESS,
                                      serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat(),
                                      axVar, ayVar, azVar);
      }

      Serial.println(ser_msg);
      ser_msg = "";
      offLed1();

      ////////////////////////////////////////////////////
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
  serDataBuffer[3] = "";
}
//////////////////////////////////////////////////////////////////////////

//----------------- I2C COMMUNICATION -----------------//

String i2c_msg = "";

String i2cMsg = "", i2cMsgBuffer, i2cDataBuffer[4];

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

  if (i2cDataBuffer[0] == "/gain")
  {
    i2c_msg = sendSerialData1(filterGain, 3);
  }
  else if (i2cDataBuffer[0] == "/frame-id")
  {
    if (i2cDataBuffer[1] == "")
      i2c_msg = sendSerialData1(worldFrameId, 0);
    else
      i2c_msg = updateSerialData1(FRAME_ID_EEPROM_ADDRESS,
                                  i2cDataBuffer[1].toInt(),
                                  worldFrameId);
  }
  else if (i2cDataBuffer[0] == "/acc-cal")
  {
    i2c_msg = sendSerialData3(axCal, ayCal, azCal, 4);
  }
  else if (i2cDataBuffer[0] == "/gyr-cal")
  {
    i2c_msg = sendSerialData3(gxCal, gyCal, gzCal, 4);
  }
  else if (i2cDataBuffer[0] == "/rpy")
  {
    i2c_msg = sendSerialData3(roll, pitch, yaw, 4);
  }
  else if (i2cDataBuffer[0] == "/r-var")
  {
    i2c_msg = sendSerialData1(roll_variance, 10);
  }
  else if (i2cDataBuffer[0] == "/p-var")
  {
    i2c_msg = sendSerialData1(pitch_variance, 10);
  }
  else if (i2cDataBuffer[0] == "/y-var")
  {
    i2c_msg = sendSerialData1(yaw_variance, 10);
  }
  else if (i2cDataBuffer[0] == "/ax-var")
  {
    i2c_msg = sendSerialData1(axVar, 10);
  }
  else if (i2cDataBuffer[0] == "/ay-var")
  {
    i2c_msg = sendSerialData1(ayVar, 10);
  }
  else if (i2cDataBuffer[0] == "/az-var")
  {
    i2c_msg = sendSerialData1(azVar, 10);
  }
  else if (i2cDataBuffer[0] == "/gx-var")
  {
    i2c_msg = sendSerialData1(axVar, 10);
  }
  else if (i2cDataBuffer[0] == "/gy-var")
  {
    i2c_msg = sendSerialData1(ayVar, 10);
  }
  else if (i2cDataBuffer[0] == "/gz-var")
  {
    i2c_msg = sendSerialData1(azVar, 10);
  }

  i2cMsg = "";
  i2cMsgBuffer = "";
  i2cDataBuffer[0] = "";
  i2cDataBuffer[1] = "";
  i2cDataBuffer[2] = "";
  i2cDataBuffer[3] = "";

  offLed0();
}

//-----------------------------------------------------//

#endif