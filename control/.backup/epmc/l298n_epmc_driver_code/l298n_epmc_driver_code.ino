#include <Wire.h>
#include "global_eeprom_variables.h"
#include "serial_i2c_comm_api.h"
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
///////// my sepcial delay function ///////////////
void delayMs(int ms)
{
  for (int i = 0; i < ms; i += 1)
  {
    delayMicroseconds(1000);
  }
}
//////////////////////////////////////////////////

// low pass filter (lpf) variables/parameters are decleared
// globally in the global_eeprom_variables.h file.

void lpfInit()
{
  lpfA.setOrder(orderA);
  lpfA.setCutOffFreq(cutOffFreqA);

  lpfB.setOrder(orderB);
  lpfB.setCutOffFreq(cutOffFreqB);
}

// encoder variables/parameters are decleared
// globally in the global_eeprom_variables.h file.

void encoderInit()
{
  encA.setPulsePerRev(encA_ppr);
  encB.setPulsePerRev(encB_ppr);

  encA.setStopFreqInUs(encA_stopFreq);
  encB.setStopFreqInUs(encB_stopFreq);

  attachInterrupt(digitalPinToInterrupt(encA.clkPin), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encB.clkPin), readEncoderB, RISING);
}

void readEncoderA()
{
  encA.freqPerTick = 1000000.00 / (float)(micros() - encA.oldFreqTime);
  encA.oldFreqTime = micros();
  encA.checkFreqTime = micros();

  if (digitalRead(encA.clkPin) == digitalRead(encA.dirPin))
  {
    encA.tickCount -= 1;
    encA.frequency = -encA.freqPerTick / (float)encA.pulsePerRev;
  }
  else
  {
    encA.tickCount += 1;
    encA.frequency = encA.freqPerTick / (float)encA.pulsePerRev;
  }
}

void readEncoderB()
{
  encB.freqPerTick = 1000000.00 / (float)(micros() - encB.oldFreqTime);
  encB.oldFreqTime = micros();
  encB.checkFreqTime = micros();

  if (digitalRead(encB.clkPin) == digitalRead(encB.dirPin))
  {
    encB.tickCount -= 1;
    encB.frequency = -encB.freqPerTick / (float)encB.pulsePerRev;
  }
  else
  {
    encB.tickCount += 1;
    encB.frequency = encB.freqPerTick / (float)encB.pulsePerRev;
  }
}
////////////////////////////////////////////////////////////////

// motor_bridge_control varaiables/parameters are decleared
// globally in the global_eeprom_variables.h file.

// PID variables/parameters are decleared
// globally in the global_eeprom_variables.h file.

void pidInit()
{
  pidMotorA.setParameters(kpA, kiA, kdA, outMin, outMax);
  pidMotorB.setParameters(kpB, kiB, kdB, outMin, outMax);
  pidMotorA.begin();
  pidMotorB.begin();
}
/////////////////////////////////////////////

////////////////////// MAIN CODE ////////////////////////////////////////

unsigned long serialCommTime, serialCommSampleTime = 10; // ms -> (1000/sampleTime) hz
// unsigned long pidTime, pidSampleTime = 5;                // ms -> (1000/sampleTime) hz
unsigned long pidStopTime, pidStopSampleTime = 250;      // ms -> (1000/sampleTime) hz

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial.setTimeout(2);
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  initLed0();
  initLed1();

  offLed0();
  offLed1();

  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////

  Wire.begin(getI2CADDRESS());
  Wire.onReceive(i2cSlaveReceiveData);
  Wire.onRequest(i2cSlaveSendData);

  onLed0();
  delay(500);
  offLed0();
  delay(500);
  onLed1();
  delay(500);
  offLed1();

  encoderInit();
  pidInit();
  lpfInit();
  /* motor needs no initialization as it used no global variable dependent on eeprom*/

  serialCommTime = millis();
  // pidTime = millis();
  pidStopTime = millis();
  cmdVelTimeout = millis();
}

void loop()
{
  // if (millis() - previousBatteryMillis >= batteryInterval) {
  //   previousBatteryMillis = millis();
    
  //   filteredBattery = (1 - ALPHA) * filteredBattery + ALPHA * readBatteryVoltage();
  //   float batPercent = (filteredBattery - 10.2) / (12.6 - 10.2) * 100.0;
  //   batteryPercent = constrain(batPercent, 0, 100);
  // }
  //update IMU data
  sixDOF.getEuler(angles);
  // go_btn = digitalRead(goPin);
  
  // Serial.print("Speed: "); Serial.print(cmd_speed);
  // Serial.print(", Direction: "); Serial.print(cmd_direction);
  // Serial.print(", mode: "); Serial.print(mode);  
  // Serial.print("\tyaw: "); Serial.print(angles[0]);
  // Serial.print(", pitch: "); Serial.print(angles[1]);
  // Serial.print(", roll: "); Serial.println(angles[2]);
  // Serial.print(", go_btn: "); Serial.print(go_btn);
  
  // Serial.print(", Bat: "); Serial.print(filteredBattery);
  // Serial.print("V,"); Serial.print(batteryPercent); Serial.println("%"); 
  // Send data to PC
  // sendIMUData(angles[0], angles[1], angles[2], batteryPercent, go_btn);
  ///// do not touch ////////
  ///// useful for velocity reading to check when rotation has stopped
  encA.resetFrequency();
  encB.resetFrequency();
  //////////////////////////

  ////////// using the serial communiaction API ///////////////
  if ((millis() - serialCommTime) >= serialCommSampleTime)
  {
    serialReceiveAndSendData();
    serialCommTime = millis();
  }
  /////////////////////////////////////////////////////////////

  ////////////// PID OPERATION ////////////////////////////////
  filteredAngVelA = lpfA.filter(encA.getAngVel());
  filteredAngVelB = lpfB.filter(encB.getAngVel());

  if (pidMode)
  {
    outputA = pidMotorA.compute(targetA, filteredAngVelA); // targetA is among the global params
    outputB = pidMotorB.compute(targetB, filteredAngVelB); // targetB is among the global params

    motorA.sendPWM((int)outputA);
    motorB.sendPWM((int)outputB);
  }
  // if ((millis() - pidTime) >= pidSampleTime) {
  //   pidTime = millis();
  // }
  //////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////
  if (abs(targetA) < 0.001 && abs(targetB) < 0.001)
  {
    if (pidMode == true)
    {
      if ((millis() - pidStopTime) >= pidStopSampleTime)
      {
        targetA = 0.00;
        targetB = 0.00;
        setPidModeFunc(0);
        pidStopTime = millis();
      }
    }
    else
    {
      pidStopTime = millis();
    }
  }
  else
  {
    if (pidMode == false)
    {
      setPidModeFunc(1);
    }
    pidStopTime = millis();
  }
  //////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////
  int cmdTimeout = (int)cmdVelTimeoutSampleTime;
  if (cmdTimeout > 0)
  {
    if ((millis() - cmdVelTimeout) >= cmdVelTimeoutSampleTime)
    {
      targetA = 0.00;
      targetB = 0.00;
      setPidModeFunc(0); // stop motor
    }
  }
  /////////////////////////////////////////////////////////////
}


// Function to send IMU & button data over Serial
void sendIMUData(float yaw, float pitch, float roll, uint8_t sys, uint8_t go) {
    uint8_t buffer[14]; // Header (1) + 3 floats (12) + 3 bytes (2)

    buffer[0] = 0xEE;  // Header for synchronization

    // Copy floats (roll, pitch, yaw) into buffer
    memcpy(&buffer[1], &yaw, sizeof(float));
    memcpy(&buffer[5], &pitch, sizeof(float));
    memcpy(&buffer[9], &roll, sizeof(float));

    // Copy button states
    buffer[13] = sys;
    buffer[14] = go;

    // Send binary data
    // Serial3.write(buffer, sizeof(buffer));
}

float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(voltagePin);
    delay(2);
  }
  float avgADC = sum / 10.0;  
  float voltage = (avgADC / 1023.0) * 5.0 * scaleFactor;  
  return voltage;
}