#include "invensense_mpu9250_spi.h"
#include "vectlab.h"

#include <Wire.h>
#include "serial_i2c_comm_api.h"

float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}

/* Mpu9250 object, SPI bus, CS on pin 10 */
bfs::Mpu9250 imu(&SPI, 10);

unsigned long serialCommTime, serialCommSampleTime = 10; // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuSampleTime = 5;        // ms -> (1000/sampleTime) hz

void setup()
{
  /* Serial to display data */
  Serial.begin(115200);
  Serial.setTimeout(4);

  initLed0();
  initLed1();

  offLed0();
  offLed1();

  //---------------- START IMU IN SPI MODE -----------------------//
  /* Start the SPI bus */
  SPI.begin();
  //----------------------------------------------------------------//

  //---------------- INITIALIZE IMU -----------------------//
  /* Initialize and configure IMU */
  if (!imu.Begin())
  {
    // Serial.println("Error initializing communication with IMU");

    while (1)
    {
    }
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19))
  {
    // Serial.println("Error configured SRD");
    while (1)
    {
    }
  }
  //----------------------------------------------------------------//

  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////

  Wire.begin(i2cAddress);
  Wire.onReceive(i2cSlaveReceiveData);
  Wire.onRequest(i2cSlaveSendData);

  madgwickFilter.setAlgorithmGain(filterGain);
  madgwickFilter.setWorldFrameId(worldFrameId); // 0 - NWU,  1 - ENU,  2 - NED

  onLed0();
  delay(500);
  offLed0();
  delay(500);
  onLed1();
  delay(500);
  offLed1();

  serialCommTime = millis();
  readImuTime = millis();
}

void loop()
{

  ////////// using the serial communiaction API ////////////////////////
  if ((millis() - serialCommTime) >= serialCommSampleTime)
  {
    serialReceiveAndSendData();
    serialCommTime = millis();
  }
  //////////////////////////////////////////////////////////////////////

  if ((millis() - readImuTime) >= readImuSampleTime)
  {

    if (imu.Read())
    {
      //------------READ SENSOR DATA IN ENU FRAME---------------//
      axRaw = imu.accel_y_mps2();
      ayRaw = imu.accel_x_mps2();
      azRaw = -1.00 * imu.accel_z_mps2();

      gxRaw = imu.gyro_y_radps();
      gyRaw = imu.gyro_x_radps();
      gzRaw = -1.00 * imu.gyro_z_radps();

      mxRaw = imu.mag_y_ut();
      myRaw = imu.mag_x_ut();
      mzRaw = -1.00 * imu.mag_z_ut();
      //--------------------------------------------------------//

      //---------------CALIBRATE SENSOR DATA IN ENU FRAME -----------------//
      // calibrate acc data
      axCal = axRaw - axOff;
      ayCal = ayRaw - ayOff;
      azCal = azRaw - azOff;

      // calibrate gyro data
      gxCal = gxRaw - gxOff;
      gyCal = gyRaw - gyOff;
      gzCal = gzRaw - gzOff;

      // calibrate mag data
      // magCal = A_1*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
      mag_vect[0] = mxRaw;
      mag_vect[1] = myRaw;
      mag_vect[2] = mzRaw;

      // mag_vect = mag_vect - b_vect
      mag_vect[0] = mag_vect[0] - b_vect[0];
      mag_vect[1] = mag_vect[1] - b_vect[1];
      mag_vect[2] = mag_vect[2] - b_vect[2];

      // mag_vect = A_mat * mag_vect
      vectOp.transform(mag_vect, A_mat, mag_vect);

      mxCal = mag_vect[0];
      myCal = mag_vect[1];
      mzCal = mag_vect[2];
      //-----------------------------------------------------//

      //------------- APPLY MADWICK FILTER -----------------//
      float _ax, _ay, _az;
      float _gx, _gy, _gz;
      float _mx, _my, _mz;
      // filter is updated based on the choosen world frame
      switch (worldFrameId)
      {
      case 0: // NWU
        _ax = ayCal;
        _ay = -1.00 * axCal;
        _az = azCal;

        _gx = gyCal;
        _gy = -1.00 * gxCal;
        _gz = gzCal;

        _mx = MicroTeslaToTesla(myCal);
        _my = MicroTeslaToTesla(-1.00 * mxCal);
        _mz = MicroTeslaToTesla(mzCal);
        break;

      case 1: // ENU
        _ax = axCal;
        _ay = ayCal;
        _az = azCal;

        _gx = gxCal;
        _gy = gyCal;
        _gz = gzCal;

        _mx = MicroTeslaToTesla(mxCal);
        _my = MicroTeslaToTesla(myCal);
        _mz = MicroTeslaToTesla(mzCal);
        break;

      case 2: // NED
        _ax = ayCal;
        _ay = axCal;
        _az = -1.00 * azCal;

        _gx = gyCal;
        _gy = gxCal;
        _gz = -1.00 * gzCal;

        _mx = MicroTeslaToTesla(myCal);
        _my = MicroTeslaToTesla(mxCal);
        _mz = MicroTeslaToTesla(-1.00 * mzCal);
        break;
      }
      

      madgwickFilter.madgwickAHRSupdate(_gx, _gy, _gz, _ax, _ay, _az, _mx, _my, _mz);

      madgwickFilter.getOrientationRPY(roll, pitch, yaw);
      madgwickFilter.getOrientationQuat(qw, qx, qy, qz);
      //----------------------------------------------------//
    }

    readImuTime = millis();
  }
}