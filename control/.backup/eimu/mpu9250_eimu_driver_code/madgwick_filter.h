#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <math.h>

class MadgwickFilter
{
public:
  MadgwickFilter();

private:
  // **** paramaters
  double gain_ = 1.0;                                  // algorithm gain
  const String world_frame[3] = {"NWU", "ENU", "NED"}; // NWU, ENU, NED
  int world_frame_id;                                  // 0 - NWU, 1 - ENU, 2 - NED

  float roll;
  float pitch;
  float yaw;

  // **** state variables
  float q0, q1, q2, q3; // quaternion

  long last_time_;

public:
  void setAlgorithmGain(double gain)
  {
    /*Gain of the filter.
     Higher values lead to faster convergence but more noise.
     Lower values lead to slower convergence but smoother signal.*/
    gain_ = gain;
  }

  void setWorldFrameId(int id)
  {
    world_frame_id = id;
  }

  void computeRPY()
  {
    float qw = this->q0;
    float qx = this->q1;
    float qy = this->q2;
    float qz = this->q3;

    //------ CALC RPY from QUAT -----------------------//
    float t0 = 2.0 * (qw * qx + qy * qz);
    float t1 = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(t0, t1);

    float t2 = 2.0 * (qw * qy - qz * qx);
    if (t2 > 1.0)
    {
      t2 = 1.0;
    }
    else if (t2 < -1.0)
    {
      t2 = -1.0;
    }
    pitch = asin(t2);

    float t3 = 2.0 * (qw * qz + qx * qy);
    float t4 = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(t3, t4);
    //----------------------------------------------------//
  }

  void getOrientationQuat(float &q0, float &q1, float &q2, float &q3)
  {
    q0 = (float)this->q0;
    q1 = (float)this->q1;
    q2 = (float)this->q2;
    q3 = (float)this->q3;
  }

  void getOrientationRPY(float &roll, float &pitch, float &yaw)
  {
    roll = this->roll;
    pitch = this->pitch;
    yaw = this->yaw;
  }

  void madgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay,
                          float az, float mx, float my, float mz);
};

#endif // MADGWICK_FILTER_H
