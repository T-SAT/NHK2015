#ifndef _KALMANFILTER_H_INCLUDED
#define _kALMANFILTER_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define CLAMP(x, low, high) (x > high)? high : (x < low)? low : x

#define LPS331_ERRORVALUE  1.6
#define CUT_ALTITUDE       6.0

class KalmanFilter{
  public:
    float getDt(void);
    float kalmanFilter_9DOF(float u, float y, float dt);
    float kalmanFilter_Barometer(float accel, float altitude, float dt);
    float kalmanFilter_Distance(float accel, float distance, float dt);
    
  public:
    float getState() const;
    void setState(float state);
    void setCovariance(float covariance);
    KalmanFilter(float q = 1, float r = 1, float f = 1, float h = 1);
    void correct(float data);

  public:
    float getCovariance() const;
    float getX0() const;
    float getP0() const;
    float getF() const;
    float getQ() const;
    float getH() const;
    float getR() const;
    
  private:
    float x0; // predicted state
    float p0; // predicted covariance
    float F; // factor of real value to previous real value
    float Q; // measurement noise
    float H; // factor of measured value to real value
    float R; // environment noise
    float state;
    float covariance;
};

extern KalmanFilter Kalman;

#endif
