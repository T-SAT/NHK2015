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
    double getDt(void);
    double kalmanFilter_9DOF(double u, double y, double dt);
    double kalmanFilter_Barometer(double accel, double altitude, double dt);
    double kalmanFilter_Distance(double accel, double distance, double dt);
    
  public:
    double getState() const;
    void setState(double state);
    void setCovariance(double covariance);
    KalmanFilter(double q = 1, double r = 1, double f = 1, double h = 1);
    void correct(double data);

  public:
    double getCovariance() const;
    double getX0() const;
    double getP0() const;
    double getF() const;
    double getQ() const;
    double getH() const;
    double getR() const;
    
  private:
    double x0; // predicted state
    double p0; // predicted covariance
    double F; // factor of real value to previous real value
    double Q; // measurement noise
    double H; // factor of measured value to real value
    double R; // environment noise
    double state;
    double covariance;
};

extern KalmanFilter Kalman;

#endif
