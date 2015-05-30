#ifndef _KALMANFILTER_H_INCLUDED
#define _kALMANFILTER_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define CLAMP(x, low, high) (x > high)? high : (x < low)? low : x

#define LPS331_ERRORVALUE  1.6  //[m]
#define CUT_ALTITUDE       1.5  //[m]
#define MOST_ALTITUDE      10.0  //[m]

class KalmanFilter{
  public:
    double getDt(void);
    double kalmanFilter_Distance(double accel, double distance, double dt);
};

extern KalmanFilter Kalman;

#endif
