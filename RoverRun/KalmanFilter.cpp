#include <math.h>
#include "KalmanFilter.h"

KalmanFilter Kalman;

double KalmanFilter::getDt(void)
{
  static long lastTime=0;

  long nowTime = micros();
  double time = (double)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;

  return( time );
}

double KalmanFilter::kalmanFilter_Distance(double accel, double distance, double dt)
{
  static double x[2] = {
    0.0, 0.0  };
  static double P[2][2] = { 
    {
      0, 0    }
    , {
      0, 0    }
  };
  static double K[2];
  const  double Q[2][2] = { 
    {
      0.01, 0    }
    , {
      0, 0.003    }
  }; 
  const  double R = 1.0;

  x[1] = x[1] + 9.8*accel*dt;
  x[0] = x[0] + x[1]*dt;

  P[0][0] += dt*(P[1][0] + dt*(P[0][1] + dt*P[1][1]) + Q[0][0]);
  P[0][1] += dt*P[1][1];
  P[1][0] += dt*P[1][1];
  P[1][1] += dt*Q[1][1];

  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);

  x[0] += K[0]*(distance - x[0]);
  x[1] += K[1]*(distance - x[0]);

  P[0][0] -= P[0][0]*K[0];
  P[0][1] -= P[0][1]*K[0];
  P[1][0] -= K[1]*P[0][0];
  P[1][1] -= K[1]*P[0][1];

  return(x[0]);
}



