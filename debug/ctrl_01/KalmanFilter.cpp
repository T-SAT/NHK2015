#include "Arduino.h"
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

double KalmanFilter::kalmanFilter_9DOF(double u, double y, double dt)
{
    static double x[2] = {180, 0};
    static double P[2][2] = { {0, 0}, {0, 0}};
    static double K[2];
    const  double Q[2][2] = { {0.01, 0}, {0, 0.003}};
    const  double R = 1;
  
    x[0] += dt * (u - x[1]);

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q[1][1] * dt;
    
    K[0] = P[0][0] / (P[0][0] + R);
    K[1] = P[1][0] / (P[0][0] + R);

    x[0] += K[0] * (y - x[0]);
    x[1] += K[1] * (y - x[0]);

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
    
    return x[0];
}

double KalmanFilter::kalmanFilter_Barometer(double accel, double altitude, double dt)
{
    static double x[2] = {0.0, 0.0};
    static double P[2][2] = { {0, 0}, {0, 0}};
    static double K[2];
    const  double Q[2][2] = { {0.01, 0}, {0, 0.003}}; 
    const  double R = 1.0;
    
    x[1] = x[1] + accel*dt*9.8;
    x[0] = x[0] - dt*x[1];
    
    P[0][0] -= dt*(P[1][0] - dt*P[1][1] + P[0][1] - Q[0][0]);
    P[0][1] -= dt*P[1][1];
    P[1][0] -= dt*P[1][1];
    P[1][1] += dt*Q[1][1];

    K[0] = P[0][0] / (P[0][0] + R);
    K[1] = P[1][0] / (P[0][0] + R);
  
    x[0] += K[0]*(altitude - x[0]);
    x[1] += K[1]*(altitude - x[0]);
    
    P[0][0] -= P[0][0]*K[0];
    P[0][1] -= P[0][1]*K[0];
    P[1][0] -= K[1]*P[0][0];
    P[1][1] -= K[1]*P[0][1];
 
   return(x[0]);    
}

double KalmanFilter::kalmanFilter_Distance(double accel, double distance, double dt)
{
  static double x[2] = {0.0, 0.0};
  static double P[2][2] = { {0, 0}, {0, 0}};
  static double K[2];
  const  double Q[2][2] = { {0.01, 0}, {0, 0.003}}; 
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

KalmanFilter::KalmanFilter(double q, double r, double f, double h) {
    F = f;
    Q = q;
    H = h;
    R = r;
}

double KalmanFilter::getX0() const {
    return x0;
}

double KalmanFilter::getP0() const {
    return p0;
}

double KalmanFilter::getF() const {
    return F;
}

double KalmanFilter::getQ() const {
    return Q;
}

double KalmanFilter::getH() const {
    return H;
}

double KalmanFilter::getR() const {
    return R;
}

double KalmanFilter::getState() const {
    return state;
}

void KalmanFilter::setState(double state) {
    this->state = state;
}

double KalmanFilter::getCovariance() const {
    return covariance;
}

void KalmanFilter::setCovariance(double covariance) {
    this->covariance = covariance;
}

void KalmanFilter::correct(double data) {
    x0 = F*state;
    p0 = F*F*covariance + Q;
    
    //measurement update - correction
    double K = (H*p0)/(H*p0*H + R);
    state = x0 + K*(data - H*x0);
    covariance = (1 - K*H)*p0;
}


