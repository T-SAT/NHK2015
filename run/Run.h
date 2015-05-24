#ifndef _RUN_H_INCLUDED_
#define _RUN_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define HEIGHT 4.0

#define ROVER_R     0.09 //[m]
#define WHEEL_R     0.07 //[m]

#define PI 3.1415926535898
#define A 6378137.0 /* Semi-major axis */
#define ONE_F 298.257223563 /* 1/F */
#define B (A*(1.0 - 1.0/ONE_F))
#define E2 ((1.0/ONE_F)*(2-(1.0/ONE_F)))
#define ED2 (E2*A*A/(B*B))
#define NN(p) (A/sqrt(1.0 - (E2)*pow(sin(p*PI/180.0), 2)))
#define NUM  3

typedef struct {
  double matrix3TO3[3][3];
  double vector3TO1[3];         //Z軸との型が違うのでキャストしとけ
} VECTOR;

typedef struct {
  float LAT;
  float LON;
} GEDE;

typedef struct {
  double X;
  double Y;
  double Z;
} ECEF;

typedef struct {
  double E;
  double N;
  long unsigned int U;
} ENU;

typedef struct {
  float distance;
  float angle;  //[deg]
} PolarCoordinates;

class Run{
public :
  void motorInit(int LF, int LB, int RF, int RB);

public :
  float get_lineDistance(void);
  float get_lineAngle(void);
  float get_lineGyro(double gyro);
  float get_angle(float vec1X, float vec1Y, float vec2X, float vec2Y);
  float get_angle(GEDE origin, GEDE dest);
  float get_targetValue(void);
  int crossProduct(float vec1X, float vec1Y, float vec2X, float vec2Y);

public :
  void motor_control(int motorL, int motorR);
  void motor_controlVolt(float motorL, float motorR);
  void steer(float current_value, float target_value);

public :
  float batt_voltage(void);

public :
  void update_PolarCoordinates(GEDE current, float Dseata);
  void update_targetValue(float gyro, double dt);

public :
  float distanceOFgede2gede(GEDE origin, GEDE dest);
  void improveCurrentCoordinates(GEDE current);

public:
  float last_targetValue;

public :
  GEDE ORIGIN;
  GEDE GOAL;
  GEDE LANDING;
  PolarCoordinates rover;
  PolarCoordinates goal;

private :
  int motorPinLF;
  int motorPinLB;
  int motorPinRF;
  int motorPinRB;
};

extern Run run;

#endif


