#ifndef _RUN_H_INCLUDED
#define _RUN_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

struct PolarCoordinate {
  float angle_deg;
  float distance_m;
};

void Control(float curren_angle_deg, float D_command_deg);
void gede2polar(float *flat_deg, float *flon_deg, float *dis_m, float *angle_deg);
float get_distanceCurrent2Goal(float *flat_deg, float *flon_deg);
void recvGPS(float *flat, float *flon, unsigned long int *age);
float get_RotationalSpeed(void);
void turn(float current_angle, float target_angle, float R_speed);
float getDt_sec(void);

#endif
