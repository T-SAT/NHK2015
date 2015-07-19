#ifndef _TEST_CTRL_0120150531_H_INCLUDED
#define _TEST_CTRL_0120150531_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SoftwareTest.h"

namespace Test_run {
  void Test_gede2polar_ErrorValueFloat(void (*target)(float *, float *, float *, float *),
                                       float *flat_deg, float *flon_deg,
                                       float targetVal_distance, float targetVal_angle);
  void Test_turn_FirstTest(void (*target)(float , float , float ), float current_angle, float target_angle, float R_speed);
  void Test_get_RotationalSpeed_FirstTest(float (*target)(void), int T_speed);

};

#endif


