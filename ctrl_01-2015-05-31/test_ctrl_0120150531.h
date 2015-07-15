#ifndef _TEST_CTRL_0120150531_H_INCLUDED
#define _TEST_CTRL_0120150531_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <assert.h>

namespace Test {
  void Test_gede2polar_ErrorValueFloat(float *flat_deg, float *flon_deg,
                                       float *dis_km, float *angle_deg);
  void Test_turn_FirstTest(float current_angle, float target_angle, float R_speed);
};
#endif


