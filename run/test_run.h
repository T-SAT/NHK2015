#ifndef _TEST_CTRL_0120150531_H_INCLUDED
#define _TEST_CTRL_0120150531_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SoftwareTest.h"
#include "run.h"

namespace Test_run {
  void Test_gede2polar_ErrorValueFloat(float *flat_deg, float *flon_deg,
                                       float targetVal_distance_m, float targetVal_angle_deg);
                                       
  void Test_turn_FirstTest(float current_angle_deg, float R_speed_rps, 
                          float target_angle_deg);
                          
  void Test_get_RotationalSpeed_FirstTest(int T_speed_duty, 
                                          float target_speed_rps);

};

#endif


