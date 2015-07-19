#include "test_run.h"

void Test_run::Test_gede2polar_ErrorValueFloat(void (*target)(float *, float *, float *, float *), 
                                              float *flat_deg, float *flon_deg, float targetVal_distance, float targetVal_angle)
{
  float dis_km, angle_deg;
  float dis_error, angle_error;

  
  target(flat_deg, flon_deg, &dis_km, &angle_deg);
  dis_error = abs(targetVal_distance - dis_km);
  angle_error = abs(targetVal_angle - angle_deg);
  assert(SoftTest.is_inRangeEqual(0.0, dis_error, 1.0) && SoftTest.is_inRangeEqual(0.0, angle_error, 15.0));
  SoftTest.no_error();
}

void Test_run::Test_turn_FirstTest(void (*target)(float , float , float ),
                                   float current_angle, float target_angle, float R_speed)
{
}

void Test_run::Test_get_RotationalSpeed_FirstTest(float (*target)(void), int T_speed)
{     
}


