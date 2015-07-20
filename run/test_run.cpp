#include "test_run.h"

void Test_run::Test_gede2polar_ErrorValueFloat(float *flat_deg, float *flon_deg, float targetVal_distance_m, float targetVal_angle_deg)
{
  float dis_m, angle_deg;
  float error_dis_m, error_angle_deg;

  
  gede2polar(flat_deg, flon_deg, &dis_m, &angle_deg);
  error_dis_m = abs(targetVal_distance_m - dis_m);
  error_angle_deg = abs(targetVal_angle_deg - angle_deg);
  assert(SoftTest.is_inRangeEqual(0.0, error_dis_m, 1.0) && SoftTest.is_inRangeEqual(0.0, error_angle_deg, 15.0));
  SoftTest.no_error();
}

void Test_run::Test_turn_FirstTest(float current_angle, float target_angle, float R_speed)
{
  turn(current_angle, target_angle, R_speed);
 
}

void Test_run::Test_get_RotationalSpeed_FirstTest(int T_speed, float target_speed_rps)
{     
}


