#include "test_ctrl_0120150531.h"

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {

  // transmit diagnostic informations through serial link.
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // abort program execution.

  abort();

}

void Test::Test_gede2polar_ErrorValueFloat(float *flat_deg, float *flon_deg,
    float *dis_km, float *angle_deg)
{

}

void Test::Test_turn_FirstTest(float current_angle, float target_angle, float R_speed)
{


}

