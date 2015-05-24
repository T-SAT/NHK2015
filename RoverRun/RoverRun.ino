#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <MsTimer2.h>
#include <wiring_private.h>
#include "Run.h"
#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include <SoftwareSerial.h>

void setup()
{
  run.motorInit(1, 2, 3, 4);
  //MsTimer2::set(10, Control);
  //MsTimer2::start();
}

void loop()
{
  int num1;
  
  num1 = 2 * 3;
  
}

void Control()
{
/*  float CV_D = Kp_D*(run.D_command - run.D_input);
  
  float err = CV_D - run.R_input;
  static float i_err;
  float CV_R = Kp_R * err + Ki_R * i_err;
  i_err += err;
  i_err = constrain(i_err, -100, 100);
  
  run.motor_control(T_speed +CV_R, T_speed -CV_R);
*/
}










