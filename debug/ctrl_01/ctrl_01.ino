#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include "motor.h"
#include <Wire.h>
#include <MsTimer2.h>

#define Kp_R 1.0
#define Ki_R 1.0
#define Kp_D 0.01
#define Ki_D 0.0
#define T_speed 50

float D_command = 0.0;
long Digree = 0.0;
float CV_R = 0.0;

double gyroX;
double gyroY;
double gyroZ;

void setup() {
  Serial.begin(19200); 
  IMU.sensorInit();
  Motor::init();
}

void loop() { 
  /*
  static unsigned long u_time;
  static boolean flag;
  if(millis()-u_time > 5000)
  {
    u_time = millis();
    flag = !flag;
    if(flag){D_command = 150.0;}
    else{D_command = 0.0;}
  }
  */
  D_command = 0.0;
  //⊿tの測定 bh
  unsigned long s_time = millis();
/*
  double accYval;
  double accZval;
  double accXval; //各手法による角度の導出
*/
  //IMU.receiveAcc();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  gyroY   = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
  gyroZ   = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
/*
  accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  accZval = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  accXval = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  
  Serial.print(s_time);
  Serial.print(",");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.println(gyroZ);
*/ 
  //Serial.println(CV_R);
  
  Digree += gyroZ;
  
  float err = D_command - Digree;
  static long i_err;
  float CV_R = Kp_D *err + Ki_D *i_err;
  i_err += err;
  i_err = constrain(i_err, -100, 100);
  
  CV_R = constrain(CV_R, -200, 200);
  Serial.println(CV_R);
  
  Motor::run(T_speed -CV_R, T_speed +CV_R);
}

void Control()
{
  /*
  float CV_D = Kp_D*(D_command - dig);
  
  float err = CV_D - gyroZ;
  static float i_err;
  float CV_R = Kp_R *err + Ki_R *i_err;
  i_err += err;
  */
  float err = D_command - Digree;
  static float i_err;
  float CV_R = Kp_D *err + Ki_D *i_err;
  i_err += err;
  constrain(i_err, -100, 100);
}
