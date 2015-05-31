#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include "motor.h"
#include <Wire.h>
#include <MsTimer2.h>

#define Kp_R 1.0
#define Ki_R 1.0
#define Kp_D 0.03
#define Ki_D 0.0
#define T_speed 100

#define GOAL_LON 1
#define GOAL_LAT 2

#define R 6378.137

#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)

#define TRIGER 13

#define DOG_DOUNT16   (byte)0
#define DOG_COUNT32   (byte)1
#define DOG_COUNT64   (byte)2
#define DOG_COUNT125  (byte)3
#define DOG_COUTN250  (byte)4
#define DOG_COUNT500  (byte)5
#define DOG_COUNT1000 (byte)6
#define DOG_COUNT2000 (byte)7
#define DOG_COUNT4000 (byte)8
#define DOG_COUNT8000 (byte)9

float D_command = 0.0;
long Digree = 0.0;
float CV_R = 0.0;

float origin_lat;
float origin_lon;

float gyroX;
float gyroY;
float gyroZ;

void setup() {
  Serial.begin(19200);
  pinMode(TRIGER, OUTPUT);
  digitalWrite(TRIGER, LOW);
  set_WDTcount(DOG_COUNT500);
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
    floataccYval;
    floataccZval;
    floataccXval; //各手法による角度の導出
  */
  //IMU.receiveAcc();
  //enable_WDT();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR, 'x') - IMU.getZero(GYR, 'x'); //オフセットぶんを差し引く
  gyroY   = IMU.get(GYR, 'y') - IMU.getZero(GYR, 'y');
  gyroZ   = IMU.get(GYR, 'z') - IMU.getZero(GYR, 'z');
  //unable_WDT();
  
  Serial.print("gyroZ = ");
  Serial.println(gyroZ);
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
  static long i_err = 0;
  float CV_R = Kp_D * err + Ki_D * i_err;
  i_err += err;
  i_err = constrain(i_err, -100, 100);
  CV_R = constrain(CV_R, -200, 200);
  Serial.println(CV_R);

  Motor::run(T_speed - CV_R, T_speed + CV_R);
}

void Control(void)
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
  float CV_R = Kp_D * err + Ki_D * i_err;
  i_err += err;
  i_err = constrain(i_err, -100, 100);
}

void gede2polar(float flat, float flon, float *distance, float *angle)
{
  *distance =  R * acos(sin(origin_lon) * sin(flon) + cos(origin_lon) * cos(flon) * cos(flat - origin_lat));
  *angle = 90.0 - RAD2DEG * atan2(sin(flat - origin_lat), cos(origin_lon) * tan(flon) - sin(origin_lon) * cos(flat - origin_lat));
}

void enable_WDT(void)
{
  WDTCSR = WDTCSR | _BV(WDCE);
  WDTCSR = WDTCSR | _BV(WDE);
  WDTCSR = WDTCSR | _BV(WDIE);
}

void unable_WDT(void)
{
  WDTCSR = WDTCSR | _BV(WDCE);
  WDTCSR = WDTCSR & !_BV(WDE);
  WDTCSR = WDTCSR & !_BV(WDIE);
}

void set_WDTcount(byte count)
{
  WDTCSR = WDTCSR | _BV(WDCE);
  WDTCSR = WDTCSR | count;
}

ISR(WDT_vect)
{
  digitalWrite(TRIGER, HIGH);
  digitalWrite(TRIGER, LOW);
}


