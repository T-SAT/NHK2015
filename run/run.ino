#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include "motor.h"
#include "test_run.h"
#include "SoftwareTest.h"
#include <Wire.h>
#include <MsTimer2.h>
#include <math.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

////////////////////ローバの体長////////////////////////////////////////
#define  WHEEL_R     68.0 //タイヤの半径[mm]
#define  ROVER_R     112.5 //ローバの半身長[mm]
////////////////////////////////////////////////////////////////////////

/////////////////////GPS処理定数////////////////////////////////////////
#define GPS_GET_NUM  10.0
#define EARTH_R      6378.137
#define DEG2RAD      (PI/180.0)
#define RAD2DEG      (180.0/PI)
#define ORIGIN       0
#define DEST         1
/////////////////////////////////////////////////////////////////////////

///////////////////////ゴール座標と目標到達距離//////////////////////////
#define GOAL_LAT                     35.515778  //°
#define GOAL_LON                     134.171402  //°
#define GOAL_DISTANCE_RANGE_MIN      0  //[m]
#define GOAL_DISTANCE_RANGE_MAX      1.0  //[m]
/////////////////////////////////////////////////////////////////////////

//////////////////////制御定数//////////////////////////////////////////////
#define Kp_R                        1.0
#define Ki_R                        1.0
#define Kp_D                        0.70
#define Ki_D                        0.001
#define T_speed                     50
///////////////////////////////////////////////////////////////////////////

//////////////////////ウォッチドッグタイマー関連//////////////////////////////
#define TRIGER                      13
/////////////////////////////////////////////////////////////////////////////

typedef struct PolarCoordinate {
  float angle;
  float distance;
};

float flat_deg[2], flon_deg[2];
unsigned long int age_m;
PolarCoordinate current_polar, goal_polar;

float D_command = 0.0;
long Digree = 0.0;
float CV_R = 0.0;

uint8_t wdt_num;

double gyroX_deg;
double gyroY_deg;
double gyroZ_deg;

SoftwareSerial ss(2, 3);
TinyGPS gps;

void setup() {

  Serial.begin(9600);

  /*
    int i;
    float flat_ave_deg = 0.0;
    float flon_ave_deg = 0.0;

    Serial.begin(9600);
    ss.begin(9600);
    IMU.sensorInit();
    Motor::init();

    for (i = 0; i < GPS_GET_NUM; i++) {
      recvGPS(&flat_deg[ORIGIN], &flon_deg[ORIGIN], &age_m);
      flat_ave_deg += flat_deg[ORIGIN];
      flon_ave_deg += flon_deg[ORIGIN];
    }

    flat_deg[ORIGIN] = flat_ave_deg / GPS_GET_NUM;
    flon_deg[ORIGIN] = flon_ave_deg / GPS_GET_NUM;

    Serial.print("origin_lat_deg = ");  Serial.println(flat_deg[ORIGIN], 6);
    Serial.print("origin_lon_deg = ");  Serial.println(flon_deg[ORIGIN], 6);
    Motor::run(T_speed, T_speed);

    for (i = 0; i < GPS_GET_NUM; i++) {
      recvGPS(&flat_deg[DEST], &flon_deg[DEST], &age_m);
    }

    Motor::run(0, 0);
    delay(1000);

    gede2polar(flat_deg, flon_deg, &current_polar.distance, &current_polar.angle);

    flat_deg[DEST] = GOAL_LAT;
    flon_deg[DEST] = GOAL_LON;

    gede2polar(flat_deg, flon_deg, &goal_polar.distance, &goal_polar.angle);

    turn(current_polar.angle, goal_polar.angle, get_RotationalSpeed());
    Serial.print("goal_distance_m = ");  Serial.println(goal_polar.distance, 6);
    Serial.print("goal_angle_deg = ");  Serial.println(goal_polar.angle, 6);
    Serial.println("throw setup()");
  */
}

void loop() {
  float flat[2], flon[2];
  int numd = 2;
  float numf = 2.0;
  char numc = 1;

  flat_deg[ORIGIN] = 139.74477;
  flat_deg[DEST] = 139.74472;
  flon_deg[ORIGIN] = 35.6544;
  flon_deg[DEST] = 35.6539;
  Test_run::Test_gede2polar_ErrorValueFloat(gede2polar2, flat_deg, flon_deg, 55.843, 184.645379);

  /*
  int i;
  float target_angle_deg, target_distance_m;
  Serial.println("into recvGPS()");
  recvGPS(&flat_deg[DEST], &flon_deg[DEST], &age_m);
  Serial.println("throw recvGPS()");
  //Motor::run(0, 0);
  delay(1000);

  Serial.println("into gede2polar()");
  gede2polar(flat_deg, flon_deg, &current_polar.distance, &current_polar.angle);
  target_angle_deg = target_distance_m = abs(goal_polar.distance - current_polar.distance);
  Serial.println("throw gede2polar()");

  if (GOAL_DISTANCE_RANGE_MIN <= target_distance_m && target_distance_m <= GOAL_DISTANCE_RANGE_MAX) {
    Serial.println("GOAL!");
    while (1) {
      Motor::run(0, 0);
    }
  }

  D_command = target_angle_deg;
  //D_command = 0.0;
  //⊿tの測定 bh
  unsigned long s_time = millis();
  Serial.println("into receive_Gyro");
  IMU.receiveGyro();
  Serial.println("finish receive_Gyro");
  gyroX_deg = IMU.get(GYR, 'x') - IMU.getZero(GYR, 'x'); //オフセットぶんを差し引く
  gyroY_deg = IMU.get(GYR, 'y') - IMU.getZero(GYR, 'y');
  gyroZ_deg = IMU.get(GYR, 'z') - IMU.getZero(GYR, 'z');

  Digree += gyroZ_deg;

  float err = D_command - Digree;
  static long i_err;
  float CV_R = Kp_D * err + Ki_D * i_err;
  i_err += err;
  i_err = constrain(i_err, -100, 100);

  CV_R = constrain(CV_R, -200, 200);
  Serial.println(CV_R);

  Motor::run(T_speed - CV_R, T_speed + CV_R);
  */
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
  float CV_R = Kp_D * err + Ki_D * i_err;
  i_err += err;
  i_err = constrain(i_err, -100, 100);
}

/*FIXME: うまく値がでない
void gede2polar(float *flat_deg, float *flon_deg, float *dis_km, float *angle_deg)
{
  float sin4dis1, sin4dis2;
  float cos4dis1, cos4dis2, cos4dis3;
  float sin4angle1, sin4angle2;
  float cos4angle1, cos4angle2;
  float tan4angle;

  sin4dis1 = sin(flon_deg[ORIGIN] * DEG2RAD);
  sin4dis2 = sin(flon_deg[DEST] * DEG2RAD);
  cos4dis1 = cos(flon_deg[ORIGIN] * DEG2RAD);
  cos4dis2 = cos(flon_deg[DEST] * DEG2RAD);
  cos4dis3 = cos((flat_deg[DEST] - flat_deg[ORIGIN]) * DEG2RAD);

  sin4angle1 = sin((flat_deg[DEST] - flat_deg[ORIGIN]) * DEG2RAD);
  sin4angle2 = sin(flon_deg[ORIGIN] * DEG2RAD);
  cos4angle1 = cos(flon_deg[ORIGIN] * DEG2RAD);
  cos4angle2 = cos((flat_deg[DEST] - flat_deg[ORIGIN]) * DEG2RAD);
  tan4angle = tan(flon_deg[DEST] * DEG2RAD);

  *dis_km = EARTH_R * acos(sin4dis1 * sin4dis2 + cos4dis1 * cos4dis2 * cos4dis3);
  *angle_deg = RAD2DEG * atan2(sin4angle1, cos4angle1 * tan4angle - sin4angle2 * cos4angle2);
}
*/

void gede2polar(float *flat_deg, float *flon_deg, float *dis_m, float *angle_deg)
{
  float disLat_km, disLon_km;
  float sin4dis1, sin4dis2;
  float cos4dis1, cos4dis2, cos4dis3;
  float sin4angle1, sin4angle2;
  float cos4angle1, cos4angle2;
  float tan4angle;

  disLat_km = EARTH_R * sin((flat_deg[DEST] - flat_deg[ORIGIN]) * DEG2RAD);
  disLon_km = EARTH_R * sin((flon_deg[DEST] - flon_deg[ORIGIN]) * DEG2RAD);

  sin4angle1 = sin((flat_deg[DEST] - flat_deg[ORIGIN]) * DEG2RAD);
  sin4angle2 = sin(flon_deg[ORIGIN] * DEG2RAD);
  cos4angle1 = cos(flon_deg[ORIGIN] * DEG2RAD);
  cos4angle2 = cos((flat_deg[DEST] - flat_deg[ORIGIN]) * DEG2RAD);
  tan4angle = tan(flon_deg[DEST] * DEG2RAD);

  *dis_m = sqrt(pow(disLat_km, 2) + pow(disLon_km, 2)) * 1000.0;
  *angle_deg = RAD2DEG * atan2(sin4angle1, cos4angle1 * tan4angle - sin4angle2 * cos4angle2);
}

void recvGPS(float *flat, float *flon, unsigned long int *age)
{
  bool newData = false;

  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(flat, flon, age);
    Serial.print(*flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : *flat, 6);
    Serial.print(",");
    Serial.println(*flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : *flon, 6);
  }

}

float get_RotationalSpeed(void)
{
  return (T_speed / 100.0);
}

void turn(float current_angle, float target_angle, float R_speed)
{
  float turn_angle = target_angle - current_angle;

  if (0 < turn_angle) {
    Motor::run(-T_speed, T_speed);
    delay(((turn_angle * ROVER_R) / (360.0 * WHEEL_R * R_speed)) * 1000.0);
  } else if (turn_angle < 0) {
    Motor::run(T_speed, -T_speed);
    delay(((-turn_angle * ROVER_R) / (360 * WHEEL_R * R_speed)) * 1000.0);
  } else {
    Motor::run(T_speed, -T_speed);
    delay((ROVER_R / (2 * WHEEL_R * R_speed)) * 1000);
  }

  Motor::run(0, 0);
}

ISR(WDT_vect)
{
  Serial.println(wdt_num);
  digitalWrite(TRIGER, HIGH);
  digitalWrite(TRIGER, LOW);
}

