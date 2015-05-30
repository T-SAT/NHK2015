#include "Run.h"
#include <SD.h>
#include <wiring_private.h>

Run run;

unsigned long int time00;
unsigned long int time01;
unsigned long int time02;
unsigned long int time10;
unsigned long int time11;
unsigned long int time12;

void Run::motorInit(int LF, int LB, int RF, int RB)
{
  motorPinLF = LF;
  motorPinLB = LB;
  motorPinRF = RF;
  motorPinRB = RB;


  pinMode(motorPinLF, OUTPUT);
  pinMode(motorPinLB, OUTPUT);
  pinMode(motorPinRF, OUTPUT);
  pinMode(motorPinRB, OUTPUT);

  sbi(TCCR0A, COM0A1); //5ピンでTimer0_PWMをするためのレジスタ設定
  sbi(TCCR0A, COM0B1); //60ピンでTimer0_PWMをするためのレジスタ設定

}

float Run::get_lineDistance(void)
{
  float tmp;

  tmp = (rover.angle - goal.angle < 0 ? -1 : 1) * (abs(rover.distance * cos(PI - rover.angle) +
        (tan(rover.angle - goal.angle)) / (rover.distance * sin(rover.angle - PI / 2))));

  return (tmp);
}

float Run::get_lineAngle(void)
{
  return (rover.angle - goal.angle);
}

float Run::get_angle(float vec1X, float vec1Y, float vec2X, float vec2Y)
{
  float sin_v, cos_v;
  float seata;

  sin_v = (vec1X * vec2Y - vec2X * vec1Y) / (sqrt(vec1X * vec1X + vec1Y * vec1Y) * sqrt(vec2X * vec2X + vec2Y * vec2Y));
  cos_v = (vec1X * vec2X + vec1Y * vec2Y) / (sqrt(vec1X * vec1X + vec1Y * vec1Y) * sqrt(vec2X * vec2X + vec2Y * vec2Y));

  if (0 < sin_v && 0 < cos_v) {
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y) * max(asin(sin_v), -asin(sin_v));
    return (seata);
  }

  else if (0 < sin_v && cos_v < 0) {
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y) * max(acos(cos_v), -acos(cos_v));
    return (seata);
  }

  else if (sin_v < 0 && cos_v < 0) {
    seata = acos(cos_v);
    seata = seata + PI / 2;
    seata = 2 * PI - seata;
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y) * max(seata, -seata);
    return (seata);
  }

  else if (sin_v < 0 && 0 < cos_v) {
    seata = asin(sin_v);
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y) * max(seata, -seata);
    return (seata);
  }
  else return (0);
}

float Run::get_angle(GEDE origin, GEDE dest)
{
  float angle;

  angle = 90.0 - atan2(sin(dest.LON * DEG_TO_RAD - origin.LON * DEG_TO_RAD), cos(origin.LAT * DEG_TO_RAD) * tan(dest.LAT * DEG_TO_RAD) -
                       sin(origin.LAT * DEG_TO_RAD) * cos(dest.LON * DEG_TO_RAD - origin.LON * DEG_TO_RAD)) * RAD_TO_DEG;

  angle = -(180.0 <= angle ? angle - 360.0 : angle);

  return (angle);
}


float Run::get_targetValue(void)
{
  return (last_targetValue);
}

int Run::crossProduct(float vec1X, float vec1Y, float vec2X, float vec2Y)
{
  float tmp;

  tmp = vec1X * vec2Y - vec2X * vec1Y;
  if (tmp < 0) return (-1);
  else if (0 < tmp) return (1);
  else return (0);
}

void Run::motor_control(int motorL, int motorR)
{
  motorR = constrain(motorR, -254, 254);
  motorL = constrain(motorL, -254, 254);
  
  if (motorR == 0 && motorL == 0) {
    PORTD &= 0b00011111;
    PORTB &= 0b111110;
  } else {
    if (motorR < 0)
    {
      PORTD |= _BV(7);      //digitalWrite(in1Pin, HIGH);
      OCR0A = motorR + 255; //analogWrite(in2Pin, motorR+255);
    } else {
      PORTD &= ~_BV(7);      //digitalWrite(in2Pin, LOW);
      OCR0A = motorR;       //analogWrite(in2Pin, motorR);
    }

    if (motorL < 0)
    {
      PORTB &= ~_BV(0);      //digitalWrite(in3Pin, HIGH);
      OCR0B = motorL + 255; //analogWrite(in4Pin, motorL+255);
    } else {
      PORTB |= _BV(0);     //digitalWrite(in3Pin, LOW);
      OCR0B = motorL;       //analogWrite(in4Pin, motorL);
    }
  }
  
}


void Run::motor_controlVolt(float motorL, float motorR)
{
  float volt;
  int dutyL, dutyR;

  volt = batt_voltage();

  dutyL = (int)(motorL / volt) * 255;
  dutyR = (int)(motorR / volt) * 255;

  motor_control(dutyL, dutyR);
}


void Run::steer(void)
{
  /*
  static float i_err;
  float CV_D = Kp_D*(D_command - D_input);
  float err = CV_D - R_input;
  float CV_R = Kp_R * err + Ki_R * i_err;
  
  i_err += err;
  i_err = constrain(i_err, -100, 100);
  
  motor_control(T_speed + CV_R, T_speed - CV_R);
*/
}

float Run::batt_voltage(void)
{
  float batt_voltage = 0.0;

  batt_voltage = (analogRead(A0) - 4.0) / 100.0;
  return (batt_voltage);
}

float Run::getDt(void)
{
  static unsigned long int lastTime = 0.0;

  long nowTime = micros();
  float time = (double)(nowTime - lastTime);
  time = max(time, 20); //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;

  return ( time );
}

void Run::update_PolarCoordinates(GEDE current, float Dseata)
{
  rover.distance = distanceOFgede2gede(LANDING, current);
  rover.angle = get_angle(LANDING, current);
}

void Run::update_targetValue(float gyro, float dt)
{
  const float Kd = 0.01; //Pゲイン
  const float Ka = 0.05; //Iゲイン（多すぎると暴走します）
  const float Kg = 0.01; //Dゲイン

  last_targetValue = gyro + dt * (-Kd * get_lineDistance() - Ka * get_lineAngle() - Kg * get_lineGyro(gyro));
  last_targetValue = constrain(last_targetValue, -70.0, 70.0);
}

float Run::distanceOFgede2gede(GEDE origin, GEDE dest)
{
  float distance;
  float cos_v, sin_v;
  float angle;

  sin_v = pow(sin((dest.LAT * DEG_TO_RAD - origin.LAT * DEG_TO_RAD) / 2.0), 2) + cos(origin.LAT * DEG_TO_RAD) * cos(dest.LAT * DEG_TO_RAD) *
          pow(sin((dest.LON * DEG_TO_RAD - origin.LON * DEG_TO_RAD) / 2.0), 2);
  angle = asin(sqrt(sin_v)) * 2.0;
  distance = 6378137 * angle;

  return (distance);
}

void Run::improveCurrentCoordinates(GEDE current)
{
  PolarCoordinates correct;
  float distance;
  ENU tmp;

  correct.distance = distanceOFgede2gede(LANDING, current);
  correct.angle = get_angle(LANDING, current);
  distance = pow(correct.distance, 2) + pow(rover.distance, 2)
             - 2.0 * correct.distance * rover.distance * cos(rover.angle - correct.angle);

  distance = sqrt(distance);
  if (abs(distance) > 10.0) {
    rover.distance = correct.distance;
    rover.angle = correct.angle;
  }
}




















