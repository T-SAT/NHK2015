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


float Run::get_lineGyro(floatgyro)
{
  return (last_targetValue - gyro);
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

floatRun::getDt(void)
{
  static unsigned long int lastTime = 0.0;

  long nowTime = micros();
  floattime = (double)(nowTime - lastTime);
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

void Run::update_targetValue(float gyro, floatdt)
{
  const floatKd = 0.01; //Pゲイン
  const floatKa = 0.05; //Iゲイン（多すぎると暴走します）
  const floatKg = 0.01; //Dゲイン

  last_targetValue = gyro + dt * (-Kd * get_lineDistance() - Ka * get_lineAngle() - Kg * get_lineGyro(gyro));
  last_targetValue = constrain(last_targetValue, -70.0, 70.0);
}

/*
ECEF Run::GEDE2ECEF(GEDE cod, floatheight)
{
  ECEF ecef;

  ecef.X = (NN(cod.LAT)+height)*cos(cod.LAT*PI/180)*cos(cod.LON*PI/180);
  ecef.Y = (NN(cod.LAT)+height)*cos(cod.LAT*PI/180)*sin(cod.LON*PI/180);
  ecef.Z = (NN(cod.LAT)*(1-E2)+height)*sin(cod.LAT*PI/180);
  return ecef;
}

GEDE Run::ECEF2GEDE(ECEF ec)
{
  GEDE blh;
  int i = 0;
  floatphi, ramda, height, p;
  floatx, y, z;
  floatsita;

  x = ec.X, y = ec.Y, z = ec.Z;
  p = sqrt(x*x + y*y);
  sita = (180/PI) * atan2(z*A, p*B);
  --- 緯度
  phi = (180/PI) * atan2(z+ED2*B*(pow(sin(sita*PI/180), 3)),(p-E2*A*(pow(cos(sita*PI/180), 3))));
  --- 経度
  ramda = (180/PI) * atan2(y,x);
  /*--- 高さ
  height = (p / cos(phi*PI/180)) - NN(phi);
  blh.LAT = phi;
  blh.LON = ramda;
  return(blh);
}

ENU Run::ECEF2ENU(ECEF origin, ECEF dest)
{
  int i, j;
  GEDE blh;
  ECEF mov;
  ENU ret;
  floatrotyp[3][3], rotzp1[3][3], rotzp2[3][3];
  floatmat_conv1[3][3] = {
  };
  floatmat_conv2[3][3] = {
  };

  blh = ECEF2GEDE(origin);
  rotz(rotzp1,90.0);
  roty(rotyp, 90.0 - blh.LAT);
  rotz(rotzp2,blh.LON);
  matmat(mat_conv1, rotzp1, rotyp);
  matmat(mat_conv2, mat_conv1, rotzp2);
  mov.X = dest.X - origin.X;
  mov.Y = dest.Y - origin.Y;
  mov.Z = dest.Z - origin.Z;
  ret = matvec(mat_conv2, mov);

  return ret;
}

ENU Run::GEDE2ENU(GEDE origin, GEDE dest)
{
  GEDE aLAT, bLAT, aLON, bLON;
  ENU tmp_enu;
  float d1X, d1Y, d2X, d2Y;
  /*
  ecef_o = GEDE2ECEF(origin, hifgh);
  ecef = GEDE2ECEF(dest, high);
  tmp_enu = ECEF2ENU(ecef_o, ecef);


  return(tmp_enu);
}

void Run::setCoordinates(char *str, float flat, float flon)
{
  if(str == "origin"){
    ORIGIN.LAT = flat;
    ORIGIN.LON = flon;
  }
  else if(str == "goal"){
    GOAL.LAT = flat;
    GOAL.LON = flon;
  }
  else if(str == "landing"){
    LANDING.LAT = flat;
    LANDING.LON = flon;
  }
}

void Run::setENU(char *str, ENU enu)
{
  if(str == "origin"){
    originENU.E = enu.E;
    originENU.N = enu.N;
    originENU.U = enu.U;
  }
  else if(str == "goal"){
    goalENU.E = enu.E;
    goalENU.N = enu.N;
    goalENU.U = enu.U;
  }

}

void Run::setPolarCoordinates(char *str, float distance, float angle)
{
  if(str == "goal"){
    goal.distance = distance;
    goal.angle = angle;
  }
  else if(str == "rover"){
    rover.distance = distance;
    goal.angle = angle;
  }
}

PolarCoordinates Run::ENU2PolarCoordinates(ENU enu1, ENU enu2)
{
  float angle, distance;
  PolarCoordinates tmp;

  angle = get_angle(enu1.E, enu1.N, enu2.E, enu2.N);
  distance = sqrt(pow(enu2.E, 2) + pow(enu2.N, 2));
  tmp.distance = distance;
  tmp.angle = angle;

  return(tmp);
}

ENU Run::PolarCoordinates2ENU(PolarCoordinates polar)
{
  ENU tmp;

  tmp.E = polar.distance * cos(polar.angle*DEG_TO_RAD);
  tmp.N = polar.distance * sin(polar.angle*DEG_TO_RAD);

  return(tmp);
}

float Run::distanceOFgoal2current(GEDE current)
{
  ENU tmp;

  tmp = GEDE2ENU(current, GOAL);

  return(sqrt(pow(tmp.E, 2) + pow(tmp.N, 2)));
}
*/

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

floatRun::kalmanFilter_DistanceX(floataccel, floatdistance, floatdt)
{
  static floatx[2] = {
    0.0, 0.0
  };
  static floatP[2][2] = {
    {
      0, 0
    }
    , {
      0, 0
    }
  };
  static floatK[2];
  const  floatQ[2][2] = {
    {
      0.01, 0
    }
    , {
      0, 0.003
    }
  };
  const  floatR = 1.0;

  x[1] = x[1] + 9.8 * accel * dt;
  x[0] = x[0] + x[1] * dt;

  P[0][0] += dt * (P[1][0] + dt * (P[0][1] + dt * P[1][1]) + Q[0][0]);
  P[0][1] += dt * P[1][1];
  P[1][0] += dt * P[1][1];
  P[1][1] += dt * Q[1][1];

  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);

  x[0] += K[0] * (distance - x[0]);
  x[1] += K[1] * (distance - x[0]);

  P[0][0] -= P[0][0] * K[0];
  P[0][1] -= P[0][1] * K[0];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];

  return (x[0]);
}


floatRun::kalmanFilter_DistanceY(floataccel, floatdistance, floatdt)
{
  static floatx[2] = {
    0.0, 0.0
  };
  static floatP[2][2] = {
    {
      0, 0
    }
    , {
      0, 0
    }
  };
  static floatK[2];
  const  floatQ[2][2] = {
    {
      0.01, 0
    }
    , {
      0, 0.003
    }
  };
  const  floatR = 1.0;

  x[1] = x[1] + 9.8 * accel * dt;
  x[0] = x[0] + x[1] * dt;

  P[0][0] += dt * (P[1][0] + dt * (P[0][1] + dt * P[1][1]) + Q[0][0]);
  P[0][1] += dt * P[1][1];
  P[1][0] += dt * P[1][1];
  P[1][1] += dt * Q[1][1];

  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);

  x[0] += K[0] * (distance - x[0]);
  x[1] += K[1] * (distance - x[0]);

  P[0][0] -= P[0][0] * K[0];
  P[0][1] -= P[0][1] * K[0];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];

  return (x[0]);
}

void Run::rotx(floatrota[3][3], floatsita)
{
  rota[0][0] = 1;
  rota[0][1] = 0;
  rota[0][2] = 0;
  rota[1][0] = 0;
  rota[1][1] = cos(sita * PI / 180.0);
  rota[1][2] = sin(sita * PI / 180.0);
  rota[2][0] = 0;
  rota[2][1] = -sin(sita * PI / 180.0);
  rota[2][2] = cos(sita * PI / 180.0);
}

void Run::roty(floatrota[3][3], floatsita)
{
  rota[0][0] = cos(sita * PI / 180.0);
  rota[0][1] = 0;
  rota[0][2] = -sin(sita * PI / 180.0);
  rota[1][0] = 0;
  rota[1][1] = 1;
  rota[1][2] = 0;
  rota[2][0] = sin(sita * PI / 180.0);
  rota[2][1] = 0;
  rota[2][2] = cos(sita * PI / 180.0);
}

void Run::rotz(floatrota[3][3], floatsita)
{
  rota[0][0] = cos(sita * PI / 180.0);
  rota[0][1] = sin(sita * PI / 180.0);
  rota[0][2] = 0;
  rota[1][0] = -sin(sita * PI / 180.0);
  rota[1][1] = cos(sita * PI / 180.0);
  rota[1][2] = 0;
  rota[2][0] = 0;
  rota[2][1] = 0;
  rota[2][2] = 1;
}


void Run::matmat(floatc[NUM][NUM], floata[NUM][NUM], floatb[NUM][NUM])
{
  int i, j, k;
  floatr, s, t;

  r = 0;
  //受け取った２つの行列の掛け算を行う。
  for (i = 0; i < NUM; i++) {
    for (j = 0; j < NUM; j++) {
      for (k = 0; k < NUM; k++) {
        t = c[i][j] + (a[i][k] * b[k][j] + r);
        r = (a[i][k] * b[k][j] + r) - (t - c[i][j]);
        c[i][j] = t;
      }
      r = 0;
    }
    r = 0;
  }

}



ENU Run::matvec(floatmat[3][3], ECEF vector)
{
  ENU tmp;

  tmp.E = mat[0][0] * vector.X + mat[0][1] * vector.Y + mat[0][2] * vector.Z;
  tmp.N = mat[1][0] * vector.X + mat[1][1] * vector.Y + mat[1][2] * vector.Z;
  tmp.U = mat[2][0] * vector.X + mat[2][1] * vector.Y + mat[2][2] * vector.Z;

  return (tmp);
}





















