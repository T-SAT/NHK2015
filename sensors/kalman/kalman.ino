#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define GPS_RX 2
#define GPS_TX 3

TinyGPS gps;
SoftwareSerial ss(GPS_RX, GPS_TX);

void setup() 
{
  Serial.begin(9600);
  ss.begin(9600);
  IMU.sensorInit();
}

void loop() 
{
  //⊿tの測定 bh
  float gyroX, gyroY, gyroZ;
  float accYval, accZval, accXval; //各手法による角度の導出
  float flat, flon;
  unsigned long int age;
  bool newData;
  
  IMU.receiveAcc();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR, 'x') - IMU.getZero(GYR, 'x'); //オフセットぶんを差し引く
  gyroY   = IMU.get(GYR, 'y') - IMU.getZero(GYR, 'y');
  gyroZ   = IMU.get(GYR, 'z') - IMU.getZero(GYR, 'z');
  accXval = IMU.get(ACC, 'x') - IMU.getZero(ACC, 'x'); //オフセットぶんを差し引く
  accYval = IMU.get(ACC, 'y') - IMU.getZero(ACC, 'y'); //オフセットぶんを差し引く
  accZval = IMU.get(ACC, 'z') - IMU.getZero(ACC, 'z');

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
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.println();
  }

}


