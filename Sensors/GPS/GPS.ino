#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "GPS_CLASS.h"

void setup()
{
  Serial.begin(9600);
  gpsLib.GPSInit();
}

void loop()
{
  float flat,flon;
  
  gpsLib.recvGPS(&flat, &flon);
  
  /*Serial.print("flat = ");  Serial.println(flat);
  Serial.print("flon = ");  Serial.println(flon);
  */
  delay(1000);
}

