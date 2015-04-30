#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "GPS_CLASS.h"

SoftwareSerial GPSSerial(SOFTWARE_SERIAL_RX, SOFTWARE_SERIAL_TX);

TinyGPS gps;

GPS_CLASS gpsLib;

void GPS_CLASS::GPSInit(void)
{
  GPSSerial.begin(9600);
}

void GPS_CLASS::recvGPS(float *lat, float *lon)
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed; 

  Serial.println("check");
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  { 
    while (GPSSerial.available())
    {
      char c = GPSSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long int age;
    
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6); 
    *lat = flat;
    *lon = flon;
  }
}
