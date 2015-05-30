#ifndef _GPS_H_INCLUDED_
#define _GPS_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SOFTWARE_SERIAL_RX       24
#define SOFTWARE_SERIAL_TX       25

class GPS_CLASS {
  public :
    void GPSInit(void);
    void recvGPS(float *flat, float *flon);
    
  private :
    int softwareSerial_rx;
    int softwareSerial_tx;
};

extern GPS_CLASS gpsLib;

#endif
