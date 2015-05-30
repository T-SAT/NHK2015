#ifndef SENSORSTICK_9DOF_H
#define SENSORSTICK_9DOF_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ACC 0
#define GYR 1
#define MAG 2

#define GYRO_SENSITIVITY 14.375

const double accSensityivity[3] = {32.99, 33.14, 32.27};
const double gyroSensityivity[3] = {14.375, 14.375, 14.375};
const double magSensityivity[3] = {1, 1, 1};


/* ------- I2C SLAVE ADDRESS ------- */
#define ADXL345_ADDRESS 0x53
#define ITG3200_ADDRESS 0x68
#define HMC5883L_ADDRESS 0x1E


/* ------- ADXL345_REGISTER ------- */
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39


/* ------- ITG3200_REGISTER ------- */
#define ITG3200_WHO_AM_I           0x00  // RW   SETUP: I2C address   
#define ITG3200_SMPLRT_DIV         0x15  // RW   SETUP: Sample Rate Divider       
#define ITG3200_DLPF_FS            0x16  // RW   SETUP: Digital Low Pass Filter/ Full Scale range
#define ITG3200_INT_CFG            0x17  // RW   Interrupt: Configuration
#define ITG3200_INT_STATUS         0x1A  // R	Interrupt: Status
#define ITG3200_TEMP_OUT           0x1B  // R	SENSOR: Temperature 2bytes
#define ITG3200_GYRO_XOUT          0x1D  // R	SENSOR: Gyro X 2bytes  
#define ITG3200_GYRO_YOUT          0x1F  // R	SENSOR: Gyro Y 2bytes
#define ITG3200_GYRO_ZOUT          0x21  // R	SENSOR: Gyro Z 2bytes
#define ITG3200_PWR_MGM            0x3E  // RW	Power Management


/* ------- HMC5883L_REGISTER ------- */
#define HMC5883L_CONFIGURATION_REGISTER_A 0x00
#define HMC5883L_CONFIGURATION_REGISTER_B 0x01
#define HMC5883L_MODE_REGISTER 0x02
#define HMC5883L_DATA_REGISTER_BEGIN 0x03

#define HMC5883L_CONTINUOUS_MEASUREMENT_MODE 0x00
#define HMC5883L_SINGLE_MEASUREMENT_MODE 0x01
#define HMC5883L_IDLE_MODE 0x03

#define HMC5883L_RANGE_0_88 0x00
#define HMC5883L_RANGE_1_3 0x20
#define HMC5883L_RANGE_1_9 0x40
#define HMC5883L_RANGE_2_5 0x60
#define HMC5883L_RANGE_4_0 0x80
#define HMC5883L_RANGE_4_7 0xa0
#define HMC5883L_RANGE_5_6 0xc0
#define HMC5883L_RANGE_8_1 0xe0


class SensorStick_9DoF{
  public:
    void begin();
    int accRead(int acc[]);
    int accRead(int* accX,int* accY,int* accZ);
    int gyroRead(int gyro[]);
    int gyroRead(int* gyroX,int* gyroY,int* gyroZ);
    int gyroRead(int gyro[],int* temp);
    int gyroRead(int* gyroX,int* gyroY,int* gyroZ,int* temp);
    int magRead(int mag[]);
    int magRead(int* magX,int* magY,int* magZ);
    int receiveGyro();
    int receiveAcc();
    int receiveMag();
    int receiveAll();
    int getRaw(char sensor,char axis);
    double get(char sensor,char axis);
    double getZero(char sensor,char axis);
    int setZero(char sensor,char axis,double zero[]);
    int setZero(double acc[],double gyro[],double mag[]);
    

  private:
    void twiWrite(byte address, byte registerAddress, byte val);
    void twiWrite(byte address, byte registerAddress, byte val[], byte num);
    int twiRead(byte address, byte registerAddress,byte output[], byte num);
    int _gyro[4];
    int _acc[3];
    int _mag[3];
    int _gyroZero[4];
    int _accZero[3];
    int _magZero[3];
};

extern SensorStick_9DoF IMU;

#endif
