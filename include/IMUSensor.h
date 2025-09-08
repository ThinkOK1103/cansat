#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> // I2C通信用

#define LIS3MDL_ADDR 0x1E

class IMUSensor {
public:
    IMUSensor();
    void begin();
    void update();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    bool isMoving();
    double getHeading(double magOffsetX, double magOffsetY, double magOffsetZ);

private:
    Adafruit_MPU6050 _mpu;
    sensors_event_t _a, _g, _temp;
    int16_t _magX, _magY, _magZ;

    void initMPU6050();
    void initLIS3MDL();
    void readLIS3MDL();
};

#endif // IMU_SENSOR_H