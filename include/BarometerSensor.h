#ifndef BAROMETER_SENSOR_H
#define BAROMETER_SENSOR_H

#include <Wire.h> // I2C通信用
#include <Arduino.h> // for delay

#define LPS331_ADDR 0x5D
#define PRESSURE_BUFFER_SIZE 8

class BarometerSensor {
public:
    BarometerSensor();
    void begin();
    void update();
    float getPressure_hPa();
    bool isPressureStable();

private:
    float _pressureBuffer[PRESSURE_BUFFER_SIZE];
    int _pressureIndex;
    bool _pressureStable;
    float _currentPressure_hPa;

    void initLPS331AP();
    void readLPS331AP();
};

#endif // BAROMETER_SENSOR_H