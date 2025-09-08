#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

class GPSSensor {
public:
    GPSSensor(uint8_t rxPin, uint8_t txPin); // コンストラクタ
    void begin(long baudRate);
    void update();
    double getLatitude();
    double getLongitude();
    double getAltitude();
    bool isValidLocation();
    bool isLocationUpdated();
    // 他に必要なGPSデータアクセサーを追加

private:
    TinyGPSPlus _gps;
    SoftwareSerial _serial;
};

#endif // GPS_SENSOR_H