#include "GPSSensor.h"
#include <Arduino.h> // for Serial.println

GPSSensor::GPSSensor(uint8_t rxPin, uint8_t txPin) : _serial(rxPin, txPin) {
    // コンストラクタでの初期化リストでSoftwareSerialオブジェクトを初期化
}

void GPSSensor::begin(long baudRate) {
    _serial.begin(baudRate);
    Serial.println("GPS sensor initialized");
}

void GPSSensor::update() {
    while (_serial.available() > 0) {
        char c = _serial.read();
        _gps.encode(c);
        // デバッグ出力は必要に応じて
        if (_gps.location.isUpdated()) {
            Serial.print("LAT="); Serial.println(_gps.location.lat(), 6);
            Serial.print("LON="); Serial.println(_gps.location.lng(), 6);
            Serial.print("ALT="); Serial.println(_gps.altitude.meters());
        }
    }
}

double GPSSensor::getLatitude() {
    return _gps.location.lat();
}

double GPSSensor::getLongitude() {
    return _gps.location.lng();
}

double GPSSensor::getAltitude() {
    return _gps.altitude.meters();
}

bool GPSSensor::isValidLocation() {
    return _gps.location.isValid();
}

bool GPSSensor::isLocationUpdated() {
    return _gps.location.isUpdated();
}