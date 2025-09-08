#include "BarometerSensor.h"

BarometerSensor::BarometerSensor() : _pressureIndex(0), _pressureStable(false), _currentPressure_hPa(0.0) {
    for (int i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
        _pressureBuffer[i] = 0; // バッファをゼロ初期化
    }
}

void BarometerSensor::begin() {
    initLPS331AP();
}

void BarometerSensor::initLPS331AP() {
    Wire.beginTransmission(LPS331_ADDR);
    Wire.write(0x20); // CTRL_REG1 レジスタ
    Wire.write(0b10010000); // PD=ON, ODR=25Hz
    Wire.endTransmission();
    delay(100);

    // 気圧バッファ初期化のために最初の気圧値を読み込む
    Wire.beginTransmission(LPS331_ADDR);
    Wire.write(0x28 | 0x80); // PRESS_OUT_XL レジスタから読み出し (MSBをセット)
    Wire.endTransmission();
    Wire.requestFrom(LPS331_ADDR, 3); // 3バイト読み出し

    if (Wire.available() >= 3) {
        uint8_t xl = Wire.read();
        uint8_t l  = Wire.read();
        uint8_t h  = Wire.read();
        long rawP = ((long)h << 16) | ((long)l << 8) | xl;
        float initPressure = rawP / 4096.0;
        for (int i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
            _pressureBuffer[i] = initPressure;
        }
    }
    Serial.println("LPS331AP気圧センサ初期化完了");
}

void BarometerSensor::readLPS331AP() {
    Wire.beginTransmission(LPS331_ADDR);
    Wire.write(0x28 | 0x80); // PRESS_OUT_XL レジスタから読み出し
    Wire.endTransmission();
    Wire.requestFrom(LPS331_ADDR, 3);
    if (Wire.available() >= 3) {
        uint8_t xl = Wire.read();
        uint8_t l  = Wire.read();
        uint8_t h  = Wire.read();
        long rawP = ((long)h << 16) | ((long)l << 8) | xl;
        _currentPressure_hPa = rawP / 4096.0;
    }

    _pressureBuffer[_pressureIndex] = _currentPressure_hPa;
    _pressureIndex = (_pressureIndex + 1) % PRESSURE_BUFFER_SIZE;

    // 気圧の安定性をチェック
    float minP = _pressureBuffer[0], maxP = _pressureBuffer[0];
    for (int i = 1; i < PRESSURE_BUFFER_SIZE; i++) {
        if (_pressureBuffer[i] < minP) minP = _pressureBuffer[i];
        if (_pressureBuffer[i] > maxP) maxP = _pressureBuffer[i];
    }
    _pressureStable = ((maxP - minP) < 0.3);
}

void BarometerSensor::update() {
    readLPS331AP();
}

float BarometerSensor::getPressure_hPa() {
    return _currentPressure_hPa;
}

bool BarometerSensor::isPressureStable() {
    return _pressureStable;
}