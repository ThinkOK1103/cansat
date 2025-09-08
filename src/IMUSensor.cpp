#include "IMUSensor.h"
#include <Arduino.h> // for Serial.println, delay
#include <math.h>    // for sqrt, fabs, atan2, cos, sin

IMUSensor::IMUSensor() : _mpu() {
    // MPU6050オブジェクトはデフォルトコンストラクタで初期化
}

void IMUSensor::begin() {
    Wire.begin(); // I2Cバスの初期化
    initMPU6050();
    initLIS3MDL();
}

void IMUSensor::initMPU6050() {
    if (!_mpu.begin()) {
        Serial.println("MPU6050が見つかりません!");
        while (1);
    }
    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050初期化完了");
    delay(100);
}

void IMUSensor::initLIS3MDL() {
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(0x0F); // WHO_AM_I レジスタ
    Wire.endTransmission();
    Wire.requestFrom(LIS3MDL_ADDR, 1);
    if (Wire.available()) {
        if (Wire.read() != 0x3D) {
            Serial.println("LIS3MDLが見つかりません!");
            while (1);
        }
    } else {
        Serial.println("LIS3MDL通信エラー!");
        while (1);
    }
    // 制御レジスタ設定
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(0x20); Wire.write(0b01111100); Wire.endTransmission(); // CTRL_REG1
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(0x21); Wire.write(0b00000000); Wire.endTransmission(); // CTRL_REG2
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(0x22); Wire.write(0x00); Wire.endTransmission();       // CTRL_REG3
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(0x23); Wire.write(0b00001100); Wire.endTransmission(); // CTRL_REG4
    Serial.println("LIS3MDL初期化完了");
    delay(100);
}

void IMUSensor::readLIS3MDL() {
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(0x28 | 0x80); // OUT_X_L レジスタから読み出し (MSBをセット)
    Wire.endTransmission();
    Wire.requestFrom(LIS3MDL_ADDR, 6); // 6バイト読み出し
    if (Wire.available() >= 6) {
        uint8_t xl = Wire.read();
        uint8_t xh = Wire.read();
        uint8_t yl = Wire.read();
        uint8_t yh = Wire.read();
        uint8_t zl = Wire.read();
        uint8_t zh = Wire.read();
        _magX = (int16_t)(xh << 8 | xl);
        _magY = (int16_t)(yh << 8 | yl);
        _magZ = (int16_t)(zh << 8 | zl);
    }
}

void IMUSensor::update() {
    _mpu.getEvent(&_a, &_g, &_temp);
    readLIS3MDL();
}

float IMUSensor::getAccelX() { return _a.acceleration.x; }
float IMUSensor::getAccelY() { return _a.acceleration.y; }
float IMUSensor::getAccelZ() { return _a.acceleration.z; }
float IMUSensor::getGyroX()  { return _g.gyro.x; }
float IMUSensor::getGyroY()  { return _g.gyro.y; }
float IMUSensor::getGyroZ()  { return _g.gyro.z; }

bool IMUSensor::isMoving() {
    float accMag = sqrt(getAccelX()*getAccelX() + getAccelY()*getAccelY() + getAccelZ()*getAccelZ());
    if (fabs(accMag - 9.81) > 0.5 || fabs(getGyroX()) > 0.1 || fabs(getGyroY()) > 0.1 || fabs(getGyroZ()) > 0.1) {
        return true;
    }
    return false;
}

double IMUSensor::getHeading(double magOffsetX, double magOffsetY, double magOffsetZ) {
    double mx = _magX - magOffsetX;
    double my = _magY - magOffsetY;
    double mz = _magZ - magOffsetZ;

    double roll  = atan2(getAccelY(), getAccelZ());
    double pitch = atan2(-getAccelX(), sqrt(getAccelY()*getAccelY() + getAccelZ()*getAccelZ()));

    double Xh = mx * cos(pitch) + mz * sin(pitch);
    double Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * cos(pitch) * sin(roll);

    double heading = atan2(Yh, Xh) * 180.0 / M_PI;
    if (heading < 0) heading += 360.0;
    return heading;
}