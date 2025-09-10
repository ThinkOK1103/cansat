#include "ServoController.h"
#include <math.h> // for fabs

ServoController::ServoController(uint8_t rightPin, uint8_t leftPin)
    : _lastActive(0), _lastCategory(0) {
    _servoRight.attach(rightPin);
    _servoLeft.attach(leftPin);
}

void ServoController::begin(int neutralRight, int neutralLeft,
                            double degThresholdHigh, double degThresholdLow,
                            int servoStepDelay) {
    _neutralRight = neutralRight;
    _neutralLeft = neutralLeft;
    _degThresholdHigh = degThresholdHigh;
    _degThresholdLow = degThresholdLow;
    _servoStepDelay = servoStepDelay;

    _servoRight.write(_neutralRight);
    _servoLeft.write(_neutralLeft);
    Serial.println("servoController initialized");
    delay(100);
}

void ServoController::control(double theta, double distanceToTarget) {
    int newActive = _lastActive;
    if (fabs(theta) <= _degThresholdLow) {
        newActive = 0;
    } else {
        if (theta > _degThresholdHigh) {
            newActive = 1;
        } else if (theta < -_degThresholdHigh) {
            newActive = -1;
        } else {
            newActive = _lastActive;
        }
    }

    int currentCat;
    double absTheta = fabs(theta);
    if (absTheta < 60.0) currentCat = 0;
    else if (absTheta < 120.0) currentCat = 1;
    else currentCat = 2;

    if (newActive == _lastActive && newActive != 0) {
        if (_lastCategory == 2 && currentCat == 1 && absTheta > 110.0) currentCat = 2;
        else if (_lastCategory == 1 && currentCat == 2 && absTheta < 130.0) currentCat = 1;
        if (_lastCategory == 1 && currentCat == 0 && absTheta > 50.0) currentCat = 1;
        else if (_lastCategory == 0 && currentCat == 1 && absTheta < 70.0) currentCat = 0;
    }

    bool farMode = (distanceToTarget >= 30.0);
    int baseAngle;
    if (farMode) {
        if (currentCat == 0) baseAngle = 140;
        else if (currentCat == 1) baseAngle = 90;
        else baseAngle = 30;
    } else {
        if (currentCat == 0) baseAngle = 110;
        else if (currentCat == 1) baseAngle = 75;
        else baseAngle = 15;
    }

    int targetAngleRight = _neutralRight;
    int targetAngleLeft  = _neutralLeft;

    if (newActive == 1) {
        targetAngleRight = baseAngle;
        targetAngleLeft  = _neutralLeft;
    } else if (newActive == -1) {
        int offset = _neutralRight - baseAngle;
        targetAngleLeft = _neutralLeft + offset;
        if (targetAngleLeft < 0) targetAngleLeft = 0;
        if (targetAngleLeft > 180) targetAngleLeft = 180;
        targetAngleRight = _neutralRight;
    } else {
        targetAngleRight = _neutralRight;
        targetAngleLeft  = _neutralLeft;
    }

    if (newActive != _lastActive) {
        if (_lastActive != 0 && newActive != 0) {
            if (_lastActive == 1) {
                smoothServoMove(_servoRight, _neutralRight);
            } else if (_lastActive == -1) {
                smoothServoMove(_servoLeft, _neutralLeft);
            }
        }
        if (newActive == 1) {
            smoothServoMove(_servoRight, targetAngleRight);
            smoothServoMove(_servoLeft, _neutralLeft); // 常にニュートラルを保つ
        } else if (newActive == -1) {
            smoothServoMove(_servoLeft, targetAngleLeft);
            smoothServoMove(_servoRight, _neutralRight); // 常にニュートラルを保つ
        } else {
            smoothServoMove(_servoRight, _neutralRight);
            smoothServoMove(_servoLeft, _neutralLeft);
        }
    } else {
        smoothServoMove(_servoRight, targetAngleRight);
        smoothServoMove(_servoLeft, targetAngleLeft);
    }

    _lastActive = newActive;
    _lastCategory = currentCat;
}

void ServoController::smoothServoMove(Servo& servo, int targetAngle) {
    int currentAngle = servo.read();
    if (currentAngle == targetAngle) return;

    int step = (targetAngle > currentAngle) ? 1 : -1;
    while (currentAngle != targetAngle) {
        currentAngle += step;
        servo.write(currentAngle);
        delay(_servoStepDelay);
    }
}