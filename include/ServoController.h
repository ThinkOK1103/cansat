#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
#include <Arduino.h> // for delay

class ServoController {
public:
    ServoController(uint8_t rightPin, uint8_t leftPin);
    void begin(int neutralRight, int neutralLeft, double degThresholdHigh, double degThresholdLow, int servoStepDelay);
    void control(double theta, double distanceToTarget);

private:
    Servo _servoRight;
    Servo _servoLeft;

    int _neutralRight;
    int _neutralLeft;
    double _degThresholdHigh;
    double _degThresholdLow;
    int _servoStepDelay;

    int _lastActive;    // -1:左サーボ作動, +1:右サーボ作動, 0:ニュートラル
    int _lastCategory;  // 前回の角度カテゴリ: 0=小(0-60°),1=60-120°,2=120°以上

    void smoothServoMove(Servo& servo, int targetAngle);
};

#endif // SERVO_CONTROLLER_H