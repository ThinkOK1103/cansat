#include <Arduino.h>
#include <Wire.h> // I2Cを使うクラスのために必要 (beginはIMUSensor::beginで呼ぶ)

// 各クラスのヘッダーをインクルード
#include "GPSSensor.h"
#include "IMUSensor.h"
#include "BarometerSensor.h"
#include "ServoController.h"

// グローバル定数 (必要に応じてクラスのpublic static constにすることも検討)
const double TARGET_LAT = 35.138413;    // 目標緯度(中京大学八事キャンパスの緯度経度)
const double TARGET_LON = 136.964136;   // 目標経度

// サーボ制御パラメータ (ServoControllerのコンストラクタやbeginメソッドに渡す)
const int NEUTRAL_RIGHT = 140;
const int NEUTRAL_LEFT  = 40;
const double DEG_THRESHOLD_HIGH = 5.0;
const double DEG_THRESHOLD_LOW  = 2.0;
const int SERVO_STEP_DELAY = 10;

// 地磁気センサのオフセット（ハードアイアン補正用）
double MAG_OFFSET_X = 0.0;
double MAG_OFFSET_Y = 0.0;
double MAG_OFFSET_Z = 0.0;

// 各クラスのインスタンスを作成
GPSSensor gpsSensor(3, 4); // RX=3, TX=4
IMUSensor imuSensor;
BarometerSensor barometerSensor;
ServoController servoController(10, 9); // 右サーボD10, 左サーボD9

void setup() {
    Serial.begin(57600);
    while (!Serial) {;}

    // 各センサーとコントローラーの初期化
    gpsSensor.begin(9600);
    imuSensor.begin(); // Wire.begin()もこの中で呼ばれる
    barometerSensor.begin();
    servoController.begin(NEUTRAL_RIGHT, NEUTRAL_LEFT, DEG_THRESHOLD_HIGH, DEG_THRESHOLD_LOW, SERVO_STEP_DELAY);

    Serial.println("すべてのシステム初期化完了");
}

void loop() {
    unsigned long loopStart = millis();

    // 1. GPSデータ更新
    gpsSensor.update();

    // 2. IMUデータ更新
    imuSensor.update();

    // 3. 気圧データ更新
    barometerSensor.update();

    // 現在方位の取得 (オフセットを渡す)
    double currentHeading = imuSensor.getHeading(MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z);

    // 目標方位の計算
    double targetHeading = 0.0;
    if (gpsSensor.isValidLocation() && gpsSensor.isLocationUpdated()) {
        targetHeading = TinyGPSPlus::courseTo(gpsSensor.getLatitude(), gpsSensor.getLongitude(), TARGET_LAT, TARGET_LON);
    }

    // 偏差角度の計算
    double diff = targetHeading - currentHeading;
    while (diff > 180.0) diff -= 360.0;
    while (diff <= -180.0) diff += 360.0;
    double theta = diff;

    // 距離の計算
    double distanceToTarget = 0.0;
    if (gpsSensor.isValidLocation()) {
        distanceToTarget = TinyGPSPlus::distanceBetween(gpsSensor.getLatitude(), gpsSensor.getLongitude(), TARGET_LAT, TARGET_LON);
    }

    // サーボ制御
    servoController.control(theta, distanceToTarget);

    // デバッグ出力
    Serial.print("Heading: "); Serial.println(currentHeading);
    Serial.print("Theta: "); Serial.println(theta);
    Serial.print("Pressure: "); Serial.println(barometerSensor.getPressure_hPa());
    Serial.print("Moving: "); Serial.println(imuSensor.isMoving() ? "Yes" : "No");
    Serial.print("Distance: "); Serial.println(distanceToTarget);

    // ループ時間の調整 (必要に応じて)
    unsigned long loopEnd = millis();
    if ((loopEnd - loopStart) < 100) {
     delay(100 - (loopEnd - loopStart));
    }
}