//================================================================
// 統合プログラム（MPU9250/9255 + LIS3MDL + LPS331 + GPS + Servo Control）
// 100 Hz更新（Madgwick）、表示0.5 Hz（2秒ごと）、ジャイロ静止バイアス推定、|a|表示
// GPSは SoftwareSerial(4:RX,3:TX[未接続でOK]) + TinyGPS++
// サーボはGPSと姿勢データに基づいて制御
//================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <MadgwickAHRS.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

Servo servoL,servoR;
const int PIN_L = 9;
const int PIN_R = 10;
const int NEUTRAL = 140;                 // 機構の中立角
const int SERVO_MIN = 0, SERVO_MAX = 180;

// ---------- I2Cアドレス ----------
#define MPU_ADDR      0x68      // MPU9250/9255
#define LIS3MDL_ADDR  0x1E
#define LPS331_ADDR   0x5D

// ---------- フィルタ ----------
Madgwick MadgwickFilter;

// ---------- スケール ----------
const float ACC_LSB_PER_G    = 16384.0f; // ±2g
const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps

// ---------- ループ制御 ----------
const float UPDATE_HZ = 100.0f;
const uint32_t UPDATE_US = (uint32_t)(1e6f / UPDATE_HZ);
const int PRINT_DIV = 50;               //  100Hz×200 = 2秒ごと（0.5Hz表示）

// ---------- バイアス ----------
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;
float magBiasX=0, magBiasY=0, magBiasZ=0;   // 後でmin–max補正を入れる場合に使用

// ---------- GPS ----------
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3); // RX=D4(←GPS TX), TX=D3(未接続でOK)
double currentLat=0, currentLng=0, currentAlt=0; // 現在地
double prevLat=0, prevLng=0; // 飛行記録の一つ前のGPS計測結果
double targetLat=35.138413, targetLng=136.964136; 
unsigned long lastGpsMs = 0;
bool gpsUpdated = false;

// ---------------- I2C utils ----------------
static inline void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static inline void i2cBurstRead(uint8_t addr, uint8_t startReg, uint8_t* buf, uint8_t len){
  Wire.beginTransmission(addr); Wire.write(startReg); Wire.endTransmission();
  Wire.requestFrom(addr, len); for(uint8_t i=0;i<len && Wire.available();++i) buf[i]=Wire.read();
}

// --------------- MPU9250 init ---------------
void mpu9250_init(){
  i2cWrite8(MPU_ADDR, 0x6B, 0x80); delay(100);   // Reset
  i2cWrite8(MPU_ADDR, 0x6B, 0x01);               // PLL(X-gyro)
  i2cWrite8(MPU_ADDR, 0x1A, 0x03);               // CONFIG (DLPF)
  i2cWrite8(MPU_ADDR, 0x1B, 0x00);               // GYRO ±250 dps
  i2cWrite8(MPU_ADDR, 0x1C, 0x00);               // ACC  ±2 g
  i2cWrite8(MPU_ADDR, 0x1D, 0x03);               // ACCEL_CONFIG2 (DLPF)
  i2cWrite8(MPU_ADDR, 0x19, 9);                  // SMPLRT_DIV→100 Hz
}

// --------------- MPU9250 read raw ---------------
void mpu9250_read_raw(int16_t& ax,int16_t& ay,int16_t& az,
                      int16_t& gx,int16_t& gy,int16_t& gz){
  uint8_t b[14];
  i2cBurstRead(MPU_ADDR, 0x3B, b, 14);
  ax=(int16_t)((b[0]<<8)|b[1]);
  ay=(int16_t)((b[2]<<8)|b[3]);
  az=(int16_t)((b[4]<<8)|b[5]);
  gx=(int16_t)((b[8]<<8)|b[9]);
  gy=(int16_t)((b[10]<<8)|b[11]);
  gz=(int16_t)((b[12]<<8)|b[13]);
}

// --------------- ジャイロ静止バイアス ---------------
void estimateGyroBias(int samples=200){
  long sx=0,sy=0,sz=0; int16_t ax,ay,az,gx,gy,gz;
  for(int i=0;i<samples;i++){
    mpu9250_read_raw(ax,ay,az,gx,gy,gz);
    sx+=gx; sy+=gy; sz+=gz;
    delay(5);
  }
  gyroBiasX=(sx/(float)samples)/GYRO_LSB_PER_DPS;
  gyroBiasY=(sy/(float)samples)/GYRO_LSB_PER_DPS;
  gyroBiasZ=(sz/(float)samples)/GYRO_LSB_PER_DPS;
}

// --------------- LIS3MDL ---------------
bool lis3mdl_init(){
  Wire.beginTransmission(LIS3MDL_ADDR); Wire.write(0x0F); Wire.endTransmission();
  Wire.requestFrom(LIS3MDL_ADDR,1); if(!Wire.available()||Wire.read()!=0x3D) return false;
  i2cWrite8(LIS3MDL_ADDR,0x20,0x70); // UHP+ODR
  i2cWrite8(LIS3MDL_ADDR,0x21,0x00); // ±4 gauss
  i2cWrite8(LIS3MDL_ADDR,0x22,0x00); // continuous
  i2cWrite8(LIS3MDL_ADDR,0x23,0x0C); // Z UHP
  return true;
}
void lis3mdl_read_xyz(int16_t& mx,int16_t& my,int16_t& mz){
  uint8_t b[6]; i2cBurstRead(LIS3MDL_ADDR,0x28|0x80,b,6);
  mx=(int16_t)((b[1]<<8)|b[0]); my=(int16_t)((b[3]<<8)|b[2]); mz=(int16_t)((b[5]<<8)|b[4]);
}

// --------------- LPS331 ---------------
void lps331_init(){ i2cWrite8(LPS331_ADDR,0x20,0b10010000); }
void lps331_read(float& p_hPa,float& t_C){
  uint8_t p[3]; i2cBurstRead(LPS331_ADDR,0x28|0x80,p,3);
  long rp=((long)p[2]<<16)|((long)p[1]<<8)|p[0]; p_hPa= rp/4096.0f;
  uint8_t t[2]; i2cBurstRead(LPS331_ADDR,0x2B|0x80,t,2);
  int16_t rt=(int16_t)((t[1]<<8)|t[0]); t_C=42.5f + rt/480.0f;
}

// --------------- GPS（非ブロッキング読取）---------------
void pollGPS(){
  while (gpsSerial.available() > 0){
    char c = gpsSerial.read();
    if (gps.encode(c) && gps.location.isUpdated()){
      // 前回のGPSデータを保存
      if (currentLat != 0 || currentLng != 0) { // 初回起動時以外
        prevLat = currentLat;
        prevLng = currentLng;
      } else { // 初回起動時
        prevLat = gps.location.lat();
        prevLng = gps.location.lng();
      }
      currentLat = gps.location.lat();
      currentLng = gps.location.lng();
      currentAlt = gps.altitude.meters();
      lastGpsMs = millis();
      gpsUpdated = true;
    }
  }
}

// --------------- サーボモーター制御ロジック ---------------
// 指定角テーブル：距離(30m境界) × 角度帯(0–60 / 60–120 / 120–180)
int lookupAngle(float l, float thetaDeg){
  float th = fabsf(thetaDeg);
  bool far = (l >= 30.0f);
  if (th < 60.0f)   return far ? 125 : 110;
  if (th < 120.0f)  return far ?  65 :  50;
  return far ? 30 : 0;
}

// 品質/安全ゲート（今回の条件では 1.0 = 弱め無し）
// HDOP, SATS, AMAG, ALT, VZ は実際の値を使用
float calcGain(float hdop, int sats, float amag, float alt, float vz){
  float g = 1.0f;
  if (!(hdop <= 2.0f && sats >= 8)) g *= 0.7f;  // 品質悪なら弱め
  if (alt <= 20.0f)                 g *= 0.5f;  // 低高度なら弱め
  if (fabs(vz) > 3.0f)              g *= 0.5f;  // 急降下/上昇なら弱め
  if (amag > 1.20f)                 g *= 0.7f;  // 振れが大なら弱め
  if (g < 0.2f) g = 0.2f;
  return g;
}

// --------------- setup ---------------
void setup(){
  Serial.begin(115200);
  Wire.begin();
  //Wire.setClock(400000); // 不安定ならコメントアウト
  gpsSerial.begin(115200);
  Serial.println(F("Init..."));
  mpu9250_init(); Serial.println(F("-> MPU9250 OK"));
  if(!lis3mdl_init()){ Serial.println(F("-> LIS3MDL not found")); while(1); }
  Serial.println(F("-> LIS3MDL OK"));
  lps331_init(); Serial.println(F("-> LPS331 OK"));
  Serial.println(F("Calibrating gyro... (keep still ~1s)"));
  estimateGyroBias(200);
  MadgwickFilter.begin(UPDATE_HZ);

  servoL.attach(PIN_L);
  servoR.attach(PIN_R);
  // 中立へ
  servoL.write(NEUTRAL);
  servoR.write(NEUTRAL);
  delay(300);

  Serial.println(F("--- Start ---"));
}

// --------------- loop ---------------
void loop(){
  static uint32_t nextTick=micros(); static int pc=0;

  // 時間合わせ（100 Hz）
  int32_t dt=(int32_t)(nextTick - micros());
  if(dt>0) delayMicroseconds(dt);
  nextTick+=UPDATE_US;

  // ---- センサ読み取り ----
  int16_t axr,ayr,azr,gxr,gyr,gzr;
  mpu9250_read_raw(axr,ayr,azr,gxr,gyr,gzr);
  float ax=axr/ACC_LSB_PER_G, ay=ayr/ACC_LSB_PER_G, az=azr/ACC_LSB_PER_G;
  float gx=(gxr/GYRO_LSB_PER_DPS)-gyroBiasX;
  float gy=(gyr/GYRO_LSB_PER_DPS)-gyroBiasY;
  float gz=(gzr/GYRO_LSB_PER_DPS)-gyroBiasZ;

  int16_t mxr,myr,mzr; lis3mdl_read_xyz(mxr,myr,mzr);
  float mx=mxr - magBiasX, my=myr - magBiasY, mz=mzr - magBiasZ;

  MadgwickFilter.update(gx,gy,gz, ax,ay,az, mx,my,mz);

  // ---- GPSを都度ポーリング（非ブロッキング）----
  pollGPS();

  // ---- モーター制御ロジック ----
  if (gpsUpdated && currentLat != 0 && currentLng != 0) { // GPSデータが更新され、かつ有効な場合
    // 飛行移動ベクトル ∆A を計算
    // TinyGPS++の距離計算関数を使用
    double deltaLat = currentLat - prevLat;
    double deltaLng = currentLng - prevLng;
    float distMoved = gps.distanceBetween(currentLat, currentLng, prevLat, prevLng);
    float headingMoved = gps.courseTo(prevLat, prevLng, currentLat, currentLng); // 移動方向

    // 目標地点への方向ベクトル G を計算
    float distToTarget = gps.distanceBetween(currentLat, currentLng, targetLat, targetLng);
    float headingToTarget = gps.courseTo(currentLat, currentLng, targetLat, targetLng); // 目標方向

    // 目標角度 θ の計算 (headingMovedとheadingToTargetの差)
    // 角度の差分を -180 から 180 の範囲に正規化
    float thetaDeg = headingToTarget - headingMoved;
    if (thetaDeg > 180) thetaDeg -= 360;
    if (thetaDeg < -180) thetaDeg += 360;

    // 姿勢角による傾斜補正 (今回はMadgwickのyawを使用するが、より厳密な補正は別途検討)
    // yaw値が機体の進行方向と一致するように調整が必要
    float currentYaw = MadgwickFilter.getYaw(); // 現在のヨー角

    // 仮の垂直速度 (VZ) はGPSの高度変化から計算することも可能だが、今回は固定値や推定値を使用
    // LPS331の気圧変化から高度変化を推定し、そこからVZを計算することもできる
    // ここでは簡易的に0とするか、前の高度との差分から計算
    static float prevAltForVz = currentAlt;
    float vz = (currentAlt - prevAltForVz) / (UPDATE_US / 1e6f); // 簡易的な垂直速度
    prevAltForVz = currentAlt;

    // サーボモーターの回転角を決定
    const int   baseAngle = lookupAngle(distToTarget, thetaDeg);
    const float gain      = calcGain(gps.hdop.hdop(), gps.satellites.value(), sqrtf(ax*ax+ay*ay+az*az), currentAlt, vz);

    // 左と右のサーボ角度を計算 (今回は右旋回を想定)
    // thetaDegが正なら右、負なら左に曲がるとして調整
    int angleL = NEUTRAL;
    int angleR = NEUTRAL;

    // 目標角度 thetaDeg に応じてサーボを操作
    // 例えば、右に曲がりたいなら右サーボの角度を小さく（引く）、左に曲がりたいなら左サーボを小さくする
    // フローチャートでは theta>0 で右サーボを回転させているので、それに従う
    if (thetaDeg > 0) { // 右に旋回
        angleR = NEUTRAL - int(baseAngle * gain); // NEUTRALから引くことで角度が小さくなる
        angleL = NEUTRAL; // 左は中立
    } else { // 左に旋回 (この実装では右旋回のみを考慮しているため、今回は例として右側に曲がる挙動)
        // 必要に応じて左旋回のロジックを追加
        // angleL = NEUTRAL + int(baseAngle * gain); // 例: 左側に曲がる場合
        angleR = NEUTRAL; // 今回は右旋回のみを考慮するため中立
        angleL = NEUTRAL; // 今回は右旋回のみを考慮するため中立
    }
    
    // 角度が範囲外にならないようにクランプ
    angleL = constrain(angleL, SERVO_MIN, SERVO_MAX);
    angleR = constrain(angleR, SERVO_MIN, SERVO_MAX);

    servoL.write(angleL);
    servoR.write(angleR);
  }


  // ---- 表示（2秒ごと）----
  if((++pc % PRINT_DIV)==0){
    float roll=MadgwickFilter.getRoll(), pitch=MadgwickFilter.getPitch(), yaw=MadgwickFilter.getYaw();
    float amag=sqrtf(ax*ax+ay*ay+az*az);
    float pres, temp; lps331_read(pres,temp);

    Serial.print(F("【姿勢】 r: "));Serial.print(roll,1);
    Serial.print(F(" p: "));Serial.print(pitch,1);
    Serial.print(F(" yaw: "));Serial.print(yaw,1); Serial.println(F(" deg"));

    Serial.print(F("【加速度】 X: "));Serial.print(ax,2);
    Serial.print(F(" Y: "));Serial.print(ay,2);
    Serial.print(F(" Z: "));Serial.print(az,2);
    Serial.print(F("  |a|="));Serial.print(amag,3); Serial.println(F(" G"));

    Serial.print(F("【地磁気】 X: "));Serial.print(mx,0);
    Serial.print(F(" Y: "));Serial.print(my,0);
    Serial.print(F(" Z: "));Serial.println(mz,0);

    Serial.print(F("【環境】 気圧: "));Serial.print(pres,2);
    Serial.print(F(" hPa 温度: "));Serial.print(temp,1); Serial.println(F(" ℃"));

    if (gpsUpdated || (millis() - lastGpsMs) <= 2000){
      Serial.print(F("【GPS】 緯度: ")); Serial.print(currentLat, 6);
      Serial.print(F(" 経度: ")); Serial.print(currentLng, 6);
      Serial.print(F(" 高度: ")); Serial.print(currentAlt, 1); Serial.print(F(" m"));
      Serial.print(F(" HDOP: ")); Serial.print(gps.hdop.hdop(), 1);
      Serial.print(F(" 衛星数: ")); Serial.print(gps.satellites.value());
      Serial.print(F(" 目標まで: ")); Serial.print(gps.distanceBetween(currentLat, currentLng, targetLat, targetLng), 1); Serial.print(F(" m"));
      Serial.print(F(" 目標角度: ")); Serial.print(gps.courseTo(currentLat, currentLng, targetLat, targetLng), 1); Serial.println(F(" deg"));
      //gpsUpdated = false; // 表示後にリセット (次のデータまで更新しない)
    } else {
      Serial.println(F("【GPS】 受信待ち…"));
    }
    Serial.print(F("【Servo】 L: "));Serial.print(servoL.read());Serial.print(F(" R: "));Serial.println(servoR.read());
    Serial.println(F("----------------------------------------"));
  }
}