# 統合コード解読
## Servo(15行目)
Servoは一言でいうとArduinoのサーボモータ制御用ライブラリのクラスの名前
<br>Servoクラスとは、#include<Servo.h>に含まれるクラスである</br>
<details>
<summary>classについて詳しく</summary>

変数や関数をまとめて管理するもの
以下は例(あくまで例)
~~~
class Servo //クラスの宣言 class class名
{  
    void set_PIN(int a){
        pin = a;
    }
    //ここからは適当な関数や変数
    bool move(double x){
        //ここでx度動かすプログラム
    }
}
int main(){//多分ここはvoid loop()
    Servo servoL,servoR;    //Servoをインスタンス化(クラスから作ったものを実体にする)

    const int PIN_L = 9;    //PIN番号を宣言
    const int PIN_R = 10;

    servoL.set_PIN(PIN_L);  //PIN番号を上のset_PIN関数に入れる
    servoL.set_PIN(PIN_R);

    servoL.move(100);       //leftを100度動かす
    return 0;
}
~~~

</details>

MadgwickとTinyGPSPlusも同じ仕様

## ACC_LSB_PER_G and GYRO_LSB_DPS(30,31行目)
- ACC_LSB_PER_G
    加速度センサの1G（重力加速度）あたりのデータ値（LSB）
    例：生データが16384なら、加速度は1G
- GYRO_LSB_DPS
    ジャイロセンサの1度/秒（dps）あたりのデータ値（LSB）
    例：生データが131なら、角速度は1dps

<details>
<summary>LSB</summary>

Least Significant Bit「最下位ビット」
デジタル変換された値の最小単位

</details>

## I2C utils(52~58行目)
- i2cWrite8
  指定したI2Cアドレスのデバイスの指定したレジスタに値を書き込む
  センサの設定、初期化時に使用
- i2cBurstRead
  指定したI2Cアドレスのデバイスの指定した開始レジスタからlenバイト分のデータをまとめて読み込みbuf配列に格納
  センサの計測データ（複数バイト）を一度に取得する時に使用

## MPU9250_init(61~69)
大雑把に説明するとノイズや、安定性を求めた調整し、初期化する

<details>
<summary>MPU9250_init()の各設定値（10進数表記）と役割</summary>

```cpp
void mpu9250_init(){
  i2cWrite8(MPU_ADDR, 107, 128); delay(100);   // Reset
  i2cWrite8(MPU_ADDR, 107, 1);                 // PLL(X-gyro)
  i2cWrite8(MPU_ADDR, 26, 3);                  // CONFIG (DLPF)
  i2cWrite8(MPU_ADDR, 27, 0);                  // GYRO ±250 dps
  i2cWrite8(MPU_ADDR, 28, 0);                  // ACC  ±2 g
  i2cWrite8(MPU_ADDR, 29, 3);                  // ACCEL_CONFIG2 (DLPF)
  i2cWrite8(MPU_ADDR, 25, 9);                  // SMPLRT_DIV→100 Hz
}
```

- `i2cWrite8(MPU_ADDR, 107, 128);`  
  MPU9250の電源管理レジスタ（PWR_MGMT_1）に128を書き込むことで、センサをリセットする。

- `i2cWrite8(MPU_ADDR, 107, 1);`  
  クロックソースをX軸ジャイロに設定することで、センサの動作が安定する。

- `i2cWrite8(MPU_ADDR, 26, 3);`  
  デジタル・ローパス・フィルタ（DLPF）を有効にし、ノイズを減らして信号を滑らかにする。値3は帯域幅44Hzである。

- `i2cWrite8(MPU_ADDR, 27, 0);`  
  ジャイロの感度を±250度/秒に設定し、精度の高い角速度データを取得する。

- `i2cWrite8(MPU_ADDR, 28, 0);`  
  加速度センサの感度を±2gに設定し、精度の高い加速度データを取得する。

- `i2cWrite8(MPU_ADDR, 29, 3);`  
  加速度センサにもローパスフィルタをかけてノイズを減らす。値3は帯域幅44Hzである。

- `i2cWrite8(MPU_ADDR, 25, 9);`  
  サンプリングレートを100Hzに設定する。1kHz/(1+9)=100Hzとなり、十分なデータ更新頻度を確保できる。

</details>

<details>
<summary>MPU9250の各設定項目の詳細</summary>

1. クロックソース（107,1）

    クロックソースとは、センサ内部の動作タイミングを決める基準信号である。MPU9250では、外部クロックやジャイロの出力をクロック源にできる。X軸ジャイロを選ぶことで、センサ全体の動作が安定し、ノイズや誤動作が減る。

2. DLPF（デジタル・ローパス・フィルタ）（26,3 と 29,3）

    DLPFは高周波ノイズ（細かい揺れや電気的な雑音）を除去し、信号を滑らかにするフィルタである。値3は帯域幅44Hz（±3dB）を意味し、これより高い周波数のノイズはカットされる。加速度・ジャイロ両方にDLPFを設定することで、ノイズ対策を強化している。

3. サンプリングレート（25,9）

    サンプリングレートとは、1秒間に何回データを取得するか（更新頻度）を示す。MPU9250の基本レートは1kHzであり、SMPLRT_DIVレジスタの値が9なら、サンプリング周期は1kHz/(1+9)=100Hzとなる。100HzはロボットやCanSatの動きには十分な速さであり、データ量も多すぎず扱いやすい値である。

4. 感度設定（27,0 と 28,0）

    ジャイロ感度±250dps（27,0）は角速度の分解能が最も高くなり、細かい回転も検出できる。加速度感度±2g（28,0）は加速度の分解能が最も高くなり、細かい動きも検出できる。

これらの設定は、高精度・安定動作・ノイズ低減・リアルタイム性を両立するためのものである。

</details>
<details>
<summary>MPU9250_read_raw()の各処理と役割</summary>

```cpp
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
```

- `uint8_t b[14];`  
  14バイト分のデータを格納するための配列を用意する。

- `i2cBurstRead(MPU_ADDR, 0x3B, b, 14);`  
  MPU9250のI2Cアドレス（0x68）の0x3B（加速度Xの上位バイト）から14バイト分のデータを一気に読み込む。  
  0x3B～0x48までのレジスタには、加速度・温度・ジャイロの生データが格納されている。

- `ax=(int16_t)((b[0]<<8)|b[1]);`  
  b[0]（加速度Xの上位バイト）とb[1]（下位バイト）を結合し、16ビットの加速度X生データにする。

- `ay=(int16_t)((b[2]<<8)|b[3]);`  
  b[2]（加速度Yの上位バイト）とb[3]（下位バイト）を結合し、加速度Y生データにする。

- `az=(int16_t)((b[4]<<8)|b[5]);`  
  b[4]（加速度Zの上位バイト）とb[5]（下位バイト）を結合し、加速度Z生データにする。

- `gx=(int16_t)((b[8]<<8)|b[9]);`  
  b[8]（ジャイロXの上位バイト）とb[9]（下位バイト）を結合し、ジャイロX生データにする。

- `gy=(int16_t)((b[10]<<8)|b[11]);`  
  b[10]（ジャイロYの上位バイト）とb[11]（下位バイト）を結合し、ジャイロY生データにする。

- `gz=(int16_t)((b[12]<<8)|b[13]);`  
  b[12]（ジャイロZの上位バイト）とb[13]（下位バイト）を結合し、ジャイロZ生データにする。

この関数は、MPU9250から加速度（X,Y,Z）とジャイロ（X,Y,Z）の生データを一度に取得し、各変数に格納する役割を持つ。

生データとは、センサが直接出力する**変換前の値（デジタルカウント値、LSB値）**のこと
例えば加速度センサなら、実際の加速度（m/s²やG）ではなく、ADC（A/D変換器）が変換した整数値（例：16384など）が生データ
</details>

## ジャイロ静止バイアス推定
ジャイロセンサは静止していてもわずかな誤差を出力する
<br>そのため、電源投入時にセンサを静止させた状態で平均を取り、オフセット値を求める</br>

### 手順
- 一秒間ループで200回情報を取る
- 各軸のジャイロ値を加算し、平均を取る
- 平均値をGYRO_LSB_PER_DPSで割り角速度に換算
- 得られた値をgyroBiasX,Y,Zに格納

以降の角速度計算ではこのオフセット値を引き算して補正する

## LIS3MDL(地磁気センサ)初期化と読み取り
- LIS3MDL センサ設定まとめ
  
  WHO_AM_I レジスタ
  - アドレス: 0x0F
  - 読み出し値: 0x3D → デバイス確認用
  - これが一致すれば LIS3MDL が正しく接続されている
  - i2cRead8(LIS3MDL_ADDR, 0x0F); // 期待値: 0x3D

  出力データレート (ODR: Output Data Rate)
  - レジスタ: 0x20
  - 設定値: 0x70
  - 意味: 80Hz でサンプリング、XY軸 Ultra High Performance
  - i2cWrite8(LIS3MDL_ADDR, 0x20, 0x70);

  ガウスレンジ設定 (磁場測定範囲)
  - レジスタ: 0x21
  - 設定値: 0x00
  - 意味: ±4 gauss (最も分解能が高い範囲)
  - i2cWrite8(LIS3MDL_ADDR, 0x21, 0x00);

  他の選択肢:
  - 0x20 → ±8 gauss
  - 0x40 → ±12 gauss
  - 0x60 → ±16 gauss

  動作モード (0x22)
  - 設定値: 0x00
  - 意味: Continuous-conversion mode (連続測定モード)
  - → 常に新しいデータを測定し続ける
  - i2cWrite8(LIS3MDL_ADDR, 0x22, 0x00);

  他の選択肢:
  - 0x01 → Single-conversion mode
  - 0x02 → Power-down mode

  Z軸の動作モード (0x23)
  - 設定値: 0x0C
  - 意味: Z軸 Ultra High Performance (高精度モード)
  - i2cWrite8(LIS3MDL_ADDR, 0x23, 0x0C);

- lis3mdl_read_xyz
  0x28から6バイト（X, Y, Z各16ビット）を一括読み出し

  地磁気の生データを取得（補正は別途必要）

  生データはオフセットやスケール誤差があるため、キャリブレーション必須

  最終的には加速度計と組み合わせて傾き補正し、「正しい北方向（ヨー角）」を求める

## LPS331(気圧センサ)初期化と読み取り
- lps331_init
  
  　初期化

- lps331_read
  
  圧力を読み取り4096で割るとhpa単位になる

  42.5度を基準に補正値を足して温度を計算している

<details>
<summary>注意点</summary>

- 圧力から高度は「国際標準大気式」を用いて変換可能
- 気象条件に依存するのであくまで目安の高度

</details>

## GPS処理
### 処理の流れ
1. gpsSerial.available()
    ソフトウェアシリアルで受信したデータを読み取る。
2. gps.encode(c)
   TinyGPS++ がNMEAフォーマットの文字列を解析し、位置情報に変換する。
3. gps.location.isUpdated()
   新しい座標データが更新されたか判定
4. 位置情報の更新
   prevLat, prevLng に一つ前の位置を保存し、currentLat, currentLng, currentAlt を最新の位置情報で更新。
5. 時刻記録
   lastGpsMs = millis(); で受信した時刻を保持し、最新かどうかを判定に使う

<details>
<summary>注意点</summary>

- この関数は「非ブロッキング」方式->loop()内で関数を呼び出すたび、チェックし新しいデータがあれば更新
- GPSは1Hz〜10Hz程度で更新されるため、100Hzループの中で「随時確認」する実装が有効。

</details>

## サーボモータ制御ロジック
