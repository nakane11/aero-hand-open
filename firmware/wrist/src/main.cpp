// 手首 磁気センサ (MLX90393) ファームウェア for M5Stack AtomS3
//
// USB シリアル (115200) に次の行を出力する:
//   "x,y,z"   … MLX90393 の磁場 X,Y,Z (約 50Hz)
//   "BTN,1"   … ボタンによるオン/オフ状態 (1=ON, 0=OFF)。
//               ボタンを押して状態が変わったとき, および 1 秒ごとに送る
//   "ERROR: …" … エラー
// ros1/scripts/aero_hand_magnetic_*.py がこの出力を読む
// (カンマ区切り 3 要素の行だけをセンサ値として扱うので BTN 行は他のノードでは無視される)。
// ボタン状態は画面にも表示する (ON: 緑, OFF: 灰)。
// 書き込み: firmware/wrist で `pio run -t upload` (README.md 参照)
#include <M5Unified.h>
#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

bool active = false;
unsigned long last_btn_report = 0;
const unsigned long BTN_REPORT_INTERVAL = 1000;  // ms

void drawState(const char* error = nullptr)
{
  M5.Display.fillScreen(error ? RED : (active ? GREEN : DARKGREY));
  M5.Display.setTextColor(error ? WHITE : BLACK);
  M5.Display.setTextDatum(middle_center);
  if (error) {
    M5.Display.setTextSize(1.5);
    M5.Display.drawString(error, M5.Display.width() / 2, M5.Display.height() / 2);
  } else {
    M5.Display.setTextSize(4);
    M5.Display.drawString(active ? "ON" : "OFF", M5.Display.width() / 2, M5.Display.height() / 2);
  }
}

void reportButton()
{
  Serial.print("BTN,");
  Serial.println(active ? 1 : 0);
  last_btn_report = millis();
}

void setup(void)
{
  auto cfg = M5.config();
  cfg.serial_baudrate = 0;           // Serial は下で自前で begin する
  cfg.internal_imu = false;          // 内蔵 IMU は使わない
  cfg.internal_rtc = false;
  cfg.external_display_value = 0;    // Grove ポート (I2C) を外部ディスプレイ探索に使わせない
  M5.begin(cfg);
  drawState();

  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  // Initialize I2C with AtomS3 Grove Port pins (SDA=2, SCL=1)
  Wire.begin(2, 1);

  // Initialize MLX90393 with default address 0x18 (STEMMA QT)
  if (! sensor.begin_I2C(0x18)) {
    drawState("NO SENSOR");
    while (1) {
      Serial.println("ERROR: No MLX90393 sensor found. Check your wiring!");
      delay(1000);
    }
  }

  // Optimize settings for high-speed tracking
  sensor.setGain(MLX90393_GAIN_1X);
  sensor.setResolution(MLX90393_X, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
  sensor.setOversampling(MLX90393_OSR_0);
  sensor.setFilter(MLX90393_FILTER_0);

  reportButton();
}

void loop(void) {
  M5.update();
  if (M5.BtnA.wasPressed()) {
    active = !active;
    drawState();
    reportButton();
  } else if (millis() - last_btn_report >= BTN_REPORT_INTERVAL) {
    reportButton();
  }

  float x, y, z;

  // Read X, Y and Z magnetic field data
  if (sensor.readData(&x, &y, &z)) {
      // Print X, Y, Z separated by commas for Python parser
      Serial.print(x, 4);
      Serial.print(",");
      Serial.print(y, 4);
      Serial.print(",");
      Serial.println(z, 4);
  } else {
      Serial.println("ERROR: Unable to read XYZ data");
  }

  // High frame rate (approx. 50Hz)
  delay(20);
}
