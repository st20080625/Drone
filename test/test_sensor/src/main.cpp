#include <esp_system.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

// SDAとSCLの定義（= は不要）
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BNO08x bno08x;

#define BNO08x_RESET -1

sh2_SensorValue_t sensorValue;

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if(!bno08x.begin_I2C(0x4B)){ // BNO08xのI2Cアドレスは通常0x4Aか0x4Bです（0x48は誤りの可能性があります）
    Serial.println("BNO08x not detected");
    while(1)
      delay(10);
  }
  Serial.println("BNO08x detected!");

  // 回転ベクトルレポート（オプション）
  if(!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000)){
    Serial.println("Failed to enable stabilized rotation vector");
    while(1)
      delay(10);
  }
  Serial.println("Rotation Vector Report Enabled!");

  // 角速度（ジャイロ）レポートを有効化
  if(!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)){
    Serial.println("Failed to enable gyroscope report");
    while(1)
      delay(10);
  }
  Serial.println("Gyroscope Report Enabled!");
}

void loop(){
  if (bno08x.getSensorEvent(&sensorValue)) {
    // ジャイロ（角速度）データの取得
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      Serial.print("Gyroscope (rad/s): ");
      Serial.print("X: "); Serial.print(sensorValue.un.gyroscope.x, 6);
      Serial.print(" Y: "); Serial.print(sensorValue.un.gyroscope.y, 6);
      Serial.print(" Z: "); Serial.println(sensorValue.un.gyroscope.z, 6);
    }
  }

  delay(100);
}

