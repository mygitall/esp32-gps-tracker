// ATGM336H GPS 测试 — 9600 baud
#include <TinyGPS++.h>

#define GPS_RX 16
#define GPS_TX 17

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ATGM336H GPS 测试 ===");
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("Serial2: 9600 baud, RX2(GPIO16) TX2(GPIO17)");
  Serial.println("等待定位...\n");
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    Serial.print(c);  // 打印原始 NMEA

    if (gps.encode(c)) {
      static unsigned long last = 0;
      if (gps.location.isValid() && millis() - last > 2000) {
        Serial.println();
        Serial.println("========================================");
        Serial.print("纬度: "); Serial.println(gps.location.lat(), 6);
        Serial.print("经度: "); Serial.println(gps.location.lng(), 6);
        Serial.print("海拔: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
        Serial.print("速度: "); Serial.print(gps.speed.kmph()); Serial.println(" km/h");
        Serial.print("卫星: "); Serial.println(gps.satellites.value());
        Serial.print("时间: "); Serial.print(gps.time.hour()+8); Serial.print(":");
        Serial.print(gps.time.minute()); Serial.print(":"); Serial.println(gps.time.second());
        Serial.println("========================================\n");
        last = millis();
      }
    }
  }
}
