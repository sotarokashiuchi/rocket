#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <ESP32Ping.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <TinyGPS++.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define DEBUG_MODE
#if defined(DEBUG_MODE)
  #define DEBUG_PRINT(str) (PCSerial.print(str))
  #define DEBUG_PRINTLN(str) (PCSerial.println(str))
#else
  #define DEBUG_PRINT(str)
  #define DEBUG_PRINTLN(str)
#endif

#define LED_BLUE 0
#define IM920_BUSY 18
#define FLIGHTPIN 23
#define BNO08X_CS 10
#define BNO08X_INT 4
#define BNO08X_RESET -1

typedef struct {
  float yaw;
  float pitch;
  float roll;
} euler_t;

void quaternionToEuler(float qr, float qi, float qj, float qk);
void setReports();
void displayInfo(); 

HardwareSerial PCSerial(0);
HardwareSerial IM920Serial(2);
HardwareSerial GpsSerial(1);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
TinyGPSPlus gps;
euler_t ypr;

void setup() {
  // PCSerial
  PCSerial.begin(19200);
  PCSerial.setTimeout(10000);
  GpsSerial.begin(57600, SERIAL_8N1, 32, 33);

  // IM920sL setting
  pinMode(IM920_BUSY, INPUT);
  IM920Serial.begin(19200);

  // PinMode setting
  pinMode(LED_BLUE, OUTPUT);
  analogSetAttenuation(ADC_0db);
  pinMode(FLIGHTPIN, INPUT_PULLUP);

  Wire.begin(21, 22); // SDAピンは21、SCLピンは22
  if (!bno08x.begin_I2C()) {
  	DEBUG_PRINTLN("BNO08xチップが見つかりませんでした");
    while (1) { delay(10); }
  }
  setReports();
  delay(100);
  DEBUG_PRINTLN("[Finished]:setting BN008x");

  digitalWrite(LED_BLUE, HIGH);
}

void loop() {
  delay(10);
  
  // Redirect from PCSerial to IM920sL
  while(PCSerial.available()){
    if(digitalRead(IM920_BUSY) == LOW){
      IM920Serial.println(PCSerial.readStringUntil('\r'));
      DEBUG_PRINTLN();
    }
  }
  while(IM920Serial.available()){
    DEBUG_PRINTLN(IM920Serial.readStringUntil('\n'));
  }
  if (bno08x.wasReset()) {
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
				quaternionToEuler(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
        DEBUG_PRINT("ヨー: "); DEBUG_PRINT(ypr.yaw);
        DEBUG_PRINT(", ピッチ: "); DEBUG_PRINT(ypr.pitch);
        DEBUG_PRINT(", ロール: "); DEBUG_PRINTLN(ypr.roll);
        break;
    }
  }

  // while (GpsSerial.available() > 0) {
  //   char c = GpsSerial.read();
  //   if (gps.encode(c)) {
  //     displayInfo();
  //   }
  // }

  if(digitalRead(FLIGHTPIN) == HIGH){
    DEBUG_PRINTLN("HIGH");
  } else {
    DEBUG_PRINTLN("LOW");
  }
}

void setReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR), 500) {
    DEBUG_PRINTLN("Could not enable quaternion report");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
  ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}

void displayInfo()
{
  PCSerial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    // 位置の精度: 通常は約2.5メートル
    PCSerial.print(gps.location.lat(), 6);
    PCSerial.print(F(","));
    PCSerial.print(gps.location.lng(), 6);
  }
  else
  {
    PCSerial.print(F("INVALID"));
  }

  PCSerial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    PCSerial.print(gps.date.month());
    PCSerial.print(F("/"));
    PCSerial.print(gps.date.day());
    PCSerial.print(F("/"));
    PCSerial.print(gps.date.year());
  }
  else
  {
    PCSerial.print(F("INVALID"));
  }

  PCSerial.print(F(" "));
  if (gps.time.isValid())
  {
    // 時間の精度: 30ナノ秒 (rms)
    if (gps.time.hour() < 10) PCSerial.print(F("0"));
    PCSerial.print(gps.time.hour());
    PCSerial.print(F(":"));
    if (gps.time.minute() < 10) PCSerial.print(F("0"));
    PCSerial.print(gps.time.minute());
    PCSerial.print(F(":"));
    if (gps.time.second() < 10) PCSerial.print(F("0"));
    PCSerial.print(gps.time.second());
    PCSerial.print(F("."));
    if (gps.time.centisecond() < 10) PCSerial.print(F("0"));
    PCSerial.print(gps.time.centisecond());
  }
  else
  {
    PCSerial.print(F("INVALID"));
  }

  PCSerial.print(F("  Speed: "));
  if(gps.speed.isValid()){
    // 速度の精度: 0.1メートル/秒
    PCSerial.print(gps.speed.mps(), 2);
    PCSerial.print(" [m/s]");
  }
  else
  {
    PCSerial.print(F("INVALID"));
  }

  PCSerial.print(F("  Altitude: "));
  if(gps.altitude.isValid()){
    // 高度の精度: 通常は約3メートル
    PCSerial.print(gps.altitude.meters(), 1);
    PCSerial.print(" [m]");
  }
  else
  {
    PCSerial.print(F("INVALID"));
  }

  PCSerial.print(F("  Hdop: "));
  if(gps.hdop.isValid()){
    PCSerial.print(gps.hdop.hdop(), 10);
  }
  else
  {
    PCSerial.print(F("INVALID"));
  }

  PCSerial.println();
}
