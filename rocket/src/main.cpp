#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <ESP32Ping.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <TinyGPS++.h> // https://github.com/mikalhart/TinyGPSPlus?tab=readme-ov-file https://arduiniana.org/libraries/tinygpsplus/

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

#define T1 4000
#define T2 8000

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

enum {
  Ready,
  Start,
  Parachute,
};

void logging();
void quaternionToEuler(float qr, float qi, float qj, float qk);
void setReports();
void displayInfo(); 

char data[32*8+5];
char timestanp[16] = "";
int FlightStatus = Ready;
unsigned long TimeStart;
HardwareSerial PCSerial(0);
HardwareSerial IM920Serial(2);
HardwareSerial GpsSerial(1);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
TinyGPSPlus gps;
euler_t ypr;

void setup() {
  // PCSerial
  PCSerial.begin(115200);
  PCSerial.setTimeout(10000);
  GpsSerial.begin(57600, SERIAL_8N1, 32, 33);

  // IM920sL setting
  pinMode(IM920_BUSY, INPUT);
  IM920Serial.begin(115200);

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
  switch(FlightStatus){
    case Ready:
      if(digitalRead(FLIGHTPIN) == HIGH){
        for(int i=1; i<10; i++){
          if(digitalRead(FLIGHTPIN) != HIGH) break;
        }
        // フライトピンの離脱
        DEBUG_PRINTLN("*************************************************************FlightStatus = true*************************************************************");
        FlightStatus = Start;
        TimeStart = millis();
      }
      break;
    case Start:
      if((millis()-TimeStart > T2) || (millis()-TimeStart > T1 && fabs(ypr.roll) > 90)){
        // 開放機構作動
        DEBUG_PRINTLN("*************************************************************Kaihou*************************************************************");
        FlightStatus = Parachute;
        while(FlightStatus == Parachute){
          logging();
        }
      }
      break;
  }

  logging();
}

// ROTATION/time/yaw/piitch/roll;
// GPS/time/lat/lng/altitude/speed/hdop;
// ACCELEROMETER/time/x/y/z;
void logging(){
  memset(data, 0, sizeof(data));
  sprintf(data, "%s", "TXDA ");

  while (GpsSerial.available() > 0) {
    gps.encode(GpsSerial.read());
  }

  // タイムスタンプの更新
  if (gps.time.isUpdated()) {
    sprintf(timestanp, "%d:%d:%d.%d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
  }

  if (bno08x.wasReset()) {
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
				quaternionToEuler(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
        sprintf(data, "%sROTATION/%s/%f/%f/%f;", data, timestanp, ypr.yaw, ypr.pitch, ypr.roll);
        break;
      case SH2_ACCELEROMETER:
        sprintf(data, "%sACCELEROMETER/%s/%f/%f/%f", data, timestanp, sensorValue.un.accelerometer.x, sensorValue.un.accelerometer.y, sensorValue.un.accelerometer.z);
        break;
    }
  }

  if(gps.location.isUpdated()){
    sprintf(data, "%sGPS/%s/%f/%f/%f/%f/%f;", data, timestanp, gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.mps(), gps.hdop.hdop());
  }

  // データの送信
  char *datap = data;
  String result;
  datap += 5;
  do{
    // PCSerial.println(datap-5);
    if(digitalRead(IM920_BUSY) == LOW){
      IM920Serial.println(datap-5);
      result = IM920Serial.readStringUntil('\n');
      if(result == "NG\r"){
        // DEBUG_PRINTLN("NG");
        continue;
      } else if(result == "OK\r") {
        // DEBUG_PRINTLN(datap-5);
      } else if(result == ""){
        FlightStatus = Ready;
        return;
      }
      *(datap+26) = 'T';
      *(datap+27) = 'X';
      *(datap+28) = 'D';
      *(datap+29) = 'A';
      *(datap+30) = ' ';
      datap += 31;
    }
  } while(*datap != '\0');
  // delay(10);

  // Redirect from PCSerial to IM920sL
}

void setReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR), 500) {
    DEBUG_PRINTLN("Could not enable quaternion report");
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER), 500) {
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

