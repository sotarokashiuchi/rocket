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

#define T1 3290
#define T2 9740

#define LED_BLUE 0
#define IM920_BUSY 18
#define FLIGHTPIN 23
#define STBY 12
#define AIN1 27
#define AIN2 26
// 0,1,3,14,15,34,35,36,39は使用しないほうが良い
#define PWMA 25
#define PWMCH 0
#define BNO08X_CS 10
#define BNO08X_INT 4
#define BNO08X_RESET -1

enum {
  Ready,
  Start,
  Parachute,
};

void logging();
void release();
void close();
void setReports();
void displayInfo();
void flightpin();

char data[32*8+5];
char timestanp[16] = "";
int FlightStatus = Ready;
volatile int FlightPinCount;
volatile SemaphoreHandle_t timerSemaphore;
unsigned long TimeStart;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
HardwareSerial PCSerial(0);
HardwareSerial IM920Serial(2);
HardwareSerial GpsSerial(1);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
TinyGPSPlus gps;
hw_timer_t *timer = NULL;

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
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  ledcSetup(PWMCH, 7812.5, 8);
  ledcAttachPin(PWMA, PWMCH);
  ledcWrite(PWMCH, 128);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  digitalWrite(STBY, HIGH);

  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &flightpin, true);
  timerAlarmWrite(timer, 10000, true);

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
        int flightpincount;
        DEBUG_PRINT("Start:");
        DEBUG_PRINTLN(millis());
        FlightPinCount = flightpincount = 15;
        timerAlarmEnable(timer);
        while(flightpincount > 0){
          if(xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
            portENTER_CRITICAL(&timerMux);
            flightpincount = FlightPinCount;
            portEXIT_CRITICAL(&timerMux);
          }
        }
        timerAlarmDisable(timer);
        if(flightpincount == -256){
          break;
        }
        DEBUG_PRINT("Stop:");
        DEBUG_PRINTLN(millis());
        // フライトピンの離脱
        DEBUG_PRINTLN("*************************************************************** Start ***************************************************************");
        FlightStatus = Start;
        TimeStart = millis();
      }
      break;
    case Start:
      if((millis()-TimeStart > T2) || (millis()-TimeStart > T1 && fabs(ypr.roll) > 90)){
        // 開放機構作動
        DEBUG_PRINTLN("************************************************************** Release **************************************************************");
        release();
        DEBUG_PRINTLN("************************************************************* Parachute *************************************************************");
        FlightStatus = Parachute;
        while(FlightStatus == Parachute){
          logging();
        }
      }
      break;
  }
  logging();
}

// Ttime/time
// Rtime/yaw/piitch/roll 18 RO
// Gtime/lat/lng/altitude/speed 22 GPS
// Atime/x/y/z 18 ACC
void logging(){
  memset(data, 0, sizeof(data));
  sprintf(data, "%s", "TXDA ");

  while (GpsSerial.available() > 0) {
    gps.encode(GpsSerial.read());
  }

  if(gps.location.isUpdated()){
    sprintf(data, "%sG%d/%7f/%7f/%.1f/%3f", data, millis(), gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.mps());
  }

  // タイムスタンプの更新
  if (gps.time.isUpdated()) {
    sprintf(data, "%sT%d/%d:%d:%d.%d", millis(), gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
  }

  if (bno08x.wasReset()) {
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        sprintf(data, "%sR%d/%6f/%6f/%6f/%6f", data, millis(), sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
        break;
      case SH2_ACCELEROMETER:
        sprintf(data, "%sA%d/%6f/%6f/%6f", data, millis(), sensorValue.un.accelerometer.x, sensorValue.un.accelerometer.y, sensorValue.un.accelerometer.z);
        break;
    }
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
        ///// mugennru-pu
      } else if(result == "OK\r") {
        
      } else {
        FlightStatus = Ready;
        close();
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
  // while(PCSerial.available()){
  //   if(digitalRead(IM920_BUSY) == LOW){
  //     IM920Serial.println(PCSerial.readStringUntil('\r'));
  //     DEBUG_PRINTLN();
  //   }
  // }
  // while(IM920Serial.available()){
  //   DEBUG_PRINTLN(IM920Serial.readStringUntil('\n'));
  // }
  if(IM920Serial.available()){
    close();
  }
}

void release(){
  DEBUG_PRINTLN("RELEASE");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 50);
  delay(2500);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  return;
}

void close(){
  DEBUG_PRINTLN("CLOSE");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 50);
  delay(2500);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void IRAM_ATTR flightpin(){
  portENTER_CRITICAL_ISR(&timerMux);
  if(digitalRead(FLIGHTPIN) == HIGH){
    FlightPinCount--;
  } else {
    FlightPinCount = -256;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR), 500) {
    DEBUG_PRINTLN("Could not enable quaternion report");
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER), 500) {
    DEBUG_PRINTLN("Could not enable quaternion report");
  }
}

