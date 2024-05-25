#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <SoftwareSerial.h>

#define LED_BLUE 22
#define IM920_BUSY 18

HardwareSerial PCSerial(0);
HardwareSerial IM920Serial(2);

void setup() {
  // PCSerial
  PCSerial.begin(19200);
  PCSerial.setTimeout(10000);

  // IM920sL setting
  pinMode(IM920_BUSY, INPUT);
  IM920Serial.begin(19200);

  // PinMode setting
  pinMode(LED_BLUE, OUTPUT);

  return;

  // WiFi setting
  WiFi.config(
    IPAddress(10, 42, 0, 101),      // IPaddress
    IPAddress(10, 42, 0, 1),        // Gateway
    IPAddress(255, 255, 255, 255)   // Subnet
  );
  set_microros_wifi_transports(
    "ESP32",
    "Password12345678",
    IPAddress(10, 42, 0, 1),
    8888
  );
  PCSerial.println("WiFi Connected!!");

  analogSetAttenuation(ADC_0db);
}

void loop() {
  while(PCSerial.available()){
    if(digitalRead(IM920_BUSY) == LOW){
      IM920Serial.println(PCSerial.readStringUntil('\r'));
      PCSerial.println();
    }
  }
  while(IM920Serial.available()){
    PCSerial.println(IM920Serial.readStringUntil('\n'));
  }
}
