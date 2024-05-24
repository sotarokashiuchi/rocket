#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
// UART0はUSBと共有して出力されているのか？

#define LED_BLUE 22
#define BUSY 18
String encodeAscii(String str);
String asciiElement(char c);

// HardwareSerial UART2(2);
// SoftwareSerial IM920Serial(8, 9);

void setup() {
  // UART2.begin(19200);
  // IM920Serial.begin(19200);
  
  pinMode(BUSY, OUTPUT);
  digitalWrite(BUSY, HIGH);


  // Serial
  Serial.begin(19200);

  // PinMode setting
  pinMode(LED_BLUE, OUTPUT);
  delay(500);
  // IM920Serial.println("RDID");
  delay(500);
  // IM920Serial.println(encodeAscii("RDID"));

  // WiFi setting
  // WiFi.config(
  //   IPAddress(10, 42, 0, 101),      // IPaddress
  //   IPAddress(10, 42, 0, 1),        // Gateway
  //   IPAddress(255, 255, 255, 255)   // Subnet
  // );
  // set_microros_wifi_transports(
  //   "ESP32",
  //   "Password12345678",
  //   IPAddress(10, 42, 0, 1),
  //   8888
  // );
  // Serial.println("WiFi Connected!!");

  // analogSetAttenuation(ADC_0db);
}

void loop() {
  Serial.println("RDID");
  delay(500);

  while(Serial.available()){
    Serial.println(Serial.readStringUntil('\n'));
  }


  // while(IM920Serial.available()){
  //   Serial.print("reading.");
  // }
  // while(Serial.available()){
  //   Serial.print("Send to IM920 ...");
  //   IM920Serial.println(Serial.readStringUntil('\n'));
  //   Serial.println("OK");
  //   // Serial.println(Serial.readStringUntil('\n'));
  // }
  // while(IM920Serial.available()){
  //   Serial.print("Send to PC...");
  //   Serial.println(IM920Serial.readStringUntil('\n'));
  //   Serial.print("Send...");
  // }
  // digitalWrite(LED_BLUE, HIGH);
  // delay(1000);)
  // digitalWrite(LED_BLUE, LOW);
  // delay(1000);
  // ENWR 45 4E 57 52 0A 0D
  // RDID 52 44 49 44
}


String encodeAscii(String str){
  String ascii = "";
  int j=0;
  bool _dot_flag = false;
  for(int i=0; i<str.length(); i++){
    if(str.charAt(i) == '.' || _dot_flag){
      j++;
      _dot_flag = true;
    }
    ascii += asciiElement(str.charAt(i));
    if(j > 1) break;
  }
  return ascii;  
}

String asciiElement(char c){
  String s;
  if(c == ',')  s = "2c";
  if(c == '.')  s = "2e";
  if(c == '0')  s = "30";
  if(c == '1')  s = "31";
  if(c == '2')  s = "32";
  if(c == '3')  s = "33";
  if(c == '4')  s = "34";
  if(c == '5')  s = "35";
  if(c == '6')  s = "36";
  if(c == '7')  s = "37";
  if(c == '8')  s = "38";
  if(c == '9')  s = "39";

  return s;
}