#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>

#define LED_BLUE 22

void setup() {
  // Serial
  Serial.begin(115200);

  // PinMode setting
  pinMode(LED_BLUE, OUTPUT);

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
  Serial.println("WiFi Connected!!");

}

void loop() {
  digitalWrite(LED_BLUE, HIGH);
  delay(1000);
  digitalWrite(LED_BLUE, LOW);
  delay(1000);
}
