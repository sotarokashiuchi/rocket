#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  IPAddress agent_ip(192, 168, 1, 113);
  size_t agent_port = 8888;

  char ssid[] = "WIFI_SSID";
  char psk[]= "WIFI_PSK";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}