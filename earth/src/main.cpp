#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <std_msgs/msg/int32.h>
#include <ESP32Ping.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

// #define DEBUG_MODE
#if defined(DEBUG_MODE)
  #define DEBUG_PRINT(str) (PCSerial.print(str))
  #define DEBUG_PRINTLN(str) (PCSerial.println(str))
#else
  #define DEBUG_PRINT(str)
  #define DEBUG_PRINTLN(str)
#endif

#define LED_BLUE 22
#define IM920_BUSY 18

//エラーハンドリング用のマクロ
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//micro-ROS関連でエラーが出るとマイコン上の青色LEDが点滅するように設定している
void error_loop(){
  while(1){
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

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
  analogSetAttenuation(ADC_0db);

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("[Starting]:Setting");
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
    2000
  );
  DEBUG_PRINTLN("[Finished]:WiFi Connected!!");
  DEBUG_PRINT("[Info]:Local IPaddress is ");
  DEBUG_PRINTLN(WiFi.localIP());
  delay(2000);

  // Ping
  if(Ping.ping(IPAddress(10, 42, 0, 1))){
		DEBUG_PRINTLN("[Success]:Ping to 10.42.0.1");
  } else {
		DEBUG_PRINTLN("[Error]:Ping to 10.42.0.1");
  }

  //micro-ROSの設定
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;

  //セットアップが完了するとLEDが点灯する
  digitalWrite(LED_BLUE, HIGH);
	DEBUG_PRINTLN("[Finished]:Micro-ROS Setting");
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
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
}
