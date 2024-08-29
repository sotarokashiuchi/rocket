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
#include <custom_message/msg/rotation.h>
#include <custom_message/msg/accelerometer.h>
#include <custom_message/msg/gps.h>
#include <custom_message/msg/time.h>
#include <Wire.h>

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

//エラーハンドリング用のマクロ
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void pareDouble(double *x);
void pareInt(int *ret);

String buf;
String line;

//micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t rotation_publisher;
rcl_publisher_t accelerometer_publisher;
rcl_publisher_t gps_publisher;
rcl_publisher_t time_publisher;
custom_message__msg__Rotation rotation_msg;
custom_message__msg__Accelerometer accelerometer_msg;
custom_message__msg__Gps gps_msg;
custom_message__msg__Time time_msg;
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

void position_publish();

HardwareSerial PCSerial(0);
HardwareSerial IM920Serial(2);

void setup() {
  // PCSerial
  PCSerial.begin(115200);
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

  /*
   * micro-ROS
   */
  // micro-ROS用のメモリを確保
  allocator = rcl_get_default_allocator();

  // rclcの初期化
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // nodeの作成
  RCCHECK(rclc_node_init_default(&node, "ESP_node", "", &support));

  // publisherの作成
  rclc_publisher_init_default(
    &accelerometer_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Accelerometer),
    "accelerometer_publisher");

	rclc_publisher_init_default(
    &rotation_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Rotation),
    "rotation_publisher");

  rclc_publisher_init_default(
    &gps_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Gps),
    "gps_publisher");

  rclc_publisher_init_default(
    &time_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Time),
    "time_publisher");

  // executorの初期化
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  // timerをexecutorに追加
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

	DEBUG_PRINTLN("[Finished]:Micro-ROS Setting");

  digitalWrite(LED_BLUE, HIGH);
}

void loop() {
  // Redirect from PCSerial to IM920sL
  // while(PCSerial.available()){
  //   if(digitalRead(IM920_BUSY) == LOW){
  //     IM920Serial.println(PCSerial.readStringUntil('\r'));
  //     DEBUG_PRINTLN();
  //   }
  // }  

  line = "";
  for(int i=0, j=0; 1; i++, j++){
    if(buf.length() == j){
      do{
        // buf = IM920Serial.readStringUntil('\n');
        buf = PCSerial.readStringUntil('\r');
      }while(buf.length() <= 0);
      j=0;
    }
    if(buf[j] == 'T' || buf[j] == 'R' || buf[j] == 'G' || buf[j] == 'A'){
      if(i!=0){
        buf = buf.substring(j);
        break;
      }
    }
    line += buf[j];
  }
  DEBUG_PRINT("line = ");
  DEBUG_PRINTLN(line);

  switch (line[0])  {
    case 'T':
      char time_buf[64];

      line = line.substring(1);
      pareInt(&time_msg.time);

      line.toCharArray(time_buf, 64);
      time_msg.date.data = time_buf;
      time_msg.date.size = line.length();
      time_msg.date.capacity = 64;

      RCSOFTCHECK(rcl_publish(&time_publisher, &time_msg, NULL));
      break;
		case 'R':
			line = line.substring(1);
      pareInt(&rotation_msg.time);
      pareDouble(&rotation_msg.yaw);
      pareDouble(&rotation_msg.pitch);
      pareDouble(&rotation_msg.roll);
      RCSOFTCHECK(rcl_publish(&rotation_publisher, &rotation_msg, NULL));
      break;
		case 'G':
      line = line.substring(1);
      pareInt(&gps_msg.time);
      pareDouble(&gps_msg.lat);
      pareDouble(&gps_msg.lng);
      pareDouble(&gps_msg.altitude);
      pareDouble(&gps_msg.speed);
      RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
      break;
		case 'A':
      line = line.substring(1);
      pareInt(&accelerometer_msg.time);
      pareDouble(&accelerometer_msg.x);
      pareDouble(&accelerometer_msg.y);
      pareDouble(&accelerometer_msg.z);
      RCSOFTCHECK(rcl_publish(&accelerometer_publisher, &accelerometer_msg, NULL));
      break;
  }
}

}

void pareDouble(double *x){
  int i;
	*x = line.toDouble();
  for(i=0; line[i] != '\0' && line[i] != '/'; i++);  
	line = line.substring(i+1);
}

void pareInt(int *x){
  int i;
	*x = line.toInt();
  for(i=0; line[i] != '\0' && line[i] != '/'; i++);  
	line = line.substring(i+1);
}

