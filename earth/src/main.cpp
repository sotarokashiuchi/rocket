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
#include <custom_message/msg/position.h>
#include <custom_message/msg/euler.h>
#include <Adafruit_BNO08x.h>
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

#define LED_BLUE 19
#define IM920_BUSY 18
#define BNO08X_CS 10
#define BNO08X_INT 4
#define BNO08X_RESET -1

//エラーハンドリング用のマクロ
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t position_publisher;
rcl_publisher_t euler_publisher;
custom_message__msg__Position position_msg;
custom_message__msg__Euler euler_msg;
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
void quaternionToEuler(float qr, float qi, float qj, float qk);
void setReports();

HardwareSerial PCSerial(0);
HardwareSerial IM920Serial(2);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

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
    &position_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Position),
    "position_publisher");

	rclc_publisher_init_default(
			&euler_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Euler),
			"euler_publisher");

  // executorの初期化
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  // timerをexecutorに追加
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  position_msg.x = 0;
  position_msg.y = 0;
  position_msg.z = 0;
	DEBUG_PRINTLN("[Finished]:Micro-ROS Setting");

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
  position_publish();
  
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
        DEBUG_PRINT("ヨー: "); DEBUG_PRINT(euler_msg.yaw);
        DEBUG_PRINT(", ピッチ: "); DEBUG_PRINT(euler_msg.pitch);
        DEBUG_PRINT(", ロール: "); DEBUG_PRINTLN(euler_msg.roll);
				RCSOFTCHECK(rcl_publish(&euler_publisher, &euler_msg, NULL));
        break;
    }
  }

}

// position_publish
void position_publish() {
  position_msg.x++;
  position_msg.y--;
  position_msg.z=position_msg.x*position_msg.y;
  RCSOFTCHECK(rcl_publish(&position_publisher, &position_msg, NULL));
}

void setReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR), 500) {
    Serial.println("Could not enable quaternion report");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  euler_msg.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
  euler_msg.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  euler_msg.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}


