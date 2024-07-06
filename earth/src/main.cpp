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

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

//micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t position_publisher;
custom_message__msg__Position position_msg;
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
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
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

  // executorの初期化
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  // timerをexecutorに追加
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  position_msg.x = 0;
  position_msg.y = 0;
  position_msg.z = 0;

  // セットアップが完了するとLEDが点灯する
  digitalWrite(LED_BLUE, HIGH);
	DEBUG_PRINTLN("[Finished]:Micro-ROS Setting");
  Wire.begin(21, 22); // SDAピンは21、SCLピンは22
  if (!bno08x.begin_I2C()) {
    PCSerial.println("BNO08xチップが見つかりませんでした");
    while (1) { delay(10); }
  }
  PCSerial.println("BNO08xが見つかりました！");

  setReports();

  PCSerial.println("イベントを読み込んでいます");
  delay(100);
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
      case SH2_ACCELEROMETER:
        PCSerial.print("加速度: ");
        PCSerial.print(sensorValue.un.accelerometer.x); PCSerial.print(", ");
        PCSerial.print(sensorValue.un.accelerometer.y); PCSerial.print(", ");
        PCSerial.print(sensorValue.un.accelerometer.z); PCSerial.println(" m/s^2");
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        PCSerial.print("ジャイロ: ");
        PCSerial.print(sensorValue.un.gyroscope.x); PCSerial.print(", ");
        PCSerial.print(sensorValue.un.gyroscope.y); PCSerial.print(", ");
        PCSerial.print(sensorValue.un.gyroscope.z); PCSerial.println(" rad/s");
        break;
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        PCSerial.print("ヨー: "); PCSerial.print(ypr.yaw);
        PCSerial.print(", ピッチ: "); PCSerial.print(ypr.pitch);
        PCSerial.print(", ロール: "); PCSerial.println(ypr.roll);
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
  PCSerial.println("希望するレポートを設定しています");
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
    PCSerial.println("加速度計を有効にできませんでした");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    PCSerial.println("ジャイロスコープを有効にできませんでした");
  }
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
    PCSerial.println("安定化されたリモートベクトルを有効にできませんでした");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


