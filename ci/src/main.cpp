#include <Arduino.h>

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#include <WiFi.h>
#endif

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// Test custom transports
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
bool platformio_transport_open(struct uxrCustomTransport * transport) {return false;};
bool platformio_transport_close(struct uxrCustomTransport * transport) {return false;};
size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {return 0;};
size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {return 0;};
#endif

// Test extra packages
#include <control_msgs/msg/joint_controller_state.h>
#include <my_custom_message/msg/my_custom_message.h>
control_msgs__msg__JointControllerState control_message;
my_custom_message__msg__MyCustomMessage custom_msg;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
  byte local_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  IPAddress local_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 113);
  size_t agent_port = 8888;

  set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) || defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA)
  IPAddress agent_ip(192, 168, 1, 113);
  size_t agent_port = 8888;

  char ssid[] = "WIFI_SSID";
  char psk[]= "WIFI_PSK";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
  rmw_uros_set_custom_transport(
    MICROROS_TRANSPORTS_FRAMING_MODE,
    NULL,
    platformio_transport_open,
    platformio_transport_close,
    platformio_transport_write,
    platformio_transport_read
  );
#else
#error "No transport defined"
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "microros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "microros_platformio_publisher"));

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}