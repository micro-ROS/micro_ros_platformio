/* This example shows micro-ROS can be used with Wi-Fi transport on the
   ESP32-CAM board.

  NOTE: There are two FIXMES in the code where you need to replace the
  existing code with your Wi-Fi credentials and IP address of the agent.

   To test the ROS 2 communication, you can use the following commands:
    1. Subscribe to the topic /micro_ros_platformio_node_publisher:
        ros2 topic echo /micro_ros_platformio_node_publisher std_msgs/msg/Int32
    2. Publish a message to the topic /cmd_vel:
        ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear:{x: 1.0}, angular:{z: -1.0}}"
*/

#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

// FIXME: Replace with your Wi-Fi credentials.
#define SSID "your ssid"
#define SSID_PW "your password"

// Publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

// Subscriber
static const char *k_twist = "cmd_vel";
static rcl_subscription_t subscriber_twist;
static geometry_msgs__msg__Twist twist_msg;

// Node, executor and timer variables.
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)                                                    \
  {                                                                    \
    rcl_ret_t temp_rc = fn;                                            \
    if ((temp_rc != RCL_RET_OK)) {                                     \
      printf("Failed status on line %d: %d. Message: %s, Aborting.\n", \
             __LINE__, (int)temp_rc, rcl_get_error_string().str);      \
      error_loop(temp_rc);                                             \
    }                                                                  \
  }

#define RCSOFTCHECK(fn)                                               \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK)) {                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, \
             (int)temp_rc);                                           \
    }                                                                 \
  }

// Error handle loop
void error_loop(rcl_ret_t rc) {
  Serial.println("Error loop");
  while (1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
    Serial.println("Published message");
  }
}

void twist_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msg_in;
  Serial.println("Received twist message");
  Serial.print("Linear: ");
  Serial.print(msg->linear.x);
  Serial.print(", ");
  Serial.print("Angular: ");
  Serial.print(msg->angular.z);
  Serial.println();
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  Serial.println("Started");

  char ssid[] = SSID;
  char ssid_pw[] = SSID_PW;
  // FIXME: Replace with your Wi-Fi credentials.
  IPAddress agent_ip(192, 168, 0, 2);
  const uint16_t k_agent_port = 8888;
  set_microros_wifi_transports(ssid, ssid_pw, agent_ip, k_agent_port);
  delay(2000);
  Serial.println("Connected");

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // create timer.
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // Create subscriber.
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_twist, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), k_twist));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_twist, &twist_msg, &twist_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
