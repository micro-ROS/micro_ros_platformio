#define ETH_PHY_POWER 5
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_PHY_ADDR 0
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// Network configuration
const IPAddress kClientIP(10, 4, 4, 177);
const IPAddress kGateway(10, 4, 4, 1);
const IPAddress kNetmask(255, 255, 255, 0);
const IPAddress kAgentIP(10, 4, 4, 187);
const uint16_t kAgentPort = 8888;
const char* kHostname = "micro-ros-eth";

// ROS node configuration
const char* kNodeName = "eth_pubsub_node";
const char* kPublisherTopic = "micro_ros_response";
const char* kSubscriberTopic = "micro_ros_name";
const int kExecutorTimeout = 100;  // ms
const size_t kDomainId = 8;        // ROS domain ID

// ROS entities
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;

// Message buffers
std_msgs__msg__String received_msg;
std_msgs__msg__String response_msg;
char received_buffer[50];
char response_buffer[100];

// Connection management
enum class ConnectionState {
  kInitializing,
  kWaitingForAgent,
  kConnecting,
  kConnected,
  kDisconnected
};
ConnectionState connection_state = ConnectionState::kInitializing;

// Forward declarations
void HandleConnectionState();
void SubscriptionCallback(const void* msgin);
void PublishResponse();
bool CreateEntities();
void DestroyEntities();

// Ethernet event callback
void OnEthernetEvent(arduino_event_id_t event, void* event_info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Connected");
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Disconnected");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] Got IP: ");
      Serial.println(*(IPAddress*)event_info);
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("[INIT] Starting micro-ROS node...");

  // Initialize message buffers
  received_msg.data.data = received_buffer;
  received_msg.data.capacity = sizeof(received_buffer);
  response_msg.data.data = response_buffer;
  response_msg.data.capacity = sizeof(response_buffer);

  // Initialize micro-ROS transport with unified callback
  set_microros_ethernet_transports(
    kClientIP, kGateway, kNetmask, kAgentIP, kAgentPort,
    kHostname, OnEthernetEvent);

  delay(2000);
  connection_state = ConnectionState::kWaitingForAgent;
}

void loop() {
  HandleConnectionState();
  delay(100);  // Prevent tight loop
}

void HandleConnectionState() {
  switch (connection_state) {
    case ConnectionState::kWaitingForAgent:
      if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
        Serial.println("[ROS] Agent found, establishing connection...");
        connection_state = ConnectionState::kConnecting;
      }
      break;

    case ConnectionState::kConnecting:
      if (CreateEntities()) {
        Serial.println("[ROS] Connected and ready!");
        connection_state = ConnectionState::kConnected;
      } else {
        Serial.println("[ROS] Connection failed, retrying...");
        connection_state = ConnectionState::kWaitingForAgent;
      }
      break;

    case ConnectionState::kConnected:
      if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
        Serial.println("[ROS] Agent disconnected!");
        connection_state = ConnectionState::kDisconnected;
      } else {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(kExecutorTimeout));
      }
      break;

    case ConnectionState::kDisconnected:
      DestroyEntities();
      Serial.println("[ROS] Waiting for agent...");
      connection_state = ConnectionState::kWaitingForAgent;
      break;

    default:
      break;
  }
}

void SubscriptionCallback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  Serial.print("[SUB] Received: ");
  Serial.println(msg->data.data);

  // Create response message: "Hello <received_name>!"
  snprintf(response_buffer, sizeof(response_buffer), "Hello %s!", msg->data.data);
  response_msg.data.size = strlen(response_buffer);

  // Publish response
  PublishResponse();
}

void PublishResponse() {
  rcl_ret_t ret = rcl_publish(&publisher, &response_msg, NULL);
  if (ret == RCL_RET_OK) {
    Serial.print("[PUB] Published: ");
    Serial.println(response_msg.data.data);
  } else {
    Serial.println("[PUB] Error publishing message");
  }
}

bool CreateEntities() {
  allocator = rcl_get_default_allocator();

  // Initialize options and set domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    return false;
  }
  if (rcl_init_options_set_domain_id(&init_options, kDomainId) != RCL_RET_OK) {
    return false;
  }

  // Initialize support with domain ID options
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Clean up initialization options
  if (rcl_init_options_fini(&init_options) != RCL_RET_OK) {
    return false;
  }

  // Initialize node and rest of entities
  if (rclc_node_init_default(&node, kNodeName, "", &support) != RCL_RET_OK) {
    return false;
  }

  // Create publisher
  if (rclc_publisher_init_default(&publisher, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                  kPublisherTopic)
      != RCL_RET_OK) {
    return false;
  }

  // Create subscriber
  if (rclc_subscription_init_default(&subscriber, &node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                     kSubscriberTopic)
      != RCL_RET_OK) {
    return false;
  }

  // Initialize executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    return false;
  }

  // Add subscriber to executor
  if (rclc_executor_add_subscription(&executor, &subscriber, &received_msg,
                                     &SubscriptionCallback, ON_NEW_DATA)
      != RCL_RET_OK) {
    return false;
  }

  return true;
}

void DestroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc = RCL_RET_OK;
  rc = rcl_subscription_fini(&subscriber, &node);
  rc = rcl_publisher_fini(&publisher, &node);
  rclc_executor_fini(&executor);
  rc = rcl_node_fini(&node);
  rclc_support_fini(&support);
}