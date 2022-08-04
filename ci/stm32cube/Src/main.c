/* USER CODE BEGIN Includes */
#include <micro_ros_platformio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <std_msgs/msg/int32.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Error_Handler();             \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

static uint8_t uart_rbuffer[2048];
static uint8_t uart_tbuffer[2048];
static struct DMAStream stream = {
    .uart = &huart1,
    .rbuffer_size = 2048,
    .rbuffer = uart_rbuffer,
    .tbuffer_size = 2048,
    .tbuffer = uart_tbuffer,
};
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart1) {
    uart_transfer_complete_callback(&stream);
  }
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 2 */
  set_microros_serial_transports(&stream);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "microros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "microros_platformio_publisher"));

  // create timer,
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
