#include <micro_ros_platformio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

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

  }
}

void timer_callback(rcl_timer_t * inner_timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (inner_timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

volatile uint32_t system_millis;

/* Called when systick fires */
void sys_tick_handler(void)
{
	system_millis++;
}

void setup() {

  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  systick_set_reload(168000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
  systick_interrupt_enable();

  // Enable uart
	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART6);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);

	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF1, GPIO6);

	/* Setup UART parameters. */
	usart_set_baudrate(USART6, 115200);
	usart_set_databits(USART6, 8);
	usart_set_stopbits(USART6, USART_STOPBITS_1);
	usart_set_mode(USART6, USART_MODE_TX);
	usart_set_parity(USART6, USART_PARITY_NONE);
	usart_set_flow_control(USART6, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART6);

  while(1) {
    usart_send_blocking(USART6, 'a');
    usart_send_blocking(USART6, '\n');
  }


  set_microros_serial_transports();


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

int main(){
  setup();
  while(1){
    loop();
  }
  return 1;
}

// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>
// #include <libopencm3/cm3/nvic.h>
// #include <libopencm3/cm3/systick.h>

// /* monotonically increasing number of milliseconds from reset
//  * overflows every 49 days if you're wondering
//  */
// volatile uint32_t system_millis;

// /* Called when systick fires */
// void sys_tick_handler(void)
// {
// 	system_millis++;
// }

// /* sleep for delay milliseconds */
// static void msleep(uint32_t delay)
// {
// 	uint32_t wake = system_millis + delay;
// 	while (wake > system_millis);
// }

// /* Set up a timer to create 1mS ticks. */
// static void systick_setup(void)
// {
// 	/* clock rate / 1000 to get 1mS interrupt rate */
// 	systick_set_reload(168000);
// 	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
// 	systick_counter_enable();
// 	/* this done last */
// 	systick_interrupt_enable();
// }

// /* Set STM32 to 168 MHz. */
// static void clock_setup(void)
// {
// 	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

// 	/* Enable GPIOD clock. */
// 	rcc_periph_clock_enable(RCC_GPIOC);
// }

// static void gpio_setup(void)
// {
// 	/* Set GPIO11-15 (in GPIO port D) to 'output push-pull'. */
// 	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
// }

// int main(void)
// {
// 	clock_setup();
// 	gpio_setup();
// 	systick_setup();

// 	/* Set two LEDs for wigwag effect when toggling. */
// 	gpio_set(GPIOC, GPIO13);

// 	/* Blink the LEDs (PD12, PD13, PD14 and PD15) on the board. */
// 	while (1) {
// 		gpio_toggle(GPIOC, GPIO13);
// 		msleep(1000);
// 	}

// 	return 0;
// }