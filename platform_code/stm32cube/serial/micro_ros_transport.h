#include "stm32l4xx_hal.h"

static inline void set_microros_serial_transports(UART_HandleTypeDef *handle){
	rmw_uros_set_custom_transport(
		true,
		handle,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
}