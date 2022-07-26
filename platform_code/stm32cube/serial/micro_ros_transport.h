#include <rmw_microros/custom_transport.h>

#if defined(STM32F0xx)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1xx)
#include "stm32f1xx_hal.h"
#elif defined(STM32F2xx)
#include "stm32f2xx_hal.h"
#elif defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal.h"
#endif

struct DMAStream {
  UART_HandleTypeDef* uart;
  uint16_t rbuffer_size;
  uint8_t* rbuffer;
  uint16_t tbuffer_size;
  uint8_t* tbuffer;
};

static inline void set_microros_serial_transports(DMAStream& stream) {
  rmw_uros_set_custom_transport(
      true, &stream, platformio_transport_open, platformio_transport_close,
      platformio_transport_write, platformio_transport_read);
}

void uart_transfer_complete_callback(DMAStream* stream);
