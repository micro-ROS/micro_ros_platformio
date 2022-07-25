#include <sys/time.h>

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

constexpr uint64_t GETTICK_ROLLOVER_USECONDS = 4294967296000UL;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp) {
  (void)unused;
  static uint32_t rollover = 0;
  static uint32_t last_measure = 0;

  uint32_t m = HAL_GetTick();
  rollover += (m < last_measure) ? 1 : 0;

  uint64_t real_us =
      (uint64_t)(m * 1000 + rollover * GETTICK_ROLLOVER_USECONDS);
  tp->tv_sec = real_us / 1000000;
  tp->tv_nsec = (real_us % 1000000) * 1000;
  last_measure = m;

  return 0;
}