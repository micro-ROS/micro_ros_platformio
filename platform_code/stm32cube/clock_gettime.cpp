#include "stm32l4xx_hal.h"

#include <sys/time.h>
#define micro_rollover_useconds 4294967295

#ifndef _POSIX_TIMERS

static inline uint32_t dwt_get_microseconds(void)
{
    // Return the number of cycles converted to microseconds
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;

    // Get time in microseconds using DWT
    uint32_t us = dwt_get_microseconds();

    // Convert to seconds and nanoseconds
    tp->tv_sec = us / 1000000;
    tp->tv_nsec = (us % 1000000) * 1000;

    return 0;
}
#endif  // ifndef _POSIX_TIMERS