#include <Arduino.h>

#include <sys/time.h>
#define micro_rollover_useconds 4294967295

#if !defined(_POSIX_TIMERS) || !_POSIX_TIMERS

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
    static uint32_t rollover = 0;
    static uint32_t last_measure = 0;

    uint32_t m = micros();
    rollover += (m < last_measure) ? 1 : 0;

    uint64_t real_us = (uint64_t) (m + rollover * micro_rollover_useconds);
    tp->tv_sec = real_us / 1000000;
    tp->tv_nsec = (real_us % 1000000) * 1000;
    last_measure = m;

    return 0;
}

#endif  // ifndef _POSIX_TIMERS
