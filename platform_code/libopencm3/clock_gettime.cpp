#include <sys/time.h>
#include <stdint.h>

extern uint32_t system_millis;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
    uint32_t m = system_millis * 1000;

    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;

    return 0;
}