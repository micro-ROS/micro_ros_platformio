
#ifndef MICRO_ROS_PLATFORMIO
#define MICRO_ROS_PLATFORMIO

#include <uxr/client/transport.h>

// In GNU C < 6.0.0 __attribute__((deprecated(msg))) is not supported for enums, used in rmw/types.h
#if __GNUC__ < 6
#define aux__attribute__(x) __attribute__(x)
#define __attribute__(x)
#endif

#include <rmw_microros/rmw_microros.h>

#if __GNUC__ < 6
#undef __attribute__
#define __attribute__(x) aux__attribute__(x)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

bool platformio_transport_open(struct uxrCustomTransport * transport);
bool platformio_transport_close(struct uxrCustomTransport * transport);
size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#include <micro_ros_transport.h>

#endif  // MICRO_ROS_PLATFORMIO
