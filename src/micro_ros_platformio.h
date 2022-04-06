
#ifndef MICRO_ROS_PLATFORMIO
#define MICRO_ROS_PLATFORMIO

#define __attribute__(x)

#ifdef __cplusplus
extern "C"
{
#endif

#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

bool platformio_transport_open(struct uxrCustomTransport * transport);
bool platformio_transport_close(struct uxrCustomTransport * transport);
size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

extern bool platformio_transport_require_framing;

static inline void set_microros_transports(){
	rmw_uros_set_custom_transport(
		platformio_transport_require_framing,
		NULL,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
}

#ifdef __cplusplus
}
#endif

#endif  // MICRO_ROS_PLATFORMIO
