
#ifndef MICRO_ROS_ARDUINO
#define MICRO_ROS_ARDUINO

#ifdef __cplusplus
extern "C"
{
#endif

#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

bool platformio_arduino_serial_open(struct uxrCustomTransport * transport);
bool platformio_arduino_serial_close(struct uxrCustomTransport * transport);
size_t platformio_arduino_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t platformio_arduino_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

static inline void set_microros_serial_transports(){
	rmw_uros_set_custom_transport(
		true,
		NULL,
		platformio_arduino_serial_open,
		platformio_arduino_serial_close,
		platformio_arduino_serial_write,
		platformio_arduino_serial_read
	);
}

#ifdef __cplusplus
}
#endif

#endif  // MICRO_ROS_ARDUINO
