#include <Stream.h>
#include <rmw_microros/rmw_microros.h>
#include "micro_ros_transport.cpp"

static inline void set_microros_serial_transports(Stream & stream){
	rmw_uros_set_custom_transport(
		true,
		&stream,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
}