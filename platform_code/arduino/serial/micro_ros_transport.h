#include <Stream.h>

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