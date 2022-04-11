
static inline void set_microros_serial_transports(){
	rmw_uros_set_custom_transport(
		true,
		NULL,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
}