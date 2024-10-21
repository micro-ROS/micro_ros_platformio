#include <string.h>

struct micro_ros_agent_locator {
    char * address;
    int port;
};

static inline void set_microros_socket_transports(const char * agent_address, uint16_t agent_port){
    static struct micro_ros_agent_locator locator;
    locator.address = strdup(agent_address);
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
