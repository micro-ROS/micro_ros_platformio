#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

struct micro_ros_agent_locator {
    IPAddress address;    // Agent IP address
    IPAddress gateway;    // Gateway IP address
    const char* hostname; // Device hostname
    int port;            // Agent port
};

static void ethernet_event_handler(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            if (((micro_ros_agent_locator*)transport_args)->hostname != nullptr) {
                ETH.setHostname(((micro_ros_agent_locator*)transport_args)->hostname);
            }
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            break;
        case ARDUINO_EVENT_ETH_STOP:
            break;
        default:
            break;
    }
}

static inline void set_microros_ethernet_transports(
    IPAddress client_ip,
    IPAddress gateway,
    IPAddress netmask,
    IPAddress agent_ip, 
    uint16_t agent_port,
    const char* hostname = nullptr
) {
    static struct micro_ros_agent_locator locator;

    WiFi.onEvent(ethernet_event_handler);
    ETH.begin();
    ETH.config(client_ip, gateway, netmask);
    delay(1000);

    locator.address = agent_ip;
    locator.gateway = gateway;
    locator.hostname = hostname;
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