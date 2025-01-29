#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

struct micro_ros_agent_locator {
    IPAddress address;    // Agent IP address
    IPAddress gateway;    // Gateway IP address
    const char* hostname; // Device hostname
    int port;            // Agent port
    
    // Optional event callbacks
    void (*on_eth_connected)() = nullptr;
    void (*on_eth_disconnected)() = nullptr;
    void (*on_eth_got_ip)(IPAddress ip) = nullptr;
};

static void ethernet_event_handler(WiFiEvent_t event) {
    auto* locator = (micro_ros_agent_locator*)transport_args;
    if (!locator) return;

    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            if (locator->hostname != nullptr) {
                ETH.setHostname(locator->hostname);
            }
            break;
            
        case ARDUINO_EVENT_ETH_CONNECTED:
            if (locator->on_eth_connected) {
                locator->on_eth_connected();
            }
            break;
            
        case ARDUINO_EVENT_ETH_GOT_IP:
            if (locator->on_eth_got_ip) {
                locator->on_eth_got_ip(ETH.localIP());
            }
            break;
            
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            if (locator->on_eth_disconnected) {
                locator->on_eth_disconnected();
            }
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
    const char* hostname = nullptr,
    void (*on_connected)() = nullptr,
    void (*on_disconnected)() = nullptr,
    void (*on_got_ip)(IPAddress ip) = nullptr
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
    
    // Store optional callbacks
    locator.on_eth_connected = on_connected;
    locator.on_eth_disconnected = on_disconnected;
    locator.on_eth_got_ip = on_got_ip;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}