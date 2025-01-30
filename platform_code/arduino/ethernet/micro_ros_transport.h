#ifndef MICRO_ROS_TRANSPORT_H_
#define MICRO_ROS_TRANSPORT_H_

#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

typedef void (*ethernet_event_callback_t)(arduino_event_id_t event, void* event_info);

struct micro_ros_agent_locator {
    IPAddress address;    // Agent IP address
    IPAddress gateway;    // Gateway IP address
    const char* hostname; // Device hostname
    int port;            // Agent port
    
    ethernet_event_callback_t on_eth_event = nullptr;
};

extern void* transport_args;

extern "C" {
    bool platformio_transport_open(struct uxrCustomTransport * transport);
    bool platformio_transport_close(struct uxrCustomTransport * transport);
    size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
    size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
}

static void ethernet_event_handler(WiFiEvent_t event) {
    auto* locator = (micro_ros_agent_locator*)transport_args;
    if (!locator || !locator->on_eth_event) return;

    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            if (locator->hostname != nullptr) {
                ETH.setHostname(locator->hostname);
            }
            locator->on_eth_event(event, nullptr);
            break;
            
        case ARDUINO_EVENT_ETH_CONNECTED:
            locator->on_eth_event(event, nullptr);
            break;
            
        case ARDUINO_EVENT_ETH_GOT_IP: {
            IPAddress ip = ETH.localIP();
            locator->on_eth_event(event, &ip);
            break;
        }
            
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            locator->on_eth_event(event, nullptr);
            break;
            
        case ARDUINO_EVENT_ETH_STOP:
            locator->on_eth_event(event, nullptr);
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
    ethernet_event_callback_t event_callback = nullptr
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
    locator.on_eth_event = event_callback;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}

#endif // MICRO_ROS_TRANSPORT_H_