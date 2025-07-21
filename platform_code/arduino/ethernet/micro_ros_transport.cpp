#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

void *transport_args;

extern "C" {

  static WiFiUDP udp_client;

  bool platformio_transport_open(struct uxrCustomTransport *transport) {
    transport_args = transport->args;
    struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;
    bool success = udp_client.begin(locator->port);
    return success;
  }

  bool platformio_transport_close(struct uxrCustomTransport *transport) {
    transport_args = nullptr;
    udp_client.stop();
    return true;
  }

  size_t platformio_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
    struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;

    size_t sent = 0;
    if (true == udp_client.beginPacket(locator->address, locator->port)) {
      sent = udp_client.write(buf, len);
      if (true == udp_client.endPacket()) {
        udp_client.flush();
        if (sent < len) {
          *errcode = 1;  // Incomplete write
          return sent;
        }
        return sent;
      }
    }

    *errcode = 2;  // Write failed
    return 0;
  }

  size_t platformio_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
    int64_t start_time = uxr_millis();

    // Wait for packet or timeout
    while ((uxr_millis() - start_time) < ((int64_t)timeout)) {
      int packet_size = udp_client.parsePacket();
      if (packet_size > 0) {
        size_t available = udp_client.read(buf, len);
        if (available < len) {
          *errcode = 1;  // Incomplete read
          return available;
        }
        return available;
      }
      delay(1);
    }

    *errcode = 2;  // Timeout
    return 0;
  }
}