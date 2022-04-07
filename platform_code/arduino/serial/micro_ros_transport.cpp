#include <Arduino.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

extern "C" {

bool platformio_transport_open(struct uxrCustomTransport * transport)
{
  Serial.begin(115200);
  return true;
}

bool platformio_transport_close(struct uxrCustomTransport * transport)
{
  Serial.end();
  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode)
{
  (void)errcode;
  size_t sent = Serial.write(buf, len);
  return sent;
}

size_t platformio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  (void)errcode;
  Serial.setTimeout(timeout);
  return Serial.readBytes((char *)buf, len);
}

}