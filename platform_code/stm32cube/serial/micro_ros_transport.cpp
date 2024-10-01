#include "stm32l4xx_hal.h"

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

extern "C" {

#ifdef MICRO_ROS_TRANSPORT_SERIAL_POLLING
bool platformio_transport_open(struct uxrCustomTransport * transport)
{
  return true;
}

bool platformio_transport_close(struct uxrCustomTransport * transport)
{
  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
  size_t ret;
  (void)errcode;

  UART_HandleTypeDef *handle = (UART_HandleTypeDef *) transport->args;

  ret = (HAL_UART_Transmit(handle, buf, len, HAL_MAX_DELAY) == HAL_OK) ? len : 0;

  return ret;
}

size_t platformio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  size_t ret;
  (void)errcode;

  UART_HandleTypeDef *handle = (UART_HandleTypeDef *) transport->args;

  ret = (HAL_UART_Receive(handle, buf, len, timeout) == HAL_OK) ? len : 0;

  return ret;
}

#elif MICRO_ROS_TRANSPORT_SERIAL_IT
// Define the size of the UART reception buffer
#define UART_BUFFER_SIZE 2048U

// Static buffer to hold received data
static uint8_t rxBuffer[UART_BUFFER_SIZE] = {0};
// Single byte for interrupt-based reception
static uint8_t rxByte = 0;
// Index where data is read from (circular buffer)
static size_t headIndex = 0;
// Index where new data is written to (circular buffer)
static volatile size_t tailIndex = 0;

bool platformio_transport_open(struct uxrCustomTransport * transport) {
  // Retrieve UART handle from the custom transport structure
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *) transport->args;

  // Start receiving data in interrupt mode, receiving one byte at a time
  HAL_UART_Receive_IT(handle, &rxByte, 1);

  return true;
}

bool platformio_transport_close(struct uxrCustomTransport * transport) {
  // Retrieve UART handle from the custom transport structure
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *) transport->args;

  // Abort ongoing UART operations in interrupt mode
  HAL_UART_Abort_IT(handle);

  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
  size_t ret;
  HAL_StatusTypeDef halStatus;
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *) transport->args;

  // Check if UART is ready to transmit
  if (handle->gState == HAL_UART_STATE_READY) {
    // Initiate interrupt-based transmission of the data
    halStatus = HAL_UART_Transmit_IT(handle, buf, len);

    // Wait for the transmission to complete by checking the UART state
    while ((halStatus == HAL_OK) && (handle->gState != HAL_UART_STATE_READY)) {
      // Add a small delay to prevent a tight loop (1 ms delay)
      HAL_Delay(1);
    }

    // Return the number of bytes transmitted if successful, otherwise 0
    ret = (halStatus == HAL_OK) ? len : 0;
  } else {
    // UART is not ready for transmission, return 0
    ret = 0;
  }

  return ret;
}

size_t platformio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
  size_t wrote = 0;

  // Read data from the buffer as long as there is data available and the requested length has not been reached
  while ((headIndex != tailIndex) && (wrote < len)) {
    // Copy data from the circular buffer to the provided buffer
    buf[wrote] = rxBuffer[headIndex];
    // Increment and wrap headIndex for circular behavior
    headIndex = (headIndex + 1) % UART_BUFFER_SIZE;
    // Increment the number of bytes written to the provided buffer
    wrote++;
  }

  return wrote;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // Ensure the tail index wraps around if it reaches the end of the buffer
  if (tailIndex == UART_BUFFER_SIZE) {
    tailIndex = 0;
  }

  // Store the received byte in the buffer at the current tail index
  rxBuffer[tailIndex] = rxByte;

  // Increment the tail index to point to the next free position in the buffer
  tailIndex++;

  // Continue receiving the next byte in interrupt mode
  HAL_UART_Receive_IT(huart, &rxByte, 1);
}
#elif MICRO_ROS_TRANSPORT_SERIAL_DMA
#warning "MICRO_ROS_TRANSPORT_SERIAL_DMA is not supported yet"
#else
#warning "You need to specify a transport mode for micro-ros"
#endif // MICRO_ROS_TRANSPORT_SERIAL_POLLING
}