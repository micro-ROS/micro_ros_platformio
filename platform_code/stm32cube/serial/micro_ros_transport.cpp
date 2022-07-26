#include <micro_ros_platformio.h>

static uint16_t rhead, rtail;
static volatile uint16_t thead, thead_next, ttail;
static uint32_t open_count = 0;

static void stream_flush(DMAStream *stream) {
  static volatile bool mutex = false;

  if ((stream->uart->gState == HAL_UART_STATE_READY) && !mutex) {
    mutex = true;

    if (thead != ttail) {
      uint16_t len =
          thead < ttail ? ttail - thead : stream->tbuffer_size - thead;
      thead_next = (thead + len) & (stream->tbuffer_size - 1);
      HAL_UART_Transmit_DMA(stream->uart, stream->tbuffer + thead, len);
    }
    mutex = false;
  }
}

void uart_transfer_complete_callback(DMAStream *stream) {
  thead = thead_next;
  stream_flush(stream);
}

extern "C" {

bool platformio_transport_open(struct uxrCustomTransport *transport) {
  DMAStream *stream = (DMAStream *)transport->args;

  if (open_count == 0) {
    rhead = rtail = thead = thead_next = ttail = 0;
    HAL_UART_Receive_DMA(stream->uart, stream->rbuffer, stream->rbuffer_size);
  }
  open_count++;

  return true;
}

bool platformio_transport_close(struct uxrCustomTransport *transport) {
  DMAStream *stream = (DMAStream *)transport->args;

  open_count--;
  if (open_count == 0) {
    HAL_UART_DMAStop(stream->uart);
  }

  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport *transport,
                                  const uint8_t *buf, size_t len,
                                  uint8_t *errcode) {
  DMAStream *stream = (DMAStream *)transport->args;

  uint16_t n = len;
  uint16_t buffer_available =
      thead <= ttail ? stream->tbuffer_size - ttail + thead : thead - ttail;
  if (n > buffer_available) n = buffer_available;

  uint16_t n_tail =
      n <= stream->tbuffer_size - ttail ? n : stream->tbuffer_size - ttail;

  memcpy(stream->tbuffer + ttail, buf, n_tail);

  if (n != n_tail) {
    memcpy(stream->tbuffer, buf + n_tail, n - n_tail);
  }

  ttail = (ttail + n) & (stream->tbuffer_size - 1);

  stream_flush(stream);

  return n;
}

size_t platformio_transport_read(struct uxrCustomTransport *transport,
                                 uint8_t *buf, size_t len, int timeout,
                                 uint8_t *errcode) {
  DMAStream *stream = (DMAStream *)transport->args;

  int ms_used = 0;
  while (true) {
    __disable_irq();
    rtail = stream->rbuffer_size - __HAL_DMA_GET_COUNTER(stream->uart->hdmarx);
    __enable_irq();

    if (rhead != rtail || ms_used >= timeout) break;

    HAL_Delay(1);
    ms_used++;
  }

  size_t wrote = 0;
  while ((rhead != rtail) && (wrote < len)) {
    buf[wrote++] = stream->rbuffer[rhead];
    rhead = (rhead + 1) & (stream->rbuffer_size - 1);
  }

  return wrote;
}
}