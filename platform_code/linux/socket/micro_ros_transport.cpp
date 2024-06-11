#include <micro_ros_platformio.h>

#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <string>

extern "C" {

static int fd;

bool platformio_transport_open(struct uxrCustomTransport * transport)
{
  const auto * locator = (const struct micro_ros_agent_locator *) transport->args;

  struct addrinfo hints;

  memset(&hints, 0, sizeof(hints));

  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV;
  hints.ai_protocol = 0;

  const auto port_s = std::to_string(locator->port);
  struct addrinfo * p_addrs = nullptr;

  if(getaddrinfo(locator->address, port_s.c_str(), &hints, &p_addrs) != 0)
    return false;

  struct addrinfo * p_addr;

  for(p_addr = p_addrs; p_addr != nullptr; p_addr = p_addr->ai_next){
    if(0 > (fd = socket(p_addr->ai_family, p_addr->ai_socktype, p_addr->ai_protocol)))
      continue;

    if(0 == connect(fd, p_addr->ai_addr, p_addr->ai_addrlen))
      break;

    close(fd);
  }

  freeaddrinfo(p_addrs);

  return p_addr != nullptr;
}

bool platformio_transport_close(struct uxrCustomTransport * transport)
{
  close(fd);
  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *)
{
  auto ret = send(fd, buf, len, 0);
  return ret < 0 ? 0 : (size_t)ret;
}

size_t platformio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *)
{
  struct pollfd pfd;
  int ret;

  pfd.fd = fd;
  pfd.events = POLLIN;
  ret = poll(&pfd, 1, timeout);
  switch(ret){
    case -1:
    case 0:
      return 0;
    default:
      ret = recv(fd, buf, len, 0);
      return ret < 0 ? 0 : (size_t)ret;
  }
}

}
