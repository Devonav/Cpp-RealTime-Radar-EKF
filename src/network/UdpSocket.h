#pragma once

#include <stdexcept>
#include <string>
#include <vector>
#include <winsock2.h>
#include <ws2tcpip.h>


#pragma comment(lib, "Ws2_32.lib")

namespace aegis::net {

class UdpSocket {
public:
  UdpSocket();
  ~UdpSocket();

  // Prevent copying
  UdpSocket(const UdpSocket &) = delete;
  UdpSocket &operator=(const UdpSocket &) = delete;

  // Initialize the socket
  void Bind(int port);

  // Send data to a specific address and port
  void SendTo(const std::string &address, int port, const void *data,
              size_t size);

  // Receive data (blocking or non-blocking depending on socket state)
  // Returns number of bytes received. Fills senderAddress and senderPort.
  int ReceiveFrom(void *buffer, size_t size, std::string &senderAddress,
                  int &senderPort);

  // Set non-blocking mode
  void SetNonBlocking(bool nonBlocking);

private:
  SOCKET m_socket;
  bool m_initialized;
};

} // namespace aegis::net
