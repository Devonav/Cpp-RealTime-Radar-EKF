#include "UdpSocket.h"
#include <iostream>

namespace aegis::net {

UdpSocket::UdpSocket() : m_socket(INVALID_SOCKET), m_initialized(false) {
  // Initialize Winsock
  WSADATA wsaData;
  int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (result != 0) {
    throw std::runtime_error("WSAStartup failed: " + std::to_string(result));
  }

  // Create UDP socket
  m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (m_socket == INVALID_SOCKET) {
    WSACleanup();
    throw std::runtime_error("socket failed: " +
                             std::to_string(WSAGetLastError()));
  }

  m_initialized = true;
}

UdpSocket::~UdpSocket() {
  if (m_socket != INVALID_SOCKET) {
    closesocket(m_socket);
  }
  if (m_initialized) {
    WSACleanup();
  }
}

void UdpSocket::Bind(int port) {
  sockaddr_in service;
  service.sin_family = AF_INET;
  service.sin_addr.s_addr = INADDR_ANY;
  service.sin_port = htons(static_cast<u_short>(port));

  if (bind(m_socket, (SOCKADDR *)&service, sizeof(service)) == SOCKET_ERROR) {
    throw std::runtime_error("bind failed: " +
                             std::to_string(WSAGetLastError()));
  }
}

void UdpSocket::SendTo(const std::string &address, int port, const void *data,
                       size_t size) {
  sockaddr_in dest;
  dest.sin_family = AF_INET;
  dest.sin_port = htons(static_cast<u_short>(port));
  inet_pton(AF_INET, address.c_str(), &dest.sin_addr);

  if (sendto(m_socket, static_cast<const char *>(data), static_cast<int>(size),
             0, (SOCKADDR *)&dest, sizeof(dest)) == SOCKET_ERROR) {
    std::cerr << "sendto failed: " << WSAGetLastError() << std::endl;
  }
}

int UdpSocket::ReceiveFrom(void *buffer, size_t size,
                           std::string &senderAddress, int &senderPort) {
  sockaddr_in sender;
  int senderLen = sizeof(sender);

  int bytesReceived =
      recvfrom(m_socket, static_cast<char *>(buffer), static_cast<int>(size), 0,
               (SOCKADDR *)&sender, &senderLen);

  if (bytesReceived == SOCKET_ERROR) {
    int error = WSAGetLastError();
    if (error != WSAEWOULDBLOCK) {
      std::cerr << "recvfrom failed: " << error << std::endl;
    }
    return -1;
  }

  char ipStr[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &sender.sin_addr, ipStr, INET_ADDRSTRLEN);
  senderAddress = ipStr;
  senderPort = ntohs(sender.sin_port);

  return bytesReceived;
}

void UdpSocket::SetNonBlocking(bool nonBlocking) {
  u_long mode = nonBlocking ? 1 : 0;
  ioctlsocket(m_socket, FIONBIO, &mode);
}

} // namespace aegis::net
