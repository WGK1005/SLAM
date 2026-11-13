#ifndef ASYNC_CLIENT_H
#define ASYNC_CLIENT_H

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <boost/asio.hpp>

typedef void (*fundata_t)(const char* addr, int port, unsigned char* data, const int len);

class Async_Client
{
public:
  explicit Async_Client(fundata_t fundata);
  ~Async_Client();

  bool connect(const std::string& ip, int port);
  bool disconnect();
  void recvData();
  void reconnect();
  bool sendMsg(const unsigned char buf[], int length);

  std::atomic<bool> m_bConnected{false};
  std::atomic<bool> m_bReconnecting{false};

private:
  std::string m_sServerIp;
  int m_iServerPort{2110};
  boost::asio::io_context m_io;
  boost::asio::ip::tcp::endpoint m_ep;
  std::shared_ptr<boost::asio::ip::tcp::socket> m_pSocket;
  fundata_t m_fundata{nullptr};
};

#endif // ASYNC_CLIENT_H
