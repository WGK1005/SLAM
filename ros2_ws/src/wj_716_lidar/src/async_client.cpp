#include "async_client.h"

Async_Client::Async_Client(fundata_t fundata)
  : m_fundata(fundata)
{
  m_bConnected = false;
  m_bReconnecting = false;
  m_pSocket = std::make_shared<boost::asio::ip::tcp::socket>(m_io);
  std::cout << "[WJ-716] TCP client initialized." << std::endl;
}

Async_Client::~Async_Client()
{
  disconnect();
  m_fundata = nullptr;
}

bool Async_Client::connect(const std::string& ip, int port)
{
  try
  {
    if (m_bConnected)
    {
      return false;
    }
    m_sServerIp = ip;
    m_iServerPort = port;

    boost::system::error_code ec;
    auto addr = boost::asio::ip::make_address(ip, ec);
    if (ec)
    {
      std::cerr << "[WJ-716] Invalid IP: " << ip << " (" << ec.message() << ")" << std::endl;
      return false;
    }

    m_ep = boost::asio::ip::tcp::endpoint(addr, port);
    if (!m_pSocket || !m_pSocket->is_open())
    {
      m_pSocket = std::make_shared<boost::asio::ip::tcp::socket>(m_io);
    }

    m_pSocket->connect(m_ep, ec);
    if (ec)
    {
      return false;
    }

    m_bConnected = true;
    std::thread recv_thread(&Async_Client::recvData, this);
    recv_thread.detach();
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[WJ-716] Exception in connect: " << e.what() << std::endl;
    return false;
  }
}

bool Async_Client::disconnect()
{
  try
  {
    m_bConnected = false;
    if (m_pSocket && m_pSocket->is_open())
    {
      boost::system::error_code ec;
      m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
      m_pSocket->close(ec);
    }
    return true;
  }
  catch (const std::exception& e)
  {
    return false;
  }
}

void Async_Client::recvData()
{
  try
  {
    while (m_bConnected)
    {
      unsigned char buf[10240];
      boost::system::error_code ec;
      size_t len = m_pSocket->read_some(boost::asio::buffer(buf), ec);
      if (ec)
      {
        std::cerr << "[WJ-716] Read error or connection closed: " << ec.message() << std::endl;
        break;
      }
      if (len > 0 && m_fundata)
      {
        (*m_fundata)(m_sServerIp.c_str(), m_iServerPort, buf, static_cast<int>(len));
      }
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "[WJ-716] Exception in recvData: " << e.what() << std::endl;
  }
  m_bConnected = false;
}

void Async_Client::reconnect()
{
  m_bReconnecting = true;
  while (!m_bConnected)
  {
    if (m_pSocket && m_pSocket->is_open())
    {
      boost::system::error_code ec;
      m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
      m_pSocket->close(ec);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "[WJ-716] Attempting to reconnect to " << m_sServerIp << ":" << m_iServerPort << "..." << std::endl;
    if (connect(m_sServerIp, m_iServerPort))
    {
      std::cout << "[WJ-716] Successfully reconnected!" << std::endl;
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
  m_bReconnecting = false;
}

bool Async_Client::sendMsg(const unsigned char buf[], int length)
{
  try
  {
    if (m_pSocket && m_pSocket->is_open() && m_bConnected)
    {
      boost::system::error_code ec;
      size_t sent = m_pSocket->send(boost::asio::buffer(buf, length), 0, ec);
      return (!ec && static_cast<int>(sent) == length);
    }
  }
  catch (const std::exception&)
  {
    return false;
  }
  return false;
}
