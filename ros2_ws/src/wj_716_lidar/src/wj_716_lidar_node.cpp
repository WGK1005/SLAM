#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <thread>
#include "async_client.h"
#include "wj_716_lidar_protocol.h"

using namespace wj_lidar;

static wj_716_lidar_protocol *g_protocol = nullptr;

static void CallBackRead(const char* /*addr*/, int /*port*/, unsigned char* data, const int len)
{
  if (g_protocol)
  {
    g_protocol->dataProcess(data, len);
  }
}

class Wj716LidarNode : public rclcpp::Node
{
public:
  Wj716LidarNode()
    : Node("wj_716_lidar_01")
  {
    // 声明参数及默认值
    this->declare_parameter<std::string>("hostname", "192.168.0.2");
    this->declare_parameter<int>("port", 2110);
    this->declare_parameter<double>("min_ang", -2.35619449);
    this->declare_parameter<double>("max_ang", 2.35619449);
    this->declare_parameter<double>("angle_increment", 0.00582);
    this->declare_parameter<std::string>("frame_id", "laser");
    this->declare_parameter<double>("time_increment", 0.00006167129);
    this->declare_parameter<double>("range_min", 0.05);
    this->declare_parameter<double>("range_max", 25.0);
    this->declare_parameter<int>("resize", 811);
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<std::string>("scan_two_echo_topic", "scan_TwoEcho");
  }

  void init()
  {
    hostname_ = this->get_parameter("hostname").as_string();
    port_ = this->get_parameter("port").as_int();

    LidarConfig cfg;
    cfg.frame_id = this->get_parameter("frame_id").as_string();
    cfg.min_ang = this->get_parameter("min_ang").as_double();
    cfg.max_ang = this->get_parameter("max_ang").as_double();
    cfg.angle_increment = this->get_parameter("angle_increment").as_double();
    cfg.time_increment = this->get_parameter("time_increment").as_double();
    cfg.range_min = this->get_parameter("range_min").as_double();
    cfg.range_max = this->get_parameter("range_max").as_double();
    cfg.resize = this->get_parameter("resize").as_int();

    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    std::string two_echo_topic = this->get_parameter("scan_two_echo_topic").as_string();

    pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 50);
    pub_two_echo_ = this->create_publisher<sensor_msgs::msg::LaserScan>(two_echo_topic, 5);

    protocol_ = std::make_unique<wj_716_lidar_protocol>(
        shared_from_this(), pub_scan_, pub_two_echo_, cfg);
    g_protocol = protocol_.get();

    client_ = std::make_unique<Async_Client>(&CallBackRead);

    // 2秒心跳定时器
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&Wj716LidarNode::timerCallback, this));

    // 连接监控定时器 (每2秒检查一次)
    conn_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&Wj716LidarNode::checkConnection, this));

    RCLCPP_INFO(this->get_logger(), "[WJ-716] Target LiDAR IP: %s, Port: %d", hostname_.c_str(), port_);
    RCLCPP_INFO(this->get_logger(), "[WJ-716] Connecting to laser...");

    if (client_->connect(hostname_, port_))
    {
      RCLCPP_INFO(this->get_logger(), "[WJ-716] Successfully connected. Hello wj_716_lidar!");
      timerCallback(); // 立即发送激活报文
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[WJ-716] Initial connection failed. Background reconnect will retry.");
      std::thread t(&Async_Client::reconnect, client_.get());
      t.detach();
    }
  }

  ~Wj716LidarNode() override
  {
    g_protocol = nullptr;
    if (client_)
    {
      client_->disconnect();
    }
  }

private:
  void timerCallback()
  {
    static const unsigned char scanmodeset1[34] = {
      0xFF, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
      0xEE, 0xEE
    };
    if (client_ && client_->m_bConnected)
    {
      client_->sendMsg(scanmodeset1, 34);
    }
  }

  void checkConnection()
  {
    if (!client_) return;

    if (!client_->m_bConnected && !client_->m_bReconnecting)
    {
      RCLCPP_INFO(this->get_logger(), "[WJ-716] Reconnecting to %s:%d...", hostname_.c_str(), port_);
      std::thread t(&Async_Client::reconnect, client_.get());
      t.detach();
    }
  }

  std::string hostname_;
  int port_{2110};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_two_echo_;

  std::unique_ptr<wj_716_lidar_protocol> protocol_;
  std::unique_ptr<Async_Client> client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr conn_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Wj716LidarNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
