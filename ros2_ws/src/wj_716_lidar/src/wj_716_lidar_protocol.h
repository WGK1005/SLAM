#ifndef WJ_716_LIDAR_PROTOCOL_H
#define WJ_716_LIDAR_PROTOCOL_H

#include <iostream>
#include <cstring>
#include <cmath>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace wj_lidar
{
  #define MAX_LENGTH_DATA_PROCESS 200000

  struct TagDataCache
  {
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
  };
  typedef struct TagDataCache DataCache;

  struct LidarConfig
  {
    std::string frame_id{"laser"};
    double min_ang{-2.35619449};
    double max_ang{2.35619449};
    double angle_increment{0.00582};
    double time_increment{0.00006167129};
    double range_min{0.05};
    double range_max{25.0};
    int resize{811};
  };

  class wj_716_lidar_protocol
  {
  public:
    wj_716_lidar_protocol(rclcpp::Node::SharedPtr node,
                          rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan,
                          rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_two_echo,
                          const LidarConfig& cfg);

    bool dataProcess(unsigned char *data, const int reclen);
    bool protocl(unsigned char *data, const int len);
    bool OnRecvProcess(unsigned char *data, int len);
    bool checkXor(unsigned char *recvbuf, int recvlen);

    void setConfig(const LidarConfig& cfg);

    bool heartstate{false};

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr twoecho_;

    sensor_msgs::msg::LaserScan scan_;
    sensor_msgs::msg::LaserScan scan_TwoEcho_;

    DataCache m_sdata;
    LidarConfig config_;
    unsigned int g_u32PreFrameNo{0};

    float scandata[811];
    float scandata_te[811];
    float scaninden[811];
  };

} // namespace wj_lidar

#endif // WJ_716_LIDAR_PROTOCOL_H
