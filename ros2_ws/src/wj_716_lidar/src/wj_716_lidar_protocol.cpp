#include "wj_716_lidar_protocol.h"

namespace wj_lidar
{
  wj_716_lidar_protocol::wj_716_lidar_protocol(
      rclcpp::Node::SharedPtr node,
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan,
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_two_echo,
      const LidarConfig& cfg)
    : node_(node), marker_pub_(pub_scan), twoecho_(pub_two_echo)
  {
    std::memset(&m_sdata, 0, sizeof(m_sdata));
    g_u32PreFrameNo = 0;
    heartstate = false;
    setConfig(cfg);
    RCLCPP_INFO(node_->get_logger(), "[WJ-716] Protocol initialized successfully.");
  }

  void wj_716_lidar_protocol::setConfig(const LidarConfig& cfg)
  {
    config_ = cfg;

    scan_.header.frame_id = config_.frame_id;
    scan_.angle_min = static_cast<float>(config_.min_ang);
    scan_.angle_max = static_cast<float>(config_.max_ang);
    scan_.angle_increment = static_cast<float>(config_.angle_increment);
    scan_.time_increment = static_cast<float>(config_.time_increment);
    scan_.range_min = static_cast<float>(config_.range_min);
    scan_.range_max = static_cast<float>(config_.range_max);
    scan_.ranges.resize(config_.resize);
    scan_.intensities.resize(config_.resize);

    scan_TwoEcho_.header.frame_id = config_.frame_id;
    scan_TwoEcho_.angle_min = static_cast<float>(config_.min_ang);
    scan_TwoEcho_.angle_max = static_cast<float>(config_.max_ang);
    scan_TwoEcho_.angle_increment = static_cast<float>(config_.angle_increment);
    scan_TwoEcho_.time_increment = static_cast<float>(config_.time_increment);
    scan_TwoEcho_.range_min = static_cast<float>(config_.range_min);
    scan_TwoEcho_.range_max = static_cast<float>(config_.range_max);
    scan_TwoEcho_.ranges.resize(config_.resize);
    scan_TwoEcho_.intensities.resize(config_.resize);

    RCLCPP_INFO(node_->get_logger(),
                "[WJ-716] Config: frame_id=%s, angle=[%.2f, %.2f] rad, inc=%.5f, range=[%.2f, %.2f] m",
                config_.frame_id.c_str(), config_.min_ang, config_.max_ang,
                config_.angle_increment, config_.range_min, config_.range_max);
  }

  bool wj_716_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
  {
    if (reclen > MAX_LENGTH_DATA_PROCESS)
    {
      return false;
    }

    if (m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
    {
      std::memset(&m_sdata, 0, sizeof(m_sdata));
      return false;
    }
    std::memcpy(&m_sdata.m_acdata[m_sdata.m_u32in], data, reclen * sizeof(char));
    m_sdata.m_u32in += reclen;

    while (m_sdata.m_u32out < m_sdata.m_u32in)
    {
      if (m_sdata.m_acdata[m_sdata.m_u32out] == 0x02 &&
          m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0x02 &&
          m_sdata.m_acdata[m_sdata.m_u32out + 2] == 0x02 &&
          m_sdata.m_acdata[m_sdata.m_u32out + 3] == 0x02)
      {
        unsigned int l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 4] << 24) |
                                    (m_sdata.m_acdata[m_sdata.m_u32out + 5] << 16) |
                                    (m_sdata.m_acdata[m_sdata.m_u32out + 6] << 8)  |
                                    (m_sdata.m_acdata[m_sdata.m_u32out + 7] << 0);
        l_u32reallen += 9;

        if (l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
        {
          if (OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out], l_u32reallen))
          {
            m_sdata.m_u32out += l_u32reallen;
          }
          else
          {
            int i;
            for (i = 1; i <= static_cast<int>(l_u32reallen); i++)
            {
              if ((m_sdata.m_acdata[m_sdata.m_u32out + i] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 1] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 2] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 3] == 0x02))
              {
                m_sdata.m_u32out += i;
                break;
              }
              if (i == static_cast<int>(l_u32reallen))
              {
                m_sdata.m_u32out += l_u32reallen;
              }
            }
          }
        }
        else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
        {
          std::memset(&m_sdata, 0, sizeof(m_sdata));
        }
        else
        {
          break;
        }
      }
      else if (m_sdata.m_acdata[m_sdata.m_u32out] == 0xff && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xaa)
      {
        heartstate = true;
        unsigned int l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8) |
                                    (m_sdata.m_acdata[m_sdata.m_u32out + 3] << 0);
        l_u32reallen += 4;
        m_sdata.m_u32out += l_u32reallen;
      }
      else
      {
        m_sdata.m_u32out++;
      }
    }

    if (m_sdata.m_u32out >= m_sdata.m_u32in)
    {
      std::memset(&m_sdata, 0, sizeof(m_sdata));
    }
    return true;
  }

  bool wj_716_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
  {
    if (len > 0)
    {
      if (checkXor(data, len))
      {
        protocl(data, len);
      }
    }
    else
    {
      return false;
    }
    return true;
  }

  bool wj_716_lidar_protocol::protocl(unsigned char *data, const int /*len*/)
  {
    if ((data[8] == 0x73 && data[9] == 0x52) || (data[8] == 0x73 && data[9] == 0x53))
    {
      static int s_n32ScanIndex = 0;
      int l_n32PackageNo = (data[50] << 8) + data[51];
      unsigned int l_u32FrameNo = (data[46] << 24) + (data[47] << 16) + (data[48] << 8) + data[49];

      if (l_n32PackageNo == 1)
      {
        s_n32ScanIndex = 0;
        g_u32PreFrameNo = l_u32FrameNo;
        heartstate = true;
        for (int j = 0; j < 810; j += 2)
        {
          scandata[s_n32ScanIndex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
          scandata[s_n32ScanIndex] /= 1000.0f;
          if (scandata[s_n32ScanIndex] >= config_.range_max || scandata[s_n32ScanIndex] <= config_.range_min)
          {
            scandata[s_n32ScanIndex] = std::numeric_limits<float>::quiet_NaN();
          }
          s_n32ScanIndex++;
        }
      }
      else if (l_n32PackageNo == 2)
      {
        if (g_u32PreFrameNo == l_u32FrameNo)
        {
          for (int j = 0; j < 812; j += 2)
          {
            scandata[s_n32ScanIndex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
            scandata[s_n32ScanIndex] /= 1000.0f;
            if (scandata[s_n32ScanIndex] >= config_.range_max || scandata[s_n32ScanIndex] <= config_.range_min)
            {
              scandata[s_n32ScanIndex] = std::numeric_limits<float>::quiet_NaN();
            }
            scan_.intensities[s_n32ScanIndex] = 0.0f;
            s_n32ScanIndex++;
          }
        }
        else
        {
          s_n32ScanIndex = 0;
          return false;
        }
      }
      else if (l_n32PackageNo == 3)
      {
        s_n32ScanIndex = 0;
        if (g_u32PreFrameNo == l_u32FrameNo)
        {
          for (int j = 0; j < 810; j += 2)
          {
            scaninden[s_n32ScanIndex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
            s_n32ScanIndex++;
          }
        }
        else
        {
          s_n32ScanIndex = 0;
          return false;
        }
      }
      else if (l_n32PackageNo == 4)
      {
        if (g_u32PreFrameNo == l_u32FrameNo)
        {
          for (int j = 0; j < 812; j += 2)
          {
            scaninden[s_n32ScanIndex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
            s_n32ScanIndex++;
          }

          int index_min = static_cast<int>((config_.min_ang + 2.35619449) / scan_.angle_increment);
          int index_max = 811 - static_cast<int>((2.35619449 - config_.max_ang) / scan_.angle_increment);
          if (index_min < 0) index_min = 0;
          if (index_max > 811) index_max = 811;

          int count = index_max - index_min;
          if (count > 0)
          {
            scan_.ranges.resize(count);
            scan_.intensities.resize(count);

            for (int j = index_min; j < index_max; ++j)
            {
              scan_.ranges[j - index_min] = scandata[j];
              scan_.intensities[j - index_min] = scaninden[j];
            }
          }
        }
        else
        {
          s_n32ScanIndex = 0;
          return false;
        }
      }
      else if (l_n32PackageNo == 5)
      {
        s_n32ScanIndex = 0;
        if (g_u32PreFrameNo == l_u32FrameNo)
        {
          for (int j = 0; j < 810; j += 2)
          {
            scandata_te[s_n32ScanIndex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
            scandata_te[s_n32ScanIndex] /= 1000.0f;
            if (scandata_te[s_n32ScanIndex] >= config_.range_max || scandata_te[s_n32ScanIndex] <= config_.range_min)
            {
              scandata_te[s_n32ScanIndex] = std::numeric_limits<float>::quiet_NaN();
            }
            s_n32ScanIndex++;
          }
        }
        else
        {
          s_n32ScanIndex = 0;
          return false;
        }
      }
      else if (l_n32PackageNo == 6)
      {
        if (g_u32PreFrameNo == l_u32FrameNo)
        {
          for (int j = 0; j < 812; j += 2)
          {
            scandata_te[s_n32ScanIndex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
            scandata_te[s_n32ScanIndex] /= 1000.0f;
            if (scandata_te[s_n32ScanIndex] >= config_.range_max || scandata_te[s_n32ScanIndex] <= config_.range_min)
            {
              scandata_te[s_n32ScanIndex] = std::numeric_limits<float>::quiet_NaN();
            }
            s_n32ScanIndex++;
          }

          int index_min = static_cast<int>((config_.min_ang + 2.35619449) / scan_TwoEcho_.angle_increment);
          int index_max = 811 - static_cast<int>((2.35619449 - config_.max_ang) / scan_TwoEcho_.angle_increment);
          if (index_min < 0) index_min = 0;
          if (index_max > 811) index_max = 811;

          int count = index_max - index_min;
          if (count > 0)
          {
            scan_TwoEcho_.ranges.resize(count);
            scan_TwoEcho_.intensities.resize(count);

            for (int j = index_min; j < index_max; ++j)
            {
              scan_TwoEcho_.ranges[j - index_min] = scandata_te[j];
              scan_TwoEcho_.intensities[j - index_min] = 0.0f;
            }
          }

          auto now_time = node_->now();
          scan_.header.stamp = now_time;
          marker_pub_->publish(scan_);

          scan_TwoEcho_.header.stamp = now_time;
          twoecho_->publish(scan_TwoEcho_);
        }
        else
        {
          s_n32ScanIndex = 0;
          return false;
        }
      }
      return true;
    }
    return false;
  }

  bool wj_716_lidar_protocol::checkXor(unsigned char *recvbuf, int recvlen)
  {
    if (*recvbuf == 0x02)
    {
      unsigned char *p = recvbuf + 8;
      int len = recvlen - 9;
      unsigned char check = 0;
      for (int i = 0; i < len; i++)
      {
        check ^= *p++;
      }
      return (check == *p);
    }
    return false;
  }

} // namespace wj_lidar
