#!/usr/bin/env bash
# 抓取网线上雷达主动发出的报文，从中找出雷达的真实 IP/MAC
# 用法: sudo bash /home/wgk/wrl716/sniff_lidar.sh
# 运行期间请把雷达电源拔掉再插上（重新上电），雷达开机时一般会发 ARP/广播包
IFACE="${1:-eth0}"
echo "在 ${IFACE} 上抓包 60 秒（排除本机 IPv6 噪声）..."
echo ">>> 现在请给雷达【断电 -> 重新上电】 <<<"
timeout 60 tcpdump -i "${IFACE}" -n -e -c 40 'not ip6 and not ether src 10:b6:76:43:b4:b3'
echo
echo "抓包结束。上面每行里 'ether src' 后面不是 10:b6:76:43:b4:b3 的就是雷达，"
echo "看它的 'IP x.x.x.x' 或 'ARP ... tell x.x.x.x' 就是雷达的真实 IP。"
