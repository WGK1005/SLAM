#!/usr/bin/env python3
"""
WLR-716 激光雷达扫描器：自动连接设备、发送 34 字节激活控制命令、发现雷达真实 IP。
用法:
  python3 scan_lidar.py                 # 自动探测并扫描 192.168.100.0/24, 192.168.0.0/24, 192.168.1.0/24
  python3 scan_lidar.py 192.168.100.0/24 # 指定网段
"""
import sys
import ipaddress
import socket
from concurrent.futures import ThreadPoolExecutor

COMMAND = bytes.fromhex(
    "FF AA 00 1E 00 00 00 00 00 00 01 01 00 05 00 00 "
    "00 00 00 00 00 00 05 01 00 00 00 00 00 00 00 1F EE EE"
)
PORTS = [2110]
CONNECT_TIMEOUT = 0.50
REPLY_TIMEOUT = 1.0

def test_ip(ip, port=2110):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(CONNECT_TIMEOUT)
        s.connect((ip, port))
        s.sendall(COMMAND)
        s.settimeout(REPLY_TIMEOUT)
        data = s.recv(1024)
        s.close()
        tag = ""
        if len(data) >= 32 and b"WLR" in data[:32]:
            tag = " [型号确认: WLR系列]"
        return ip, True, len(data), tag
    except (socket.timeout, ConnectionRefusedError, OSError):
        return ip, False, 0, ""

def scan_network(net_str):
    net = ipaddress.IPv4Network(net_str, strict=False)
    print(f"\n🔍 正在扫描网段: {net} (端口: {PORTS[0]})...")
    found = []
    hosts = [str(h) for h in net.hosts()]
    with ThreadPoolExecutor(max_workers=50) as executor:
        results = executor.map(test_ip, hosts)
        for ip, ok, length, tag in results:
            if ok:
                print(f"  🎉 发现响应设备! IP={ip}, 端口={PORTS[0]}, 应答={length}字节{tag}")
                found.append((ip, length))
    return found

def main():
    target_nets = []
    if len(sys.argv) > 1:
        target_nets = [sys.argv[1]]
    else:
        # 默认优先扫描 192.168.100.0/24 (当前实测网段) 以及常见出厂网段
        target_nets = ["192.168.100.0/24", "192.168.0.0/24", "192.168.1.0/24"]

    all_found = []
    for net in target_nets:
        res = scan_network(net)
        all_found.extend(res)

    print("\n" + "="*50)
    if all_found:
        print("✅ 扫描完成！发现雷达 IP 地址:")
        for ip, length in all_found:
            print(f"   👉 {ip} (端口: {PORTS[0]}, 应答: {length} 字节)")
    else:
        print("❌ 扫描完成，未发现响应 WLR-716 心跳指令的设备。")
    print("="*50)

if __name__ == "__main__":
    main()
