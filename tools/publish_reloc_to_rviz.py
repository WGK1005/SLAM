#!/usr/bin/env python3
import sys, os, time
import numpy as np

# 确保无用户目录 numpy 冲突
os.environ["PYTHONNOUSERSITE"] = "1"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros

def load_pcd(p):
    if not os.path.exists(p): return np.zeros((0, 3), dtype=np.float32)
    fields, sizes, types, points = [], [], [], 0
    with open(p, 'rb') as f:
        while True:
            line = f.readline()
            if not line: break
            k = line.split()[0].decode(errors='ignore').upper()
            if k == 'POINTS': points = int(line.split()[1])
            elif k == 'FIELDS': fields = [x.decode() for x in line.split()[1:]]
            elif k == 'SIZE': sizes = [int(x) for x in line.split()[1:]]
            elif k == 'TYPE': types = [x.decode() for x in line.split()[1:]]
            elif k == 'DATA': break
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dt = np.dtype([(n, f'<{tmap[t]}{s}') for n, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(points * dt.itemsize), dtype=dt)
    P = np.stack([np.asarray(a['x'], np.float32), np.asarray(a['y'], np.float32), np.asarray(a['z'], np.float32)], 1)
    return P[np.isfinite(P).all(1)]

def numpy_to_pc2(pts, frame_id="map", stamp=None):
    msg = PointCloud2()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg

class RelocRvizPublisher(Node):
    def __init__(self):
        super().__init__('reloc_rviz_publisher')
        self.pub_map = self.create_publisher(PointCloud2, '/map', 1)
        self.pub_coarse = self.create_publisher(PointCloud2, '/scan2map', 10)
        self.pub_fine = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        home = os.path.expanduser('~')
        map_pcd = os.path.join(home, "maps/map_1021_1050.pcd")
        coarse_pcd = "/tmp/reloc_eval/scan_coarse.pcd"
        fine_pcd = "/tmp/reloc_eval/scan_fine.pcd"
        info_file = "/tmp/reloc_eval/info.txt"

        self.get_logger().info("正在加载匹配结果点云至 RViz...")
        # 降采样加载地图
        raw_map = load_pcd(map_pcd)
        step = max(1, len(raw_map) // 50000)
        self.map_pts = raw_map[::step]
        self.coarse_pts = load_pcd(coarse_pcd)
        self.fine_pts = load_pcd(fine_pcd)

        self.info = {}
        if os.path.exists(info_file):
            with open(info_file) as f:
                for line in f:
                    if '=' in line:
                        k, v = line.strip().split('=', 1)
                        self.info[k] = float(v)

        self.timer = self.create_timer(0.2, self.timer_cb)
        self.get_logger().info("✅ RViz 匹配结果数据源就绪！")
        self.get_logger().info("  - 白色/灰白点云: /map (先验地图)")
        self.get_logger().info("  - 红色点云: /scan2map (阶段1: 粗匹配候选)")
        self.get_logger().info("  - 绿色点云: /cloud_registered (阶段2: 精微调咬合)")
        self.get_logger().info("  - 箭头 & TF: /initialpose & map->base_link")

    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        # 广播 TF
        fx = self.info.get('fine_x', 0.0)
        fy = self.info.get('fine_y', 0.0)
        fyaw = self.info.get('fine_yaw', 0.0)
        rad = np.radians(fyaw)
        
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = fx
        t.transform.translation.y = fy
        t.transform.translation.z = 0.0
        t.transform.rotation.z = float(np.sin(rad / 2.0))
        t.transform.rotation.w = float(np.cos(rad / 2.0))
        self.tf_broadcaster.sendTransform(t)

        # 广播 Pose
        p = PoseWithCovarianceStamped()
        p.header.stamp = now
        p.header.frame_id = 'map'
        p.pose.pose.position.x = fx
        p.pose.pose.position.y = fy
        p.pose.pose.orientation.z = float(np.sin(rad / 2.0))
        p.pose.pose.orientation.w = float(np.cos(rad / 2.0))
        self.pub_pose.publish(p)

        # 广播点云
        if len(self.map_pts) > 0:
            self.pub_map.publish(numpy_to_pc2(self.map_pts, 'map', now))
        if len(self.coarse_pts) > 0:
            self.pub_coarse.publish(numpy_to_pc2(self.coarse_pts, 'map', now))
        if len(self.fine_pts) > 0:
            self.pub_fine.publish(numpy_to_pc2(self.fine_pts, 'map', now))

def main():
    rclpy.init()
    node = RelocRvizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
