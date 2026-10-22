import math
import sys

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmpPose(Node):
    def __init__(self):
        super().__init__('cmp_pose')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.odo = None
        self.n = 0
        self.create_subscription(Odometry, '/Odometry', self.cb, 50)
        self.create_timer(1.0, self.tick)

    def cb(self, m):
        self.odo = m

    def tick(self):
        if self.odo is None:
            return
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            print('  TF map->base_link 查询失败: %s' % str(e)[:80], flush=True)
            return
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = self.odo.pose.pose
        oq = p.orientation
        oyaw = math.atan2(2 * (oq.w * oq.z + oq.x * oq.y), 1 - 2 * (oq.y * oq.y + oq.z * oq.z))
        d = math.hypot(tr.x - p.position.x, tr.y - p.position.y)
        dyaw = math.degrees(yaw - oyaw)
        dyaw = (dyaw + 180) % 360 - 180
        print('  loc=(%7.3f,%7.3f) yaw=%+7.2f° | fastlio=(%7.3f,%7.3f) yaw=%+7.2f° | 差 %6.3f m %+6.2f°'
              % (tr.x, tr.y, math.degrees(yaw), p.position.x, p.position.y,
                 math.degrees(oyaw), d, dyaw), flush=True)
        self.n += 1
        if self.n >= 18:
            raise KeyboardInterrupt


def main():
    rclpy.init()
    n = CmpPose()
    try:
        while rclpy.ok() and n.n < 18:
            rclpy.spin_once(n, timeout_sec=0.2)
    except (KeyboardInterrupt, Exception):
        pass
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
