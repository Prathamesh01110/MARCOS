#!/usr/bin/env python3
"""TF owner for ONE drone: map -> <base_frame> -> <lidar_frame>.
World pose = spawn offset + PX4 odometry (NED->ENU). TF stamped with the
ODOMETRY message time so scans and TF are perfectly aligned (no wobble)."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeMsg
from px4_msgs.msg import VehicleOdometry


def quat_mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.sx = float(self.get_parameter('spawn_x').value)
        self.sy = float(self.get_parameter('spawn_y').value)
        self.sz = float(self.get_parameter('spawn_z').value)
        yaw = float(self.get_parameter('spawn_yaw').value)
        self.c, self.s = math.cos(yaw), math.sin(yaw)
        self.q_spawn = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.br = TransformBroadcaster(self)
        self.create_subscription(VehicleOdometry, 'fmu/out/vehicle_odometry', self.cb, qos)
        self.get_logger().info(f'TF owner: map -> {self.base_frame} -> {self.lidar_frame}')

    def cb(self, m: VehicleOdometry):
        if not all(math.isfinite(v) for v in list(m.position) + list(m.q)):
            return
        # >>> KEY CHANGE: use the odometry's own timestamp (PX4 µs = sim time) <<<
        if m.timestamp > 0:
            stamp = TimeMsg()
            stamp.sec = int(m.timestamp // 1_000_000)
            stamp.nanosec = int((m.timestamp % 1_000_000) * 1_000)
        else:
            stamp = self.get_clock().now().to_msg()

        lx, ly, lz = m.position[1], m.position[0], -m.position[2]   # NED -> ENU
        wx = self.sx + self.c * lx - self.s * ly
        wy = self.sy + self.s * lx + self.c * ly
        wz = self.sz + lz

        w, x, y, z = m.q
        ql = (w + z, x + y, x - y, w - z)                           # NED/FRD -> ENU/FLU
        n = math.sqrt(sum(v * v for v in ql)) or 1.0
        qw, qx, qy, qz = quat_mul(self.q_spawn, tuple(v / n for v in ql))

        t1 = TransformStamped()
        t1.header.stamp, t1.header.frame_id, t1.child_frame_id = stamp, 'map', self.base_frame
        t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z = wx, wy, wz
        t1.transform.rotation.w, t1.transform.rotation.x = qw, qx
        t1.transform.rotation.y, t1.transform.rotation.z = qy, qz

        t2 = TransformStamped()
        t2.header.stamp, t2.header.frame_id, t2.child_frame_id = stamp, self.base_frame, self.lidar_frame
        t2.transform.rotation.w = 1.0
        self.br.sendTransform([t1, t2])


def main():
    rclpy.init(); rclpy.spin(OdomTF()); rclpy.shutdown()

if __name__ == '__main__':
    main()