#!/usr/bin/env python3
"""Fuses scans from ALL drones into ONE world-frame occupancy grid.
Every beam is placed via TF into the shared 'map' frame, so drones starting
anywhere still produce one consistent, correctly-registered map."""
import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

UNK, FREE, OCC = -1, 0, 100
COLORS = [(1.0, 0.2, 0.2), (0.2, 0.5, 1.0), (1.0, 0.7, 0.1)]  # per-drone voxel colors


class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('scan_topics', ['/drone0/scan', '/drone1/scan', '/drone2/scan'])
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('size', 200.0)
        self.declare_parameter('voxel_size', 1.0)
        self.world = self.get_parameter('world_frame').value
        self.topics = list(self.get_parameter('scan_topics').value)
        self.res = float(self.get_parameter('resolution').value)
        self.size = float(self.get_parameter('size').value)
        self.vox = float(self.get_parameter('voxel_size').value)

        self.nx = self.ny = int(self.size / self.res)
        self.ox = self.oy = -self.size / 2.0
        self.grid = np.full((self.ny, self.nx), UNK, dtype=np.int8)
        self.voxel_owner = {}

        self.tf = Buffer()
        TransformListener(self.tf, self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        for i, t in enumerate(self.topics):
            self.create_subscription(LaserScan, t, self.make_cb(i), qos)
            self.get_logger().info(f'Fusing {t} (color {COLORS[i % 3]})')

        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 2)
        self.pub_vox = self.create_publisher(MarkerArray, '/map_3d', 2)
        self.create_timer(1.0, self.publish)

    def to_cell(self, xs, ys):
        ix = ((xs - self.ox) / self.res).astype(np.int32)
        iy = ((ys - self.oy) / self.res).astype(np.int32)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok

    def make_cb(self, idx):
        NH = 360                      # horizontal samples, must match SDF
        V_MIN, V_MAX = -0.4363, 0.2618  # must match SDF vertical angles

        def cb(msg: LaserScan):
            try:
                tf = self.tf.lookup_transform(self.world, msg.header.frame_id,
                                              msg.header.stamp,
                                              timeout=Duration(seconds=0.1))
            except Exception:
                return
            tr, rot = tf.transform.translation, tf.transform.rotation
            ox, oy, oz = tr.x, tr.y, tr.z
            yaw = math.atan2(2 * (rot.w * rot.z + rot.x * rot.y),
                             1 - 2 * (rot.y * rot.y + rot.z * rot.z))
            c, s = math.cos(yaw), math.sin(yaw)

            r_all = np.array(msg.ranges, dtype=np.float32)
            rings = max(1, len(r_all) // NH)
            r_all = r_all[:rings * NH].reshape(rings, NH)

            # per-ring elevation angle (gz stacks rings vertical-outermost)
            if rings > 1:
                elev = np.linspace(V_MIN, V_MAX, rings)[:, None]     # (rings,1)
            else:
                elev = np.zeros((1, 1))
            valid = np.isfinite(r_all) & (r_all >= msg.range_min) & (r_all < msg.range_max)
            r_all = np.where(valid, r_all, np.inf)

            a = msg.angle_min + np.arange(NH) * msg.angle_increment
            ca, sa = np.cos(a), np.sin(a)

            # conservative 2D projection: closest hit per angle across rings
            r = r_all.min(axis=0)                                    # (NH,)
            good = np.isfinite(r)
            if not good.any():
                return
            rh = r[good]; ah_c, ah_s = ca[good], sa[good]
            eh = elev.ravel()[np.argmin(r_all, axis=0)][good]        # elevation of closest hit

            # endpoints in world — WITH height (this is the 2.5D part)
            ex = ox + rh * ah_c * c - rh * ah_s * s
            ey = oy + rh * ah_c * s + rh * ah_s * c
            ez = oz + rh * np.sin(eh)

            # free space carved only along the 2D projection, never through walls
            K = 24
            t = np.linspace(0.15, 0.92, K)[:, None] * rh[None, :]
            fx = ox + t * ah_c[None, :] * c - t * ah_s[None, :] * s
            fy = oy + t * ah_c[None, :] * s + t * ah_s[None, :] * c
            ix, iy, ok = self.to_cell(fx.ravel(), fy.ravel())
            g = self.grid[iy[ok], ix[ok]]
            self.grid[iy[ok], ix[ok]] = np.where(g == OCC, OCC, FREE)

            ix, iy, ok = self.to_cell(ex, ey)
            self.grid[iy[ok], ix[ok]] = OCC

            # 3D voxels now carry real obstacle height per ring
            for X, Y, Z in zip(ex, ey, ez):
                key = (int((X - self.ox) // self.vox),
                       int((Y - self.oy) // self.vox), int(Z // self.vox))
                self.voxel_owner.setdefault(key, idx)
        return cb

    def publish(self):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.world
        m.info.resolution = self.res
        m.info.width, m.info.height = self.nx, self.ny
        m.info.origin.position.x, m.info.origin.position.y = self.ox, self.oy
        m.info.origin.orientation.w = 1.0
        m.data = self.grid.ravel().tolist()
        self.pub_map.publish(m)

        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for (vx, vy, vz), owner in self.voxel_owner.items():
            mk = Marker()
            mk.header.frame_id = self.world
            mk.header.stamp = m.header.stamp
            mk.ns, mk.id, mk.type = f'voxels_{owner}', len(ma.markers), Marker.CUBE
            mk.pose.position.x = self.ox + (vx + 0.5) * self.vox
            mk.pose.position.y = self.oy + (vy + 0.5) * self.vox
            mk.pose.position.z = (vz + 0.5) * self.vox
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = self.vox * 0.95
            r_, g_, b_ = COLORS[owner % 3]
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r_, g_, b_, 0.5
            ma.markers.append(mk)
        self.pub_vox.publish(ma)


def main():
    rclpy.init(); rclpy.spin(MapMerger()); rclpy.shutdown()

if __name__ == '__main__':
    main()