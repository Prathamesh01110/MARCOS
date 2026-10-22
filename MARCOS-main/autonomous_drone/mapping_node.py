#!/usr/bin/env python3
"""
Advanced Mapping Node
- Smart Ghost Clearing
- Vertical Depth Sampling
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from sensor_msgs.msg import Image, LaserScan
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

def get_color_by_height(h, max_h=15.0):
    """Returns ColorRGBA based on height (Blue -> Green -> Red)"""
    val = min(max(h / max_h, 0.0), 1.0)
    c = ColorRGBA()
    c.a = 0.8
    
    # Heatmap logic
    if val < 0.5:
        # Blue to Green
        c.r = 0.0
        c.g = 2.0 * val
        c.b = 1.0 - 2.0 * val
    else:
        # Green to Red
        c.r = 2.0 * (val - 0.5)
        c.g = 1.0 - 2.0 * (val - 0.5)
        c.b = 0.0
    return c

class SimpleMapper:
    def __init__(self, resolution=0.1, size=2000):
        self.resolution = resolution
        self.size = size
        
        # Grid state
        self.grid = np.zeros((size, size), dtype=np.float32)
        self.height_grid = np.zeros((size, size), dtype=np.float32)
        self.hits = np.zeros((size, size), dtype=np.int32)
        
        self.origin = -size * resolution / 2.0
        
        # Thresholds
        self.occupied_threshold = 3.0
        self.free_threshold = -2.0
        self.min_prob = -10.0 # Clamp minimum probability

    def xy_to_grid(self, x, y):
        gx = int((x - self.origin) / self.resolution)
        gy = int((y - self.origin) / self.resolution)
        return gx, gy

    def is_valid(self, gx, gy):
        return 0 <= gx < self.size and 0 <= gy < self.size

    def line_cells(self, x0, y0, x1, y1):
        """Bresenham's Line Algorithm"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + int(dx + dy)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        err = dx - dy
        
        for _ in range(n):
            if self.is_valid(x, y):
                cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def add_ray(self, x0, y0, x1, y1, z, hit):
        gx0, gy0 = self.xy_to_grid(x0, y0)
        gx1, gy1 = self.xy_to_grid(x1, y1)
        
        if not self.is_valid(gx0, gy0): return
        
        cells = self.line_cells(gx0, gy0, gx1, gy1)
        if not cells: return
        
        # 1. CLEARING (Free Space)
        for gx, gy in cells[:-1]:
            # If map thinks there is a TALL building here (higher than current ray + 1m buffer),
            # we are likely looking "under" a roof (or it's a ghost).
            # To be safe, we only clear if the cell isn't SUPER confident yet.
            if self.height_grid[gy, gx] > z + 1.0 and self.grid[gy, gx] > 5.0:
                continue

            # Decay probability
            self.grid[gy, gx] -= 0.03
            self.grid[gy, gx] = max(self.min_prob, self.grid[gy, gx])
            
            # GHOST CLEARING: If cell becomes "very free", reset height
            # This deletes buildings that don't exist anymore
            if self.grid[gy, gx] < -4.0:
                self.height_grid[gy, gx] = 0.0
                self.hits[gy, gx] = 0
        
        # 2. OCCUPANCY (Hit)
        gx, gy = cells[-1]
        if hit:
            self.grid[gy, gx] += 1.0
            self.grid[gy, gx] = min(15.0, self.grid[gy, gx]) # Cap max confidence
            self.hits[gy, gx] += 1
            
            # Update Height: max of current stored height vs new observation
            self.height_grid[gy, gx] = max(self.height_grid[gy, gx], z + 0.2)
        else:
            # Ray went to max range without hitting anything
            if self.hits[gy, gx] < 5: # Only clear if not fully solidified
                self.grid[gy, gx] -= 0.05

    def get_occupancy_msg(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.size
        msg.info.height = self.size
        msg.info.origin.position.x = self.origin
        msg.info.origin.position.y = self.origin
        
        occ = np.zeros_like(self.grid, dtype=np.int8)
        occ[self.grid > self.occupied_threshold] = 100
        occ[self.grid < self.free_threshold] = 0
        occ[(self.grid >= self.free_threshold) & (self.grid <= self.occupied_threshold)] = -1
        
        msg.data = occ.flatten().tolist()
        return msg

class MappingNode(Node):
    def __init__(self):
        super().__init__("advanced_mapper")
        
        self.mapper = SimpleMapper(resolution=0.1, size=2000)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.has_pose = False
        
        self.bridge = CvBridge()
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position_v1", self.pos_cb, qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos)
        self.create_subscription(Image, "/depth_camera", self.depth_cb, qos)
        
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.viz_pub = self.create_publisher(MarkerArray, "/map_3d", 10)
        self.drone_pub = self.create_publisher(Marker, "/drone_marker", 10)
        
        self.create_timer(0.2, self.timer_cb)
        self.get_logger().info("MAPPER STARTED")

    def pos_cb(self, msg):
        if not msg.xy_valid: return
        self.x = msg.y      # East
        self.y = msg.x      # North  
        self.z = -msg.z     # Up

        if not hasattr(self, "start_time"):
            self.start_time = self.get_clock().now().nanoseconds
        elapsed = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
        if elapsed < 2.0:
            return

        
        if hasattr(msg, 'heading'):
            self.yaw = math.pi/2 - msg.heading
            while self.yaw > math.pi: self.yaw -= 2*math.pi
            while self.yaw < -math.pi: self.yaw += 2*math.pi

            prev_yaw = getattr(self, "prev_yaw", self.yaw)
            dt = 0.05 
            self.angular_speed = (self.yaw - prev_yaw) / dt
            self.prev_yaw = self.yaw
        
        
        self.has_pose = True
        
    
    def scan_cb(self, msg):
        if not self.has_pose: return
        if abs(getattr(self, "angular_speed", 0.0)) > 0.7:
            return
        




        max_range = 30.0
        for i, r in enumerate(msg.ranges):
            yaw_at_ray = self.yaw + getattr(self, "angular_speed", 0.0) * (i / len(msg.ranges)) * 0.05
            angle = yaw_at_ray + msg.angle_min + i * msg.angle_increment
            
            if not np.isfinite(r) or r > max_range:
                ex = self.x + max_range * math.cos(angle)
                ey = self.y + max_range * math.sin(angle)
                self.mapper.add_ray(self.x, self.y, ex, ey, self.z, hit=False)
            elif r > 0.1:
                ex = self.x + r * math.cos(angle)
                ey = self.y + r * math.sin(angle)
                self.mapper.add_ray(self.x, self.y, ex, ey, self.z, hit=True)

    def depth_cb(self, msg):
        if not self.has_pose: return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            H, W = img.shape
            mid_w = int(W / 2)
            
            # Define 5 vertical sample points (Bottom to Top)
            # This helps detect the wall structure from floor to ceiling
            sample_rows = [int(H*0.2), int(H*0.4), int(H*0.6), int(H*0.8)]
            
            for row in sample_rows:
                # Get median distance in a small window
                roi = img[row-2:row+2, mid_w-2:mid_w+2]
                v = roi.flatten()
                v = v[(v > 0.3) & (v < 20.0) & np.isfinite(v)]
                
                if len(v) > 0:
                    dist = float(np.median(v))
                    
                    # Calculate vertical angle (pitch)
                    # Simple pinhole model approximation
                    # Assuming 45 deg V-FOV. Map 0..H to +22.5..-22.5 deg
                    v_angle_deg = 22.5 - (row / H) * 45.0
                    v_angle = math.radians(v_angle_deg)
                    
                    # Calculate hit height
                    hit_z = self.z + (dist * math.tan(v_angle))
                    
                    ex = self.x + dist * math.cos(self.yaw)
                    ey = self.y + dist * math.sin(self.yaw)
                    self.mapper.add_ray(self.x, self.y, ex, ey, hit_z, hit=True)

        except Exception:
            pass

    def timer_cb(self):
        if not self.has_pose: return
        
        self.map_pub.publish(self.mapper.get_occupancy_msg())
        
        # --- RAINBOW VOXEL VISUALIZATION ---
        occupied = self.mapper.grid > 3.0
        rows, cols = np.where(occupied)
        
        if len(rows) > 0:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "voxels"
            marker.id = 0
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            
            block_h = 1.0 # 1 meter blocks
            marker.scale.x = self.mapper.resolution
            marker.scale.y = self.mapper.resolution
            marker.scale.z = block_h 
            
            points = []
            colors = []
            
            for r, c in zip(rows, cols):
                actual_height = self.mapper.height_grid[r, c]
                if actual_height < 0.5: actual_height = 0.5
                
                # Determine color based on total height of the building
                # (You could also color each block differently to make a gradient)
                c_msg = get_color_by_height(actual_height, max_h=15.0)
                
                num_blocks = int(np.ceil(actual_height / block_h))
                base_x = self.mapper.origin + (c + 0.5) * self.mapper.resolution
                base_y = self.mapper.origin + (r + 0.5) * self.mapper.resolution
                
                for i in range(num_blocks):
                    p = Point()
                    p.x = base_x
                    p.y = base_y
                    p.z = (i * block_h) + (block_h / 2.0)
                    points.append(p)
                    
                    # Gradient effect: Darker at bottom, lighter at top? 
                    # Or just uniform building color. Let's use uniform for clarity.
                    colors.append(c_msg)
            
            marker.points = points
            marker.colors = colors
            self.viz_pub.publish(MarkerArray(markers=[marker]))
        
        # Drone marker
        dm = Marker()
        dm.header.frame_id = "map"
        dm.header.stamp = self.get_clock().now().to_msg()
        dm.ns = "drone"
        dm.id = 1
        dm.type = Marker.ARROW
        dm.action = Marker.ADD
        dm.scale.x = 1.0; dm.scale.y = 0.2; dm.scale.z = 0.2
        dm.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        dm.pose.position.x = self.x; dm.pose.position.y = self.y; dm.pose.position.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.yaw)
        dm.pose.orientation.x = qx; dm.pose.orientation.y = qy; dm.pose.orientation.z = qz; dm.pose.orientation.w = qw
        self.drone_pub.publish(dm)

def main():
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
