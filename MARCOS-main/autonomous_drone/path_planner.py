#!/usr/bin/env python3

"""
Dynamic Optimistic A* Planner
- FIXED: Starts from drone's actual coordinates
- Treats UNKNOWN space as TRAVERSABLE (Cost = 3.0)
- Replans every 0.5s to adapt to new map data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from px4_msgs.msg import VehicleLocalPosition

import numpy as np
import heapq
import math
from scipy import ndimage
from scipy.interpolate import splprep, splev


class DynamicPlanner(Node):
    def __init__(self):
        super().__init__("dynamic_planner")
        
        # --- TUNING PARAMETERS ---
        self.declare_parameters("", [
            ("safety_radius", 0.6),
            ("unknown_cost", 3.0),
            ("replan_rate", 2.0)
        ])
        
        self.safety_radius = float(self.get_parameter("safety_radius").value)
        self.unknown_cost = float(self.get_parameter("unknown_cost").value)
        self.replan_rate = float(self.get_parameter("replan_rate").value)
        
        self.map_data = None
        self.map_info = None
        self.costmap = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.has_map = False
        
        # QoS (Must match Mapper)
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # QoS for PX4 topics (BEST_EFFORT like in your avoidance node)
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Subscribers
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, qos_map)
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position_v1", self.pos_cb, qos_px4)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        # Publishers
        self.global_path_pub = self.create_publisher(Path, "/global_path", 10)
        self.path_viz_pub = self.create_publisher(Path, "/planned_path", 10)
        self.debug_pub = self.create_publisher(MarkerArray, "/planner_debug", 10)

        # Replan Timer
        self.timer = self.create_timer(1.0 / self.replan_rate, self.replan_loop)
        
        self.get_logger().info(f"🧠 DYNAMIC PLANNER ONLINE | Unknown Cost: {self.unknown_cost}")

    def pos_cb(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x

    def map_cb(self, msg):
        self.map_info = msg.info
        raw = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_data = raw
        
        if not self.has_map:
            self.get_logger().info(f"🗺️ Map Received: {msg.info.width}x{msg.info.height}")
            self.get_logger().info(f"Map Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})")
            self.get_logger().info(f"Resolution: {msg.info.resolution:.3f}m")
        
        self.has_map = True
        self.generate_costmap()

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"📍 New Goal: ({self.goal_x:.1f}, {self.goal_y:.1f})")
        # Immediate replan on new goal
        self.replan_loop()

    def generate_costmap(self):
        if self.map_data is None: return
        
        # 1. Identify Obstacles (Strict 100)
        obstacles = np.zeros_like(self.map_data, dtype=np.uint8)
        obstacles[self.map_data >= 50] = 1 
        
        # 2. Identify Unknown Space (-1)
        unknowns = (self.map_data == -1)
        
        # 3. Inflate Obstacles (Safety Radius)
        inflation_cells = int(self.safety_radius / self.map_info.resolution)
        dist_grid = ndimage.distance_transform_edt(1 - obstacles)
        
        # 4. Build Costmap (Float)
        self.costmap = np.ones_like(self.map_data, dtype=np.float32)
        
        # Apply Unknown Cost
        self.costmap[unknowns] = self.unknown_cost 
        
        # Apply Inflation (Gradient)
        mask_inflate = (dist_grid < inflation_cells)
        inflation_cost = 100.0 * (1.0 - dist_grid[mask_inflate] / inflation_cells)
        self.costmap[mask_inflate] += inflation_cost
        
        # Hard Obstacles are effectively infinite
        self.costmap[obstacles == 1] = 999.0

    def replan_loop(self):
        """Called periodically to update path based on new map data"""
        if not self.has_map or self.goal_x is None: return
        
        # Check if we reached goal
        dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        if dist < 0.5:
            self.goal_x = None
            self.get_logger().info("🏁 Goal Reached!")
            return

        # Plan from current position to goal
        path_grid = self.run_astar(self.current_x, self.current_y, self.goal_x, self.goal_y)
        
        if path_grid:
            self.publish_path(path_grid)
        else:
            self.get_logger().warn("❌ No path found!")

    def run_astar(self, sx, sy, gx, gy):
        """
        A* from (sx, sy) to (gx, gy) in WORLD coordinates
        """
        # Convert world coordinates to grid coordinates
        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)
        
        # Verify conversion
        if not start:
            self.get_logger().error(f"Start position ({sx:.2f}, {sy:.2f}) is outside map bounds!")
            return None
        if not goal:
            self.get_logger().error(f"Goal position ({gx:.2f}, {gy:.2f}) is outside map bounds!")
            return None
        
        # Check if goal is valid
        if not self.is_valid(goal):
            self.get_logger().warn(f"Goal is in obstacle! Searching for nearest free cell...")
            # Try to find nearest free cell to goal
            goal = self.find_nearest_free(goal)
            if not goal:
                self.get_logger().error("Cannot find free cell near goal!")
                return None

        open_set = []
        heapq.heappush(open_set, (0.0, 0, start))
        came_from = {}
        g_score = {start: 0.0}
        
        # 8-Connected Grid
        movements = [
            (0,1,1), (1,0,1), (0,-1,1), (-1,0,1), 
            (1,1,1.414), (1,-1,1.414), (-1,-1,1.414), (-1,1,1.414)
        ]
        
        count = 0
        limit = 40000
        
        while open_set:
            if count > limit: 
                self.get_logger().warn("A* iteration limit reached!")
                return None
            count += 1
            
            _, _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = [goal]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for dx, dy, dist_cost in movements:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Bounds check
                if not (0 <= neighbor[0] < self.map_info.width and 
                       0 <= neighbor[1] < self.map_info.height):
                    continue
                
                # Get cell cost
                cell_cost = self.costmap[neighbor[1], neighbor[0]]
                
                # If cost is too high (Obstacle), skip
                if cell_cost > 200.0: continue
                
                new_g = g_score[current] + (dist_cost * cell_cost)
                
                if neighbor not in g_score or new_g < g_score[neighbor]:
                    g_score[neighbor] = new_g
                    # Heuristic: Euclidean
                    h = math.hypot(neighbor[0]-goal[0], neighbor[1]-goal[1])
                    heapq.heappush(open_set, (new_g + h, count, neighbor))
                    came_from[neighbor] = current
        
        return None

    def find_nearest_free(self, grid_pt, max_search=20):
        """Find nearest free cell to a point"""
        gx, gy = grid_pt
        for r in range(1, max_search):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if (0 <= nx < self.map_info.width and 
                        0 <= ny < self.map_info.height):
                        if self.costmap[ny, nx] < 50.0:
                            return (nx, ny)
        return None

    def publish_path(self, grid_path):
        """Convert grid path to world coordinates and publish"""
        world_path = [self.grid_to_world(gx, gy) for gx, gy in grid_path]
        
        # Simple Smoothing
        if len(world_path) > 3:
            try:
                pts = np.array(world_path)
                tck, u = splprep([pts[:,0], pts[:,1]], s=0.5, k=2) 
                u_new = np.linspace(0, 1, len(world_path))
                x_new, y_new = splev(u_new, tck)
                world_path = list(zip(x_new, y_new))
            except: 
                pass

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for wx, wy in world_path:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = wx, wy
            p.pose.position.z = 2.0
            msg.poses.append(p)
            
        self.global_path_pub.publish(msg)
        self.path_viz_pub.publish(msg)

    # --- HELPERS ---
    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid coordinates"""
        if not self.map_info: 
            return None
        
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        
        if 0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height:
            return (gx, gy)
        
        # DEBUG
        self.get_logger().warn(
            f"Point ({wx:.2f}, {wy:.2f}) -> Grid ({gx}, {gy}) "
            f"is outside bounds [0-{self.map_info.width}, 0-{self.map_info.height}]"
        )
        return None

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates (cell center)"""
        wx = (gx * self.map_info.resolution) + self.map_info.origin.position.x + (self.map_info.resolution / 2)
        wy = (gy * self.map_info.resolution) + self.map_info.origin.position.y + (self.map_info.resolution / 2)
        return wx, wy

    def is_valid(self, grid_pt):
        """Check if grid point is traversable"""
        if not grid_pt: return False
        c = self.costmap[grid_pt[1], grid_pt[0]]
        return c < 200.0


def main():
    rclpy.init()
    node = DynamicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()