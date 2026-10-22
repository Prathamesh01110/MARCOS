#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge

import numpy as np
import math
from enum import IntEnum
from collections import deque


class FlightPhase(IntEnum):
    INIT = 0
    TAKEOFF = 1
    NAVIGATE = 2
    REACHED = 3


class NavigationMode(IntEnum):
    PATH_FOLLOWING = 0
    EXPLORATION = 1


def point_to_segment_distance(px, py, x1, y1, x2, y2):
    A = px - x1
    B = py - y1
    C = x2 - x1
    D = y2 - y1
    dot = A * C + B * D
    len_sq = C * C + D * D
    if len_sq == 0:
        return math.hypot(px - x1, py - y1)
    u = max(0.0, min(1.0, dot / len_sq))
    x = x1 + u * C
    y = y1 + u * D
    return math.hypot(px - x, py - y)


class SensorFusionModule:
    def __init__(self, logger):
        self.logger = logger
        self.depth_reliability = 0.92
        self.lidar_reliability = 0.65
        self.measurement_noise_depth = 0.2
        self.measurement_noise_lidar = 0.3

    def fuse_distance(self, d_depth, d_lidar):
        d_ok = (d_depth is not None) and (0.1 < d_depth < 30.0)
        l_ok = (d_lidar is not None) and (0.1 < d_lidar < 30.0)

        if d_ok and l_ok:
            w_d = self.depth_reliability / self.measurement_noise_depth
            w_l = self.lidar_reliability / self.measurement_noise_lidar
            fused = (d_depth * w_d + d_lidar * w_l) / (w_d + w_l)
            if abs(d_depth - d_lidar) > 2.0:
                fused = min(d_depth, d_lidar)
            return fused
        if d_ok:
            return d_depth
        if l_ok:
            return d_lidar
        return 20.0

    def update(self, d_f, d_l, d_r, l_f, l_l, l_r):
        return (
            self.fuse_distance(d_f, l_f),
            self.fuse_distance(d_l, l_l),
            self.fuse_distance(d_r, l_r),
        )


class SmartObstacleNavigator(Node):
    def __init__(self):
        super().__init__("smart_obstacle_navigator")

        self.declare_parameters("", [("flight_altitude", 2.0)])
        self.flight_altitude = float(self.get_parameter("flight_altitude").value)

        # Drone-specific physical constraints (x500 model)
        self.MIN_SAFE_CLEARANCE = 0.7  # meters - based on 0.5m rotor span + safety
        self.CORRIDOR_THRESHOLD = 3.0   # consider corridor if both sides < 3m

        self.waypoints = []
        self.wp_index = 0
        self.manual_goal = False
        self.goal_pose = None
        self.nav_mode = NavigationMode.PATH_FOLLOWING
        self.last_path_hash = None

        self.phase = FlightPhase.INIT
        self.armed = False
        self.have_local_position = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.offboard_counter = 0

        self.raw_depth = {"f": None, "l": None, "r": None}
        self.raw_lidar = {"f": None, "l": None, "r": None}
        self.last_depth_time = 0.0
        self.last_lidar_time = 0.0

        self.fused_front = 20.0
        self.fused_left = 20.0
        self.fused_right = 20.0
        self.fusion = SensorFusionModule(self.get_logger())

        self.filtered_center_err = 0.0
        self.prev_yaw = 0.0
        
        self.current_forward_vel = 0.0
        self.current_side_vel = 0.0
        
        self.stuck_counter = 0
        self.position_history = deque(maxlen=50)
        self.last_waypoint_time = 0.0
        
        # Enhanced features
        self.consecutive_failures = 0
        self.last_progress_check = 0.0
        self.last_progress_distance = 0.0
        self.yaw_history = deque(maxlen=20)
        self.lateral_history = deque(maxlen=20)
        
        # Wide space handling
        self.last_significant_clearance = 0.0

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_viz = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", qos_cmd
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_cmd
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", qos_cmd
        )
        self.viz_path_pub = self.create_publisher(Path, "/planned_path", qos_viz)

        self.create_subscription(
            VehicleLocalPosition, "fmu/out/vehicle_local_position_v1", self.pos_cb, qos_cmd
        )
        self.create_subscription(
            VehicleStatus, "fmu/out/vehicle_status_v4", self.status_cb, qos_cmd
        )
        self.create_subscription(Image, "/depth_camera", self.depth_cb, qos_sensor)
        self.create_subscription(LaserScan, "/scan", self.lidar_cb, qos_sensor)
        self.create_subscription(Path, "/global_path", self.path_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Navigator v10.0 Ready - x500 Optimized (clearance: {self.MIN_SAFE_CLEARANCE}m)")

    def get_lookahead_target(self, lookahead_dist=2.5):
        if self.wp_index >= len(self.waypoints):
            return None
        
        min_clearance = min(self.fused_left, self.fused_right, self.fused_front)
        if min_clearance < 2.0:
            lookahead_dist = 1.5
        elif self.current_forward_vel > 7.0:
            lookahead_dist = 3.5
        
        accumulated_dist = 0.0
        px, py = self.current_x, self.current_y
        
        for i in range(self.wp_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            seg_dist = math.hypot(wx - px, wy - py)
            
            if accumulated_dist + seg_dist >= lookahead_dist:
                remaining = lookahead_dist - accumulated_dist
                if seg_dist > 0:
                    t = remaining / seg_dist
                    return (px + t * (wx - px), py + t * (wy - py))
                return (wx, wy)
            
            accumulated_dist += seg_dist
            px, py = wx, wy
        
        return self.waypoints[-1]

    def is_oscillating(self):
        if len(self.lateral_history) < 15:
            return False
        
        direction_changes = 0
        for i in range(1, len(self.lateral_history)):
            if self.lateral_history[i] * self.lateral_history[i-1] < 0:
                direction_changes += 1
        
        return direction_changes > 8

    def is_stuck(self):
        if len(self.position_history) < 30:
            return False
        
        positions = list(self.position_history)
        recent = positions[-10:]
        older = positions[-30:-20]
        
        recent_center = (sum(p[0] for p in recent) / len(recent),
                        sum(p[1] for p in recent) / len(recent))
        older_center = (sum(p[0] for p in older) / len(older),
                       sum(p[1] for p in older) / len(older))
        
        movement = math.hypot(recent_center[0] - older_center[0],
                             recent_center[1] - older_center[1])
        return movement < 0.5

    def publish_visual_path(self):
        if not self.waypoints:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x, y in self.waypoints[self.wp_index:]:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = self.flight_altitude
            msg.poses.append(p)
        self.viz_path_pub.publish(msg)

    def path_cb(self, msg: Path):
        new_wps = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not new_wps:
            if self.goal_pose and self.phase == FlightPhase.NAVIGATE:
                goal_dist = math.hypot(
                    self.goal_pose[0] - self.current_x,
                    self.goal_pose[1] - self.current_y
                )
                if goal_dist > 1.0:
                    self.nav_mode = NavigationMode.EXPLORATION
                    self.get_logger().warn("No path - EXPLORATION mode")
            return
        
        path_hash = hash(tuple(new_wps))
        if path_hash == self.last_path_hash:
            return
        self.last_path_hash = path_hash
        
        if self.goal_pose:
            path_length = sum(math.hypot(new_wps[i+1][0] - new_wps[i][0],
                                        new_wps[i+1][1] - new_wps[i][1])
                             for i in range(len(new_wps) - 1))
            
            direct_dist = math.hypot(
                self.goal_pose[0] - self.current_x,
                self.goal_pose[1] - self.current_y
            )
            
            if direct_dist > 5.0 and path_length > direct_dist * 1.8:
                self.nav_mode = NavigationMode.EXPLORATION
                self.get_logger().warn(f"Path inefficient - EXPLORING")
                return
        
        self.nav_mode = NavigationMode.PATH_FOLLOWING
        
        if self.manual_goal:
            self.manual_goal = False
        
        if self.waypoints and self.phase == FlightPhase.NAVIGATE:
            old_target = self.waypoints[min(self.wp_index, len(self.waypoints)-1)]
            dists = [math.hypot(wx - old_target[0], wy - old_target[1]) for wx, wy in new_wps]
            new_idx = int(np.argmin(dists))
            
            if dists[new_idx] < 5.0:
                self.waypoints = new_wps
                self.wp_index = new_idx
            else:
                self.waypoints = new_wps
                dists = [math.hypot(wx - self.current_x, wy - self.current_y) for wx, wy in new_wps]
                self.wp_index = int(np.argmin(dists))
        else:
            self.waypoints = new_wps
            dists = [math.hypot(wx - self.current_x, wy - self.current_y) for wx, wy in new_wps]
            self.wp_index = int(np.argmin(dists))
        
        self.publish_visual_path()
        if self.phase == FlightPhase.REACHED and abs(self.current_z - self.flight_altitude) < 0.5:
            self.phase = FlightPhase.NAVIGATE

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.waypoints = [self.goal_pose]
        self.wp_index = 0
        self.manual_goal = True
        self.phase = FlightPhase.NAVIGATE
        self.nav_mode = NavigationMode.EXPLORATION
        self.publish_visual_path()
        self.get_logger().info(f"Goal: ({self.goal_pose[0]:.1f}, {self.goal_pose[1]:.1f})")

    def pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = -msg.z
        self.current_yaw = msg.heading
        self.have_local_position = True
        self.position_history.append((self.current_x, self.current_y))

    def status_cb(self, msg: VehicleStatus):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def depth_cb(self, msg: Image):
        self.last_depth_time = self.get_clock().now().nanoseconds / 1e9
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            H, W = img.shape
            l, r = int(W * 0.3), int(W * 0.7)
            h1, h2 = int(H * 0.35), int(H * 0.65)
            def ext(roi):
                flat = roi.flatten()
                v = flat[(flat > 0.1) & (flat < 30.0) & np.isfinite(flat)]
                return float(np.percentile(v, 20)) if v.size > 30 else None
            self.raw_depth["l"] = ext(img[h1:h2, :l])
            self.raw_depth["f"] = ext(img[h1:h2, l:r])
            self.raw_depth["r"] = ext(img[h1:h2, r:])
            self.update_fusion()
        except Exception:
            pass

    def lidar_cb(self, msg: LaserScan):
        self.last_lidar_time = self.get_clock().now().nanoseconds / 1e9
        try:
            ranges = np.array(msg.ranges)
            def sector(angle_rad, width_deg=35.0):
                width = math.radians(width_deg)
                i_min = int((angle_rad - width - msg.angle_min) / msg.angle_increment)
                i_max = int((angle_rad + width - msg.angle_min) / msg.angle_increment)
                i_min = max(0, i_min)
                i_max = min(len(ranges), i_max)
                seg = ranges[i_min:i_max]
                v = seg[(seg > 0.1) & (seg < 30.0) & np.isfinite(seg)]
                return float(np.percentile(v, 10)) if v.size > 5 else None
            self.raw_lidar["f"] = sector(0.0)
            self.raw_lidar["l"] = sector(math.radians(90))
            self.raw_lidar["r"] = sector(math.radians(-90))
            self.update_fusion()
        except Exception:
            pass

    def update_fusion(self):
        now = self.get_clock().now().nanoseconds / 1e9
        d = self.raw_depth if now - self.last_depth_time <= 1.0 else {"f": None, "l": None, "r": None}
        l = self.raw_lidar if now - self.last_lidar_time <= 1.0 else {"f": None, "l": None, "r": None}
        self.fused_front, self.fused_left, self.fused_right = self.fusion.update(
            d["f"], d["l"], d["r"], l["f"], l["l"], l["r"]
        )

    def control_loop(self):
        if not self.have_local_position:
            return
        
        dt = 0.1
        self.pub_offboard_mode()
        
        if not self.armed and self.phase != FlightPhase.INIT:
            self.phase = FlightPhase.INIT
            self.offboard_counter = 0
            self.current_forward_vel = 0.0
            self.current_side_vel = 0.0

        if self.phase == FlightPhase.INIT:
            self.pub_setpoint(self.current_x, self.current_y, self.current_z, float('nan'))
            if self.offboard_counter == 20:
                self.pub_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
                self.get_logger().info("OFFBOARD mode")
            if self.offboard_counter == 25:
                self.pub_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, p1=1.0)
            if self.offboard_counter == 50:
                self.pub_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
            if self.armed and self.offboard_counter > 60:
                self.phase = FlightPhase.TAKEOFF
                self.get_logger().info("ARMED - Takeoff")
            self.offboard_counter += 1
            return

        if self.phase == FlightPhase.TAKEOFF:
            self.pub_setpoint(self.current_x, self.current_y, self.flight_altitude, float('nan'))
            if abs(self.current_z - self.flight_altitude) < 0.3:
                self.phase = FlightPhase.NAVIGATE
                self.prev_yaw = self.current_yaw
                self.last_waypoint_time = self.get_clock().now().nanoseconds / 1e9
            return

        # NAVIGATION
        if self.nav_mode == NavigationMode.EXPLORATION:
            if not self.goal_pose:
                self.pub_setpoint(self.current_x, self.current_y, self.flight_altitude, self.prev_yaw)
                return
            
            tx, ty = self.goal_pose
            dx = tx - self.current_x
            dy = ty - self.current_y
            dist = math.hypot(dx, dy)
            
            if dist < 1.0:
                self.phase = FlightPhase.REACHED
                self.get_logger().info("✓ Goal reached!")
                self.pub_setpoint(self.current_x, self.current_y, self.flight_altitude, self.prev_yaw)
                return
            
            dir_x = dx / dist if dist > 0.1 else 0.0
            dir_y = dy / dist if dist > 0.1 else 0.0
            dx_look, dy_look = dx, dy
            
            if self.offboard_counter % 20 == 0:
                self.get_logger().info(f"EXPLORE dist:{dist:.1f}m")
        
        else:  # PATH_FOLLOWING
            if self.wp_index >= len(self.waypoints):
                if self.goal_pose:
                    goal_dist = math.hypot(self.goal_pose[0] - self.current_x,
                                          self.goal_pose[1] - self.current_y)
                    if goal_dist > 1.0:
                        self.nav_mode = NavigationMode.EXPLORATION
                        self.get_logger().warn("→ EXPLORATION")
                        return
                
                self.pub_setpoint(self.current_x, self.current_y, self.flight_altitude, self.prev_yaw)
                return

            tx, ty = self.waypoints[self.wp_index]
            dx = tx - self.current_x
            dy = ty - self.current_y
            dist = math.hypot(dx, dy)

            lookahead = self.get_lookahead_target()
            if lookahead and dist > 1.5:
                dx_look = lookahead[0] - self.current_x
                dy_look = lookahead[1] - self.current_y
            else:
                dx_look, dy_look = dx, dy

            dist_look = math.hypot(dx_look, dy_look)
            dir_x = dx_look / dist_look if dist_look > 0.1 else 0.0
            dir_y = dy_look / dist_look if dist_look > 0.1 else 0.0

        # YAW
        yaw_target = math.atan2(dx_look, dy_look)
        self.prev_yaw = 0.8 * self.prev_yaw + 0.2 * yaw_target
        yaw_goal = self.prev_yaw
        self.yaw_history.append(self.prev_yaw)

        # SPEED - Optimized for x500 drone dimensions
        MIN_SAFE_CLEARANCE = self.MIN_SAFE_CLEARANCE
        CORRIDOR_THRESHOLD = self.CORRIDOR_THRESHOLD
        
        is_corridor = (self.fused_left < CORRIDOR_THRESHOLD and self.fused_right < CORRIDOR_THRESHOLD)
        is_wide_open = (self.fused_left > 5.0 and self.fused_right > 5.0)
        
        speed_mult = 0.85 if self.nav_mode == NavigationMode.EXPLORATION else 1.0
        
        if is_wide_open:
            # Wide open space - faster, more direct
            if self.fused_front > 5.0: base_speed = 10.0 * speed_mult
            elif self.fused_front > 3.0: base_speed = 8.0 * speed_mult
            else: base_speed = 6.0 * speed_mult
        elif is_corridor:
            # Corridor navigation - check if passage is wide enough
            min_side_clearance = min(self.fused_left, self.fused_right)
            
            if min_side_clearance < MIN_SAFE_CLEARANCE:
                # Too narrow - very slow and careful
                base_speed = 2.0 * speed_mult
            elif min_side_clearance < 1.0:
                # Tight but passable
                if self.fused_front > 3.0: base_speed = 6.0 * speed_mult
                elif self.fused_front > 2.0: base_speed = 4.5 * speed_mult
                else: base_speed = 3.0 * speed_mult
            else:
                # Good corridor clearance
                if self.fused_front > 3.0: base_speed = 9.0 * speed_mult
                elif self.fused_front > 2.0: base_speed = 7.5 * speed_mult
                else: base_speed = 5.5 * speed_mult
        else:
            # Mixed clearance - one side more open
            min_clear = min(self.fused_left, self.fused_right)
            if min_clear < MIN_SAFE_CLEARANCE: 
                base_speed = 3.5 * speed_mult  # Too close to wall
            elif min_clear < 1.2: 
                base_speed = 7.0 * speed_mult
            else: 
                base_speed = 9.5 * speed_mult

        heading_err = abs(math.atan2(dy_look, dx_look))
        if heading_err < math.radians(15): base_speed *= 1.0
        elif heading_err < math.radians(45): base_speed *= 0.8
        else: base_speed *= 0.5

        stopping_dist = 0.4 * self.current_forward_vel + 0.4
        if self.fused_front < stopping_dist + 1.2:
            factor = (self.fused_front - 1.0) / (stopping_dist + 0.5)
            base_speed *= max(0.0, min(1.0, factor))
        
        # Safety margins based on drone size
        if self.fused_front < 1.0: base_speed *= 0.3
        if self.fused_front < 0.7: base_speed = 0.0

        max_accel = 3.5
        vel_diff = base_speed - self.current_forward_vel
        vel_diff = max(min(vel_diff, max_accel * dt), -max_accel * dt)
        self.current_forward_vel += vel_diff
        forward_speed = self.current_forward_vel

        # LATERAL - Account for drone width
        repulsion = 0.0
        
        if is_corridor:
            balance = self.fused_left - self.fused_right
            
            # If either side too close, stronger repulsion
            if self.fused_left < MIN_SAFE_CLEARANCE or self.fused_right < MIN_SAFE_CLEARANCE:
                repulsion = max(min(balance * 1.2, 2.5), -2.5)
            else:
                repulsion = max(min(balance * 0.8, 1.5), -1.5)
        elif is_wide_open:
            # In wide open spaces: strongly bias toward goal direction
            if self.goal_pose:
                dx_g = self.goal_pose[0] - self.current_x
                dy_g = self.goal_pose[1] - self.current_y
                goal_angle = math.atan2(dy_g, dx_g)
                heading_diff = goal_angle - self.current_yaw
                # Normalize to [-pi, pi]
                while heading_diff > math.pi:
                    heading_diff -= 2 * math.pi
                while heading_diff < -math.pi:
                    heading_diff += 2 * math.pi
                
                # Strong lateral correction toward goal (max 3 m/s)
                repulsion = 3.0 * math.sin(heading_diff)
        else:
            # Open or mixed space - avoid walls based on drone size
            if self.fused_left < MIN_SAFE_CLEARANCE:
                repulsion -= (MIN_SAFE_CLEARANCE - self.fused_left) * 2.0
            if self.fused_right < MIN_SAFE_CLEARANCE:
                repulsion += (MIN_SAFE_CLEARANCE - self.fused_right) * 2.0
        
        wall_slide = 0.0
        if self.fused_front < 2.5 and not is_wide_open:
            clear_diff = self.fused_left - self.fused_right
            slide_int = 1.0 - (self.fused_front / 2.5)
            
            if abs(clear_diff) > 0.3:
                wall_slide = 4.0 * slide_int * (1 if clear_diff > 0 else -1)
                if self.fused_front < 1.2:
                    wall_slide *= 1.5
            else:
                if self.goal_pose:
                    dx_g = self.goal_pose[0] - self.current_x
                    dy_g = self.goal_pose[1] - self.current_y
                    cross = dx_g * math.cos(self.current_yaw) - dy_g * math.sin(self.current_yaw)
                    wall_slide = 3.5 * slide_int * (1 if cross > 0 else -1)

        target_side = max(min(repulsion + wall_slide, 5.0), -5.0)
        self.current_side_vel = 0.75 * self.current_side_vel + 0.25 * target_side
        lateral_speed = self.current_side_vel
        
        self.lateral_history.append(lateral_speed)
        if self.is_oscillating():
            lateral_speed *= 0.5

        if self.fused_front < 0.7:  # Emergency reverse based on drone safety
            forward_speed = -0.8
            lateral_speed = 3.0 if self.fused_left > self.fused_right else -3.0

        # PROGRESS CHECK
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_progress_check > 5.0:
            if self.goal_pose:
                curr_dist = math.hypot(self.goal_pose[0] - self.current_x,
                                      self.goal_pose[1] - self.current_y)
                
                if self.last_progress_distance > 0 and abs(curr_dist - self.last_progress_distance) < 0.5:
                    self.consecutive_failures += 1
                    
                    if self.consecutive_failures >= 3:
                        if self.nav_mode == NavigationMode.PATH_FOLLOWING:
                            self.nav_mode = NavigationMode.EXPLORATION
                            self.get_logger().warn("No progress → EXPLORATION")
                        else:
                            if self.wp_index < len(self.waypoints) - 1:
                                self.wp_index += 1
                        self.consecutive_failures = 0
                else:
                    self.consecutive_failures = 0
                
                self.last_progress_distance = curr_dist
            self.last_progress_check = now
        
        # STUCK CHECK - Account for drone size (needs 0.7m clearance minimum)
        # Only truly trapped if all clearances below safe threshold AND not moving
        is_trapped = (self.fused_front < MIN_SAFE_CLEARANCE and 
                     self.fused_left < MIN_SAFE_CLEARANCE and 
                     self.fused_right < MIN_SAFE_CLEARANCE and 
                     forward_speed < 0.5)
        
        # Also detect "lost in open space" - not making progress in wide areas
        is_lost_in_open = (is_wide_open and self.consecutive_failures >= 2)
        
        if is_lost_in_open and self.goal_pose:
            # Force aggressive turn toward goal
            dx_g = self.goal_pose[0] - self.current_x
            dy_g = self.goal_pose[1] - self.current_y
            goal_yaw = math.atan2(dy_g, dx_g)
            
            # Override yaw to point directly at goal
            yaw_goal = goal_yaw
            forward_speed = 3.0  # Moderate speed while reorienting
            lateral_speed = 0.0
            
            self.get_logger().warn("Lost in open space - reorienting to goal", throttle_duration_sec=1.0)
        
        if is_trapped:
            self.stuck_counter += 1  # Slower escalation
            if self.stuck_counter > 20:  # Give more time before backing up
                forward_speed = -1.5
                lateral_speed = 0.0
                
                if self.stuck_counter > 40:  # Double the threshold before giving up
                    if self.nav_mode == NavigationMode.PATH_FOLLOWING:
                        self.wp_index += 1
                        self.stuck_counter = 0
                        self.publish_visual_path()
                        if self.wp_index >= len(self.waypoints):
                            if self.goal_pose:
                                self.nav_mode = NavigationMode.EXPLORATION
                            else:
                                self.phase = FlightPhase.REACHED
                        return
                    else:
                        self.phase = FlightPhase.REACHED
                        self.get_logger().error("Truly trapped - giving up")
                        return
        elif self.is_stuck() and self.nav_mode == NavigationMode.PATH_FOLLOWING:
            self.stuck_counter += 1
            if self.stuck_counter > 20:
                self.wp_index += 1
                self.stuck_counter = 0
                self.publish_visual_path()
                if self.wp_index >= len(self.waypoints):
                    self.phase = FlightPhase.REACHED
                return
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)

        # COMPUTE POSITION
        tan_x, tan_y = -dir_y, dir_x
        nx = self.current_x + (dir_x * forward_speed + tan_x * lateral_speed) * dt
        ny = self.current_y + (dir_y * forward_speed + tan_y * lateral_speed) * dt
        nz = self.flight_altitude

        # WAYPOINT PROGRESSION
        if self.nav_mode == NavigationMode.PATH_FOLLOWING and dist < 1.0:
            self.wp_index += 1
            self.publish_visual_path()
            if self.wp_index >= len(self.waypoints):
                self.phase = FlightPhase.REACHED

        self.pub_setpoint(nx, ny, nz, yaw_goal)
        self.offboard_counter += 1

    def pub_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def pub_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [y, x, -z]
        msg.yaw = yaw
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def pub_command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SmartObstacleNavigator()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Crashed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
