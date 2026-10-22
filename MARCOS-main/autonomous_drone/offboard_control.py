#!/usr/bin/env python3
"""Always-stream offboard: mode+setpoints at 20 Hz from startup, never stops.
Engages OFFBOARD after 1 s of streaming, arms after 2 s, climbs to hover_alt.
Uses 'desired_velocity' (Twist, world ENU) from depth_avoidance when available."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleOdometry)


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.declare_parameter('hover_alt', 3.0)
        self.declare_parameter('vel_topic', 'desired_velocity')
        self.hover_alt = float(self.get_parameter('hover_alt').value)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        # RELATIVE names -> /drone0/fmu/... etc.
        self.pub_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)
        self.create_subscription(VehicleOdometry, 'fmu/out/vehicle_odometry', self.odom_cb, qos)
        self.create_subscription(Twist, self.get_parameter('vel_topic').value, self.vel_cb, 10)

        self.odom, self.vel, self.vel_time = None, None, 0
        self.n = 0
        self.create_timer(0.05, self.loop)   # 20 Hz, unconditional
        self.get_logger().info('Always-stream offboard online')

    def odom_cb(self, m): self.odom = m
    def vel_cb(self, m):
        self.vel = m
        self.vel_time = self.get_clock().now().nanoseconds

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = float(command), float(p1), float(p2)
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(m)

    def loop(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        self.n += 1
        fresh_vel = self.vel is not None and (now_us * 1000 - self.vel_time) < 500_000_000

        mode = OffboardControlMode()          # heartbeat — publishes EVERY cycle
        mode.position, mode.velocity = not fresh_vel, fresh_vel
        mode.acceleration = mode.attitude = mode.body_rate = False
        mode.timestamp = now_us
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        if fresh_vel:                          # Twist ENU-world -> PX4 NED
            sp.position = [float('nan')] * 3
            sp.velocity = [self.vel.linear.y, self.vel.linear.x, -self.vel.linear.z]
        else:                                  # hold/takeoff target
            sp.position = [0.0, 0.0, -self.hover_alt]
            sp.velocity = [float('nan')] * 3
        self.pub_sp.publish(sp)

        if self.n == 25:
            self.get_logger().info('Engaging OFFBOARD')
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.n == 50:
            self.get_logger().info('Arm command sent')
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)


def main():
    rclpy.init(); rclpy.spin(OffboardControl()); rclpy.shutdown()

if __name__ == '__main__':
    main()