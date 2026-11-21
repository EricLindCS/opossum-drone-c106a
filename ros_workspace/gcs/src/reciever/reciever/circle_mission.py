#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


# MISSION CONFIGURATION
# All coordinates are ABSOLUTE PX4 local coordinates.
#
# PX4 NED horizontal axes:
#   X = North (meters)
#   Y = East  (meters)
#
# Z is positive UP here (px4 has the opposite)

MISSION_MODE = 'circle'  

CUBE_X = 0.0
CUBE_Y = 0.0
BOTTOM_ALTITUDE = 1.5
CUBE_SIDE = 1.0

TAKEOFF_WAIT = 6.0
TIME_PER_WAYPOINT = 2.9
PUBLISH_RATE = 10.0

CIRCLE_CENTER_X = 0.0
CIRCLE_CENTER_Y = 0.0
CIRCLE_RADIUS = 1.5
CIRCLE_ALTITUDE = 1.5
CIRCLE_PERIOD = 18.0
CIRCLE_DIRECTION = 1.0  # +1 = CCW, -1 = CW


class CubeMission(Node):

    def __init__(self):
        super().__init__('cube_mission')

        self.command_pub = self.create_publisher(
            String,
            '/gcs/drone_1/command',
            10
        )

        self.control_pub = self.create_publisher(
            PoseStamped,
            '/gcs/drone_1/control',
            10
        )

        self.state = 'unknown'

        self.state_sub = self.create_subscription(
            String,
            '/drone_1/state',
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        self.state = msg.data

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

    def send_position(self, x, y, z, yaw=0.0):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        yaw_quat = self.yaw_to_quaternion(yaw)
        msg.pose.orientation.x = yaw_quat[0]
        msg.pose.orientation.y = yaw_quat[1]
        msg.pose.orientation.z = yaw_quat[2]
        msg.pose.orientation.w = yaw_quat[3]

        self.control_pub.publish(msg)

    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return (0.0, 0.0, qz, qw)

    def wait_for_state(self, target, timeout):
        self.get_logger().info(f'Waiting for state: {target}')

        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.state == target:
                self.get_logger().info(f'State confirmed: {target}')
                return True

            if time.monotonic() - start > timeout:
                return False

        return False

    def hold_position(self, x, y, z, duration, yaw=0.0):
        self.get_logger().info(
            f'Waypoint: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Yaw={yaw:.2f} rad'
        )

        start = time.monotonic()

        while rclpy.ok() and time.monotonic() - start < duration:
            self.send_position(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(1.0 / PUBLISH_RATE)

    def run_circle_mission(self):
        self.get_logger().info(
            'Starting circular orbit: radius=%.2f m, alt=%.2f m, period=%.2f s'
            % (CIRCLE_RADIUS, CIRCLE_ALTITUDE, CIRCLE_PERIOD)
        )

        orbit_start = time.monotonic()
        while rclpy.ok():
            if self.state != 'mission':
                self.get_logger().error(
                    f'Drone left mission state: {self.state}'
                )
                return

            elapsed = time.monotonic() - orbit_start
            theta = CIRCLE_DIRECTION * (2.0 * math.pi * elapsed / CIRCLE_PERIOD)
            x = CIRCLE_CENTER_X + CIRCLE_RADIUS * math.cos(theta)
            y = CIRCLE_CENTER_Y + CIRCLE_RADIUS * math.sin(theta)
            z = CIRCLE_ALTITUDE
            yaw = theta + math.pi / 2.0

            self.send_position(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(1.0 / PUBLISH_RATE)

    def run_cube_mission(self):
        self.get_logger().info('Starting cube trajectory!')

        x0 = CUBE_X
        y0 = CUBE_Y
        x1 = CUBE_X + CUBE_SIDE
        y1 = CUBE_Y + CUBE_SIDE

        z0 = BOTTOM_ALTITUDE
        z1 = BOTTOM_ALTITUDE + CUBE_SIDE

        waypoints = [
            (x0, y0, z0),
            (x1, y0, z0),
            (x1, y1, z0),
            (x0, y1, z0),
            (x0, y1, z1),
            (x1, y1, z1),
            (x1, y0, z1),
            (x0, y0, z1),
            (x0, y0, z0),
        ]

        for i, (x, y, z) in enumerate(waypoints):
            if not rclpy.ok():
                return

            if self.state != 'mission':
                self.get_logger().error(
                    f'Drone left mission state: {self.state}'
                )
                return

            self.get_logger().info(
                f'Cube waypoint {i + 1}/{len(waypoints)}'
            )

            self.hold_position(
                x, y, z, TIME_PER_WAYPOINT
            )

    def run_mission(self):
        self.get_logger().info('Waiting for ROS connections...')
        time.sleep(2.0)

        self.send_command('COMMAND_ARM')
        if not self.wait_for_state('armed', 10.0):
            self.get_logger().error('ARM timeout. Mission aborted.')
            return

        self.send_command('COMMAND_MISSION_START')

        if MISSION_MODE == 'circle':
            self.run_circle_mission()
        else:
            self.run_cube_mission()

        self.get_logger().info('Landing.')
        self.send_command('COMMAND_RETURN_HOME')


def main():
    rclpy.init()

    node = CubeMission()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().warning('cancelling.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
