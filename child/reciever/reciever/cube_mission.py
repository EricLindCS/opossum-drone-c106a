
#!/usr/bin/env python3

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

CUBE_X = 0.0       # Absolute north coordinate of first corner
CUBE_Y = 0.0       # Absolute east coordinate of first corner

BOTTOM_ALTITUDE = 1.5
CUBE_SIDE = 1.0

TAKEOFF_WAIT = 6.0
TIME_PER_WAYPOINT = 2.9
PUBLISH_RATE = 10.0


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

    def send_position(self, x, y, z):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        # Identity quaternion: yaw = 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.control_pub.publish(msg)

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

    def hold_position(self, x, y, z, duration):
        self.get_logger().info(
            f'Waypoint: X={x:.2f}, Y={y:.2f}, Z={z:.2f}'
        )

        start = time.monotonic()

        while rclpy.ok() and time.monotonic() - start < duration:
            self.send_position(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(1.0 / PUBLISH_RATE)

    def run_mission(self):

        # Wait for ROS communication to initialize
        self.get_logger().info('Waiting for ROS connections...')
        time.sleep(2.0)

        # 1. ARM / prepare PX4
        self.send_command('COMMAND_ARM')

        if not self.wait_for_state('armed', 10.0):
            self.get_logger().error('ARM timeout. Mission aborted.')
            return

        # 2. Start takeoff
        self.send_command('COMMAND_MISSION_START')

        self.get_logger().info('Taking off...')
        start = time.monotonic()

        while rclpy.ok() and time.monotonic() - start < TAKEOFF_WAIT:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 3. Define the eight cube corners.
        x0 = CUBE_X
        y0 = CUBE_Y
        x1 = CUBE_X + CUBE_SIDE
        y1 = CUBE_Y + CUBE_SIDE

        z0 = BOTTOM_ALTITUDE
        z1 = BOTTOM_ALTITUDE + CUBE_SIDE

        waypoints = [
            (x0, y0, z0),  # A: bottom front-left
            (x1, y0, z0),  # B
            (x1, y1, z0),  # C
            (x0, y1, z0),  # D
            (x0, y1, z1),  # E: climb
            (x1, y1, z1),  # F
            (x1, y0, z1),  # G
            (x0, y0, z1),  # H
            (x0, y0, z0),  # A: descend
        ]

        # 4. Trace cube
        self.get_logger().info('Starting cube trajectory!')

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

        # 5. Return home and land
        self.get_logger().info('Landing.')
        self.send_command('COMMAND_RETURN_HOME')


def main():
    rclpy.init()

    node = CubeMission()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().warning(
            'cancelling.'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
