#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3


class DronePoseVisualizer(Node):
    def __init__(self):
        super().__init__('drone_pose_visualizer')

        self._position = None
        self._attitude = None

        self.pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)

        self.position_sub = self.create_subscription(
            Vector3,
            '/drone_1/position',
            self.position_callback,
            10,
        )

        self.attitude_sub = self.create_subscription(
            Vector3,
            '/drone_1/attitude',
            self.attitude_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.publish_pose)

    def position_callback(self, msg):
        self._position = (float(msg.x), float(msg.y), float(msg.z))

    def attitude_callback(self, msg):
        self._attitude = (float(msg.x), float(msg.y), float(msg.z))

    def quaternion_from_rpy(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return (qx, qy, qz, qw)

    def publish_pose(self):
        if self._position is None or self._attitude is None:
            self.get_logger().warn('Waiting for position and attitude data')
            return

        x, y, z = self._position
        roll, pitch, yaw = self._attitude

        now = self.get_clock().now().to_msg()

        # Create pose.
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        qx, qy, qz, qw = self.quaternion_from_rpy(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(f'Publishing pose: {pose}')

        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = DronePoseVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
