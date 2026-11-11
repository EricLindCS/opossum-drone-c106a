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
            '/child/local_position',
            self.position_callback,
            10,
        )

        self.attitude_sub = self.create_subscription(
            Vector3,
            '/child/attitude',
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
            return

        x, y, z = self._position
        roll, pitch, yaw = self._attitude

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        qx, qy, qz, qw = self.quaternion_from_rpy(roll, pitch, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)


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
