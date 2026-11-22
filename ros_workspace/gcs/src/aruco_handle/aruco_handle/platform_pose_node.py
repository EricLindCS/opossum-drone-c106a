#!/usr/bin/env python3

import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


# ==================
# Config
# ==================

# Drone name
DRONE_NAME = "drone_1"

# OpenCV dictionary
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Physical side length of the ArUco marker (meters)
MARKER_SIZE = 0.025

#Physical marker offset from center to center of marker (meters)
MARKER_OFFSET = 0.060

# Platform origin = center of landing platform.
#
# Each entry:
# marker_id: (x, y, z, rot)
#
# x, y, z = marker center relative to platform center [m]
# rot = marker rotation relative to platform [radians]
        
"""
Corner order:
    0: back of drone
    1: outside of drone
    2: front of drone
    3: inside of drone
"""

MARKER_DATUMS = {
    0: (-MARKER_OFFSET,  0.000, 0.0, 0.0),
    1: ( MARKER_OFFSET,  0.000, 0.0, 0.0),
    2: ( 0.000,  MARKER_OFFSET, 0.0, 0.0),
    3: ( 0.000, -MARKER_OFFSET, 0.0, 0.0),
}


# ==================
# GEOMETRY
# ==================

def marker_object_points(marker_id):
    """
    Return the four marker corners in platform coordinates.

    OpenCV corner order:
      0: back of drone
      1: outside of drone
      2: front of drone
      3: inside of drone
    """

    x, y, z, yaw = MARKER_DATUMS[marker_id]

    h = MARKER_SIZE / 2.0

    # Marker-local coordinates.
    corners = np.array([
        [-h,  h, 0],
        [ h,  h, 0],
        [ h, -h, 0],
        [-h, -h, 0],
    ], dtype=np.float64)

    # Rotate marker around platform Z axis.
    c = math.cos(yaw)
    s = math.sin(yaw)

    R = np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])

    # Transform into platform coordinates.
    corners = corners @ R.T

    corners += np.array([x, y, z])

    return corners


def rotation_matrix_to_quaternion(R):
    """
    Convert 3x3 rotation matrix to ROS quaternion.
    Returns x, y, z, w.
    """
    #Stolen from Lab5 refrence that if this breaks
    r = R.from_matrix(R)
    q = r.as_quat()
    return q[0], q[1], q[2], q[3] #x, y, z, w


# ==================
# ROS2 NODE
# ==================

class PlatformPoseNode(Node):

    def __init__(self):

        super().__init__('platform_pose_node')

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = "camera_optical_frame"

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICT
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.image_sub = self.create_subscription(
            Image,
            f'/{DRONE_NAME}/camera/image_raw',
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            f'/{DRONE_NAME}/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/{DRONE_NAME}/landing_platform/pose',
            10
        )

        self.get_logger().info(
            'Landing platform pose estimator initialized'
        )

    # ==================
    # CAMERA CALIBRATION
    # ==================

    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(
            msg.k, dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            msg.d, dtype=np.float64
        )

        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    # ==================
    # IMAGE CALLBACK
    # ==================            

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        if ids is None:
            self.publish_debug(image)
            return

        ids = ids.flatten()

        cv2.aruco.drawDetectedMarkers(
            image,
            corners,
            ids.reshape(-1, 1)
        )

        object_points = []
        image_points = []

        detected_ids = []

        # Collect all corners from all recognized markers.
        for i, marker_id in enumerate(ids):

            if marker_id not in MARKER_DATUMS:
                continue

            marker_corners = corners[i].reshape(4, 2)

            platform_corners = marker_object_points(
                int(marker_id)
            )

            object_points.extend(platform_corners)
            image_points.extend(marker_corners)

            detected_ids.append(int(marker_id))

        # Need at least 4 correspondences.
        if len(object_points) < 4:
            self.publish_debug(image)
            return

        object_points = np.array(
            object_points,
            dtype=np.float64
        )

        image_points = np.array(
            image_points,
            dtype=np.float64
        )

        # ----------------------------------------------------
        # SOLVE PLATFORM POSE
        # ----------------------------------------------------

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            self.publish_debug(image)
            return

        # Reprojection error for quality checking.
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs
        )

        projected = projected.reshape(-1, 2)

        error = np.mean(
            np.linalg.norm(
                projected - image_points,
                axis=1
            )
        )

        # Reject poor estimates.
        if error > 3.0:
            self.get_logger().warning(
                f'PnP rejected: reprojection error = {error:.2f}px'
            )
            self.publish_debug(image)
            return

        # Convert rotation vector into matrix.
        R, _ = cv2.Rodrigues(rvec)

        # Platform origin expressed in camera coordinates.
        position = tvec.flatten()

        # Platform orientation expressed in camera coordinates.
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        # ==================
        # PUBLISH POSE
        # ==================

        pose_msg = PoseStamped()

        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f'Platform: '
            f'x={position[0]:.3f} '
            f'y={position[1]:.3f} '
            f'z={position[2]:.3f} m | '
            f'markers={detected_ids} | '
            f'error={error:.2f}px',
            throttle_duration_sec=0.5
        )

        # Draw platform coordinate axes.
        cv2.drawFrameAxes(
            image,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            0.05
        )

        self.publish_debug(image)

    # ==================
    # DEBUG IMAGE
    # ==================

    def publish_debug(self, image):

        debug_msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8'
        )

        self.debug_pub.publish(debug_msg)


# ==================
# MAIN
# ==================

def main(args=None):

    rclpy.init(args=args)

    node = PlatformPoseNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
