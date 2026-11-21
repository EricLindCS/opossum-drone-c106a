import csv
import math
import os
import sys
import termios
import threading
import time
import tty

import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from lab6_capture.hold_teleop import read_key

HELP = """
Press SPACE (or c) to save a picture.
CTRL-C to quit
"""

CSV_FIELDS = ['file', 'image_stamp', 'tf_stamp', 'tf_source',
              'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw_deg']


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CapturePose(Node):

    def __init__(self):
        super().__init__('capture_pose')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('output_dir', os.path.expanduser('~/ros_workspaces/lab6/captures'))
        self.declare_parameter('max_image_age', 1.0)
        # set this to false to just save images (for calibration, no SLAM needed)
        self.declare_parameter('with_pose', True)
        self.with_pose = self.get_parameter('with_pose').value

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.output_dir = self.get_parameter('output_dir').value
        self.max_image_age = self.get_parameter('max_image_age').value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_image_arrival = 0.0
        self.image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, self.image_topic, self.on_image, 10)

        # new folder every time you run it
        prefix = 'run' if self.with_pose else 'calib'
        self.output_dir = os.path.join(self.output_dir, time.strftime(prefix + '_%Y%m%d_%H%M%S'))
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'poses.csv')
        self.count = 0
        self.get_logger().info(
            f'listening on {self.image_topic}, saving to {self.output_dir}')

    def on_image(self, msg):
        with self.lock:
            self.latest_image = msg
            self.latest_image_arrival = time.monotonic()

    def lookup_pose(self, stamp):
        # try to get the pose from when the image was taken. if the robot and
        # workstation clocks are off from each other that fails, so fall back to
        # the latest pose
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=0.5))
            return tf, 'image_stamp'
        except TransformException as ex:
            self.get_logger().warn(f'no TF at image time ({ex}); using latest instead')
        tf = self.tf_buffer.lookup_transform(
            self.target_frame, self.source_frame, Time())
        return tf, 'latest'

    def capture(self):
        with self.lock:
            msg = self.latest_image
            age = time.monotonic() - self.latest_image_arrival
        if msg is None:
            # some of the robots publish under their name, like /lemon/image_raw
            live = [name for name, types in self.get_topic_names_and_types()
                    if 'sensor_msgs/msg/Image' in types and self.count_publishers(name) > 0]
            hint = (f' these topics have images: {", ".join(live)}, '
                    f'try --ros-args -p image_topic:=<topic>' if live
                    else ' nothing is publishing images right now.')
            self.get_logger().error(
                f'no images on {self.image_topic}, is the camera running?' + hint)
            return
        if age > self.max_image_age:
            self.get_logger().warn(f'last image is {age:.1f} s old, did the camera stop?')

        if not self.with_pose:
            self.save_image_only(msg)
            return

        try:
            tf, tf_source = self.lookup_pose(msg.header.stamp)
        except TransformException as ex:
            self.get_logger().error(
                f'can\'t get {self.target_frame} -> {self.source_frame} ({ex}). '
                f'is SLAM running?')
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as ex:
            self.get_logger().error(f'couldn\'t convert the {msg.encoding} image: {ex}')
            return

        name = f'img_{self.count:04d}.png'
        cv2.imwrite(os.path.join(self.output_dir, name), frame)

        t = tf.transform.translation
        q = tf.transform.rotation
        row = {
            'file': name,
            'image_stamp': f'{stamp_to_sec(msg.header.stamp):.3f}',
            'tf_stamp': f'{stamp_to_sec(tf.header.stamp):.3f}',
            'tf_source': tf_source,
            'x': f'{t.x:.4f}', 'y': f'{t.y:.4f}', 'z': f'{t.z:.4f}',
            'qx': f'{q.x:.5f}', 'qy': f'{q.y:.5f}', 'qz': f'{q.z:.5f}', 'qw': f'{q.w:.5f}',
            'yaw_deg': f'{math.degrees(quaternion_to_yaw(q)):.2f}',
        }
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

        self.count += 1
        self.get_logger().info(
            f'saved {name}: x={t.x:.3f} y={t.y:.3f} yaw={row["yaw_deg"]} deg ({tf_source})')

    def save_image_only(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as ex:
            self.get_logger().error(f'couldn\'t convert the {msg.encoding} image: {ex}')
            return
        name = f'img_{self.count:04d}.png'
        cv2.imwrite(os.path.join(self.output_dir, name), frame)
        self.count += 1
        self.get_logger().info(f'saved {name}')


def main():
    # handle ctrl-c ourselves, otherwise rclpy shuts down under the spin thread
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CapturePose()

    # spin in the background so we keep getting images/tf while waiting for keys
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            key = read_key(fd, 0.1)
            if key in (' ', 'c', '\n'):
                node.capture()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)

    executor.shutdown()
    spin_thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
