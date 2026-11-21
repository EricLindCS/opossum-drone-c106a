import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

HELP = """
Hold a key to drive the TurtleBot. Let go to stop.
---------------------------------------------------
        q    w    e
        a    s    d
        z    x    c

w/x : forward / backward        a/d : turn left / right in place
q/e : forward while turning     z/c : backward while turning
s or space : stop now

CTRL-C to quit
"""

# key -> (linear direction, angular direction)
BINDINGS = {
    'w': (1, 0), 'x': (-1, 0),
    'a': (0, 1), 'd': (0, -1),
    'q': (1, 1), 'e': (1, -1),
    'z': (-1, -1), 'c': (-1, 1),
}

MAX_LIN_VEL = 0.22  # m/s, burger max
MAX_ANG_VEL = 2.84  # rad/s


# The terminal never tells us when a key is released, only when it's pressed.
# But holding a key makes it auto-repeat, so we treat a key as held as long as
# repeats keep coming in. The first repeat takes longer than the rest (the
# keyboard's repeat delay), which is why there are two timeouts.
class HoldTeleop(Node):

    def __init__(self):
        super().__init__('hold_teleop')
        self.linear_speed = min(self.declare_parameter('linear_speed', 0.2).value, MAX_LIN_VEL)
        self.angular_speed = min(self.declare_parameter('angular_speed', 1.5).value, MAX_ANG_VEL)
        # these have to be longer than the keyboard's repeat delay/rate or it stutters
        self.first_press_timeout = self.declare_parameter('first_press_timeout', 0.7).value
        self.repeat_timeout = self.declare_parameter('repeat_timeout', 0.15).value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.held_key = None
        self.deadline = 0.0

    def on_key(self, key, now):
        if key in BINDINGS:
            timeout = self.repeat_timeout if key == self.held_key else self.first_press_timeout
            self.held_key = key
            self.deadline = now + timeout
        elif key in (' ', 's'):
            self.held_key = None

    def command(self):
        twist = Twist()
        if self.held_key is not None:
            lin, ang = BINDINGS[self.held_key]
            twist.linear.x = lin * self.linear_speed
            twist.angular.z = ang * self.angular_speed
        return twist

    def step(self, key):
        now = time.monotonic()
        was_moving = self.held_key is not None
        if key:
            self.on_key(key, now)
        if self.held_key is not None and now > self.deadline:
            self.held_key = None

        moving = self.held_key is not None
        # only publish while moving (plus one stop message), so we don't fight
        # anything else publishing cmd_vel
        if moving or was_moving:
            twist = self.command()
            self.cmd_pub.publish(twist)
            status = f'linear {twist.linear.x:+.2f} m/s   angular {twist.angular.z:+.2f} rad/s'
            print(f'\r{status}   ', end='', flush=True)


def read_key(fd, timeout):
    rlist, _, _ = select.select([fd], [], [], timeout)
    if not rlist:
        return ''
    # read everything waiting so repeats don't pile up, and just use the last key
    data = os.read(fd, 1024).decode(errors='ignore')
    return data[-1] if data else ''


def main():
    rclpy.init()
    node = HoldTeleop()

    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            node.step(read_key(fd, 0.05))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        node.cmd_pub.publish(Twist())
        print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
