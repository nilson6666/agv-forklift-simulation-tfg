#!/usr/bin/env python3
"""Keyboard teleop for the AGV forklift: mecanum base + fork.

Base (holonomic, mecanum):
    w/x : forward / backward
    a/d : strafe left / right
    q/e : turn left / right
    s   : STOP base

Fork (logical height 0.0 - 2.65 m, same kinematics as
fork_elevator_node.py: 0.15 m of free lift with the fork only,
then symmetric mast/fork split; h=0 -> tine 0.03 m above the floor):
    r/f : raise / lower (0.05 m step)
    R/F : raise / lower (0.25 m step)
    0   : fork down to the floor (height 0)

Speeds:
    +/- : increase / decrease linear and angular speed (x1.1 / x0.9)

CTRL-C or ESC to quit.
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Same kinematics as fork_elevator_node.py (URDF limits:
# mast [-1.25, 0], fork [-0.15, 1.25])
FREE_LIFT = 0.15      # range covered by fork_joint only (from -0.15 to 0)
LOGICAL_RANGE = 2.65  # FREE_LIFT + 2*1.25


class TeleopForklift(Node):
    def __init__(self):
        super().__init__('teleop_forklift')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mast_pub = self.create_publisher(Float64, '/mast_cmd', 10)
        self.fork_pub = self.create_publisher(Float64, '/fork_cmd', 10)

        self.lin_speed = 0.5    # m/s
        self.ang_speed = 0.5    # rad/s
        self.fork_height = 0.0  # current logical height [0, 2.65]

    def publish_twist(self, vx, vy, wz):
        t = Twist()
        t.linear.x = vx * self.lin_speed
        t.linear.y = vy * self.lin_speed
        t.angular.z = wz * self.ang_speed
        self.cmd_vel_pub.publish(t)

    def publish_fork(self):
        # Free lift: the first 0.15 m are done by the fork only (from -0.15
        # to 0); above that, symmetric mast/fork split
        if self.fork_height <= FREE_LIFT:
            mast_pos, fork_pos = 0.0, self.fork_height - FREE_LIFT
        else:
            half = (self.fork_height - FREE_LIFT) / 2.0
            mast_pos, fork_pos = -half, half  # mast axis is inverted
        m = Float64(); m.data = mast_pos
        f = Float64(); f.data = fork_pos
        self.mast_pub.publish(m)
        self.fork_pub.publish(f)

    def move_fork(self, delta):
        self.fork_height = max(0.0, min(self.fork_height + delta, LOGICAL_RANGE))
        self.publish_fork()


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopForklift()
    print(__doc__)
    status_every = 0
    try:
        while True:
            key = get_key(settings)
            if key in ('\x03', '\x1b'):   # CTRL-C or ESC
                break
            elif key == 'w':
                node.publish_twist(1, 0, 0)
            elif key == 'x':
                node.publish_twist(-1, 0, 0)
            elif key == 'a':
                node.publish_twist(0, 1, 0)
            elif key == 'd':
                node.publish_twist(0, -1, 0)
            elif key == 'q':
                node.publish_twist(0, 0, 1)
            elif key == 'e':
                node.publish_twist(0, 0, -1)
            elif key == 's':
                node.publish_twist(0, 0, 0)
            elif key == 'r':
                node.move_fork(+0.05)
            elif key == 'f':
                node.move_fork(-0.05)
            elif key == 'R':
                node.move_fork(+0.25)
            elif key == 'F':
                node.move_fork(-0.25)
            elif key == '0':
                node.fork_height = 0.0
                node.publish_fork()
            elif key == '+':
                node.lin_speed *= 1.1
                node.ang_speed *= 1.1
            elif key == '-':
                node.lin_speed *= 0.9
                node.ang_speed *= 0.9

            if key:
                status_every += 1
                print(f'\rvel={node.lin_speed:.2f} m/s | '
                      f'forquilla={node.fork_height:.2f} m   ', end='', flush=True)
    finally:
        # Stop the base on exit (the fork keeps its last setpoint)
        node.publish_twist(0, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nteleop tancat, base aturada.')


if __name__ == '__main__':
    main()
