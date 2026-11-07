#!/usr/bin/env python3
"""Visual servo approach to a pallet using AprilTag (standalone demo).

Reads the tag pose through TF (published by apriltag_ros from the fork
camera) and drives the robot to the STANDOFF: centred on the tag's
normal axis, square to the pallet face and at the requested distance.
It does NOT insert the tines: the full mission is in pallet_cycle_bt.py,
which uses this same controller as a leaf.

Usage (with the sim + apriltag.launch.py running):
  ros2 run agv_description pallet_approach.py
  ros2 run agv_description pallet_approach.py --ros-args -p tag_id:=3 -p standoff:=1.8

Simultaneous 3-channel P control (mecanum):
  vx <- (dist - standoff)      drives forward/backward to the distance
  vy <- py (lateral position)  centres the tag in front of the robot
  wz <- skew (tag skew)        squares up to the face

vy and wz converge together to "on the tag axis and facing it"
(classic pallet-docking servo). If the tag is lost for longer than
tag_timeout, the robot stops and goes back to WAIT_TAG. DONE when the 3
errors stay within tolerance for settle_time in a row.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def quat_to_R(x, y, z, w):
    """3x3 rotation matrix from a quaternion."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ]


def clamp(v, lim):
    return max(-lim, min(lim, v))


class PalletApproach(Node):

    def __init__(self):
        super().__init__('pallet_approach')

        self.declare_parameter('tag_id', 0)
        self.declare_parameter('base_frame', 'base_footprint_link')
        # Target distance from the ROBOT CENTRE to the tag face.
        # 2.5 m leaves the tine tips (x=+2.41 from the base) 0.09 m from
        # the face WITHOUT touching it (the standoff is pre-insertion) and
        # the fork camera (x=+1.146) 1.35 m from the tag, inside the
        # detection window with the fork lowered (camera-tag 0.30-1.75 m).
        self.declare_parameter('standoff', 2.5)
        self.declare_parameter('kp_x', 0.5)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('kp_yaw', 0.8)
        self.declare_parameter('max_lin', 0.3)    # slow servo: precision
        self.declare_parameter('max_ang', 0.4)
        self.declare_parameter('tol_x', 0.02)     # m
        self.declare_parameter('tol_y', 0.02)     # m
        self.declare_parameter('tol_yaw', 0.03)   # rad (~1.7 degrees)
        self.declare_parameter('tag_timeout', 1.0)   # s without seeing the tag
        self.declare_parameter('settle_time', 1.0)   # s within tolerance
        self.declare_parameter('rate', 10.0)

        tag_id = self.get_parameter('tag_id').value
        self.tag_frame = f'tag36h11:{tag_id}'
        self.base_frame = self.get_parameter('base_frame').value
        self.standoff = self.get_parameter('standoff').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_lin = self.get_parameter('max_lin').value
        self.max_ang = self.get_parameter('max_ang').value
        self.tol_x = self.get_parameter('tol_x').value
        self.tol_y = self.get_parameter('tol_y').value
        self.tol_yaw = self.get_parameter('tol_yaw').value
        self.tag_timeout = self.get_parameter('tag_timeout').value
        self.settle_time = self.get_parameter('settle_time').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = 'WAIT_TAG'
        self.settled_since = None
        period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(period, self.control_step)
        self.get_logger().info(
            f'Esperant {self.tag_frame}; standoff {self.standoff} m')

    def stop(self):
        self.cmd_pub.publish(Twist())

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
            self.state = s

    def control_step(self):
        if self.state == 'DONE':
            self.stop()
            return

        # Tag pose in the robot frame (latest available)
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame,
                                                self.tag_frame, Time())
        except Exception:
            self.set_state('WAIT_TAG')
            self.stop()
            return

        age = (self.get_clock().now() - Time.from_msg(t.header.stamp))
        if age > Duration(seconds=self.tag_timeout):
            # Stale TF: the detector no longer sees the tag
            self.set_state('WAIT_TAG')
            self.stop()
            return

        px = t.transform.translation.x
        py = t.transform.translation.y
        q = t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)

        # Tag normal (z axis of its frame) projected onto the XY plane and
        # oriented towards the robot (independent of the detector's sign
        # convention): if it points towards the tag, flip it.
        nx, ny = R[0][2], R[1][2]
        if nx * px + ny * py > 0.0:
            nx, ny = -nx, -ny
        norm = math.hypot(nx, ny)
        if norm < 1e-6:
            self.stop()
            return
        nx, ny = nx / norm, ny / norm

        # Errors: the robot must end up looking along -normal (facing the
        # tag), centred (py=0) and at standoff distance.
        dist = math.hypot(px, py)
        e_x = dist - self.standoff
        e_y = py
        e_yaw = math.atan2(-ny, -nx)   # skew relative to the normal

        self.set_state('SERVO')
        cmd = Twist()
        # Caution: do not drive forward while strongly rotated (lateral/yaw
        # take priority; avoids wide arcs next to the pallet)
        gate = 1.0 if abs(e_yaw) < 0.3 else 0.0
        cmd.linear.x = clamp(self.kp_x * e_x, self.max_lin) * gate
        cmd.linear.y = clamp(self.kp_y * e_y, self.max_lin)
        cmd.angular.z = clamp(self.kp_yaw * e_yaw, self.max_ang)
        self.cmd_pub.publish(cmd)

        # Sustained convergence -> DONE
        within = (abs(e_x) < self.tol_x and abs(e_y) < self.tol_y
                  and abs(e_yaw) < self.tol_yaw)
        now = self.get_clock().now()
        if within:
            if self.settled_since is None:
                self.settled_since = now
            elif now - self.settled_since > Duration(seconds=self.settle_time):
                self.stop()
                self.set_state('DONE')
                self.get_logger().info(
                    f'DONE: dist={dist:.3f} m, lateral={e_y:+.3f} m, '
                    f'skew={math.degrees(e_yaw):+.2f} graus')
        else:
            self.settled_since = None


def main():
    rclpy.init()
    node = PalletApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()


if __name__ == '__main__':
    main()
