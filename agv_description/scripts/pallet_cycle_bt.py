#!/usr/bin/env python3
"""Behaviour tree for the trailer unloading mission.

For each pallet: navigate to the trailer, locate the tag, pick up the
pallet, carry it to the drop zone and put it down (the first one on the
floor, the rest stacked on top of the previous one).

Why a behaviour tree instead of a hand-written state machine: an FSM
suffers a "transition explosion" as it grows (every new state forces
changes to its neighbours). A tree isolates each action in an
independent, reusable leaf, composed with a generic node (here
`Sequence`): adding a feature means adding a leaf and wiring it in,
without touching the ones that already work. It is the same paradigm
Nav2 uses internally (BehaviorTree.CPP).

Leaves (each `update()` is a non-blocking tick):
- NavigateToPose: Nav2 `navigate_to_pose` action.
- VisualServoToTag: 3-channel P servo on an AprilTag (same logic as
  `pallet_approach.py`). `tag_frame_prefix` makes it reusable for both
  cameras: `mast_tag` for a coarse correction before raising the fork,
  `tag36h11` (default) for the fine approach.
- CreepRelative: blind forward/backward motion by odometry, projected
  onto the robot heading when the leaf starts (not onto the world X
  axis), valid after a servo/Nav2 with any orientation.
- SetForkHeight: raises/lowers to a logical height h (same kinematic
  convention as `fork_elevator_node.py::height_to_joints`). Accepts a
  fixed `height` or a `height_key` (read from the blackboard).
- EstimateHeightFromTag: converts a tag's height into logical height h
  (see TAG_Z_TO_HEIGHT_OFFSET) and writes it to the blackboard.
- CompareHeightEstimates: checks that the 2 height estimates (mast and
  fork) agree; otherwise FAILURE.
- SetFootprint: changes the footprint of the Nav2 costmaps.

The load height is not a fixed number: it is determined at run time
from 2 independent AprilTag estimates (mast camera, coarse, before
docking; fork camera, fine, when confirming the tag) that must agree
within tolerance. If they disagree it is treated as a perception error
and the cycle fails instead of picking up the pallet blindly.

Usage (with the sim in slam:=true mode, nav2.launch.py, apriltag.launch.py
and the pallets created with spawn_palets.py):
  ros2 run agv_description pallet_cycle_bt.py --ros-args -p use_sim_time:=true
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

import py_trees
import py_trees_ros.trees

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose as NavigateToPoseAction
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType


# Full T-shaped footprint (with tines) vs. chassis only — see SetFootprint.
FULL_FOOTPRINT = ("[[-1.28,-0.72],[-1.28,0.72],[1.26,0.72],[1.26,0.27],"
                   "[2.43,0.27],[2.43,-0.27],[1.26,-0.27],[1.26,-0.72]]")
CHASSIS_FOOTPRINT = "[[-1.28,-0.72],[-1.28,0.72],[1.26,0.72],[1.26,-0.72]]"


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


def height_to_joints(h, free_lift=0.15):
    """Logical height [0, 2.65] -> (mast_cmd, fork_cmd).

    Must match `fork_elevator_node.py::height_to_joints` and
    `teleop_forklift.py` — if it changes in one place, change it in all.
    """
    if h <= free_lift:
        return 0.0, h - free_lift
    half = (h - free_lift) / 2.0
    return -half, half


def clamp(v, lim):
    return max(-lim, min(lim, v))


def clamp_range(v, lo, hi):
    return max(lo, min(hi, v))


class VisualServoToTag(py_trees.behaviour.Behaviour):
    """Drives the robot to the tag standoff (like `pallet_approach.py`).

    `tag_frame_prefix`: reusable for both cameras (like
    `EstimateHeightFromTag`) — 'tag36h11' for the fork camera (fine
    approach), 'mast_tag' for the mast camera (coarse x/y/yaw correction
    BEFORE the fine stage, using the fact that the same detection that
    gives the height already carries the full 3D pose of the tag).

    `correct_x`: when `False`, it neither publishes `cmd.linear.x` nor
    requires it to consider the leaf converged — lateral + yaw only."""

    def __init__(self, name, tag_id=0, standoff=2.5, base_frame='base_footprint_link',
                 tag_frame_prefix='tag36h11', min_dist=None, correct_x=True,
                 kp_x=0.5, kp_y=0.8, kp_yaw=0.8, max_lin=0.3, max_ang=0.4,
                 tol_x=0.02, tol_y=0.02, tol_yaw=0.03,
                 tag_timeout=1.0, settle_time=1.0, overall_timeout=60.0):
        super().__init__(name)
        self.tag_frame = f'{tag_frame_prefix}:{tag_id}'
        self.base_frame = base_frame
        self.standoff = standoff
        self.correct_x = correct_x
        # Safety limit: with the tines still low, the horizontal margin to
        # the trailer collision at the nominal standoff (2.5 m) is only
        # ~9 cm, and a plain overshoot of the P controller reached it.
        # `min_dist` clamps `cmd.linear.x` to 0 regardless of the P term
        # once `dist` reaches the minimum — the robot's own safety net,
        # independent of whether Nav2 has the trailer in its map.
        self.min_dist = min_dist
        self.kp_x, self.kp_y, self.kp_yaw = kp_x, kp_y, kp_yaw
        self.max_lin, self.max_ang = max_lin, max_ang
        self.tol_x, self.tol_y, self.tol_yaw = tol_x, tol_y, tol_yaw
        self.tag_timeout = tag_timeout
        self.settle_time = settle_time
        self.overall_timeout = overall_timeout

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def initialise(self):
        self.settled_since = None
        self.started_at = self.node.get_clock().now()
        self.logger.info(f'esperant {self.tag_frame}; standoff {self.standoff} m')

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def update(self):
        now = self.node.get_clock().now()
        if (now - self.started_at) > Duration(seconds=self.overall_timeout):
            self.logger.error('timeout global sense convergir')
            self._stop()
            return py_trees.common.Status.FAILURE

        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.tag_frame, Time())
        except Exception:
            self._stop()
            return py_trees.common.Status.RUNNING

        age = now - Time.from_msg(t.header.stamp)
        if age > Duration(seconds=self.tag_timeout):
            self._stop()
            return py_trees.common.Status.RUNNING

        px = t.transform.translation.x
        py_ = t.transform.translation.y
        q = t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)

        nx, ny = R[0][2], R[1][2]
        if nx * px + ny * py_ > 0.0:
            nx, ny = -nx, -ny
        norm = math.hypot(nx, ny)
        if norm < 1e-6:
            self._stop()
            return py_trees.common.Status.RUNNING
        nx, ny = nx / norm, ny / norm

        dist = math.hypot(px, py_)
        e_x = dist - self.standoff
        e_y = py_
        e_yaw = math.atan2(-ny, -nx)

        cmd = Twist()
        if self.correct_x:
            gate = 1.0 if abs(e_yaw) < 0.3 else 0.0
            cmd.linear.x = clamp(self.kp_x * e_x, self.max_lin) * gate
            if self.min_dist is not None and dist <= self.min_dist:
                cmd.linear.x = min(cmd.linear.x, 0.0)
        cmd.linear.y = clamp(self.kp_y * e_y, self.max_lin)
        cmd.angular.z = clamp(self.kp_yaw * e_yaw, self.max_ang)
        self.cmd_pub.publish(cmd)

        within = abs(e_y) < self.tol_y and abs(e_yaw) < self.tol_yaw
        if self.correct_x:
            within = within and abs(e_x) < self.tol_x
        if within:
            if self.settled_since is None:
                self.settled_since = now
            elif now - self.settled_since > Duration(seconds=self.settle_time):
                self._stop()
                self.logger.info(
                    f'DONE dist={dist:.3f} lateral={e_y:+.3f} skew={math.degrees(e_yaw):+.2f}')
                return py_trees.common.Status.SUCCESS
        else:
            self.settled_since = None
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._stop()



# Deriving the logical height h from the tag height:
#   tine at h=0       -> 0.03 m above the floor   (fork_elevator_node.py)
#   tag on the pallet -> 0.072 m above the pallet bottom (models/palet_export/robot.sdf)
#   pocket opening    -> 0.022-0.100 m (0.078 m tall), centre 0.061 m
#   tine thickness    -> 0.06 m
# The WHOLE thickness of the tine must be centred in the pocket, not just
# its base: with a 0.078 m pocket and a 0.06 m tine there is only 0.018 m
# of total clearance, and centring the base makes the top of the tine hit
# the pallet's top boards.
# h = (tag_z - 0.072 + 0.061) - 0.03 - 0.03 = tag_z - 0.071
TAG_Z_TO_HEIGHT_OFFSET = 0.071
MAX_LIFT = 2.65

# Transport height: once clear of the truck, the pallet does not travel
# raised all the way to the drop zone (high centre of gravity), but just
# above the floor.
TRANSPORT_HEIGHT = 0.20


class EstimateHeightFromTag(py_trees.behaviour.Behaviour):
    """A single TF lookup (base_frame -> tag_frame) that converts the
    tag height into the fork's logical height h (TAG_Z_TO_HEIGHT_OFFSET).
    Writes the result to the blackboard (`blackboard_key`). Reusable for
    both cameras: only `tag_frame` changes (mast_tag:<id> for the mast,
    tag36h11:<id> for the fork — see apriltag.launch.py for why the same
    physical tag has DIFFERENT frames).

    `tag_timeout` is 3 s because, with a loaded CPU, detection is
    intermittent (not just slow) and 1 s rejected valid detections.
    `overall_timeout` still fails the leaf if no detection ever arrives."""

    def __init__(self, name, tag_frame, blackboard_key, base_frame='base_footprint_link',
                 tag_timeout=3.0, overall_timeout=15.0):
        super().__init__(name)
        self.tag_frame = tag_frame
        self.blackboard_key = blackboard_key
        self.base_frame = base_frame
        self.tag_timeout = tag_timeout
        self.overall_timeout = overall_timeout
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=blackboard_key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.started_at = self.node.get_clock().now()
        self.logger.info(f'esperant {self.tag_frame} per estimar alçada')

    def update(self):
        now = self.node.get_clock().now()
        if (now - self.started_at) > Duration(seconds=self.overall_timeout):
            self.logger.error(f'timeout esperant {self.tag_frame}')
            return py_trees.common.Status.FAILURE

        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.tag_frame, Time())
        except Exception:
            return py_trees.common.Status.RUNNING

        age = now - Time.from_msg(t.header.stamp)
        if age > Duration(seconds=self.tag_timeout):
            return py_trees.common.Status.RUNNING

        tag_z = t.transform.translation.z
        h = clamp_range(tag_z - TAG_Z_TO_HEIGHT_OFFSET, 0.0, MAX_LIFT)
        setattr(self.blackboard, self.blackboard_key, h)
        self.logger.info(f'{self.tag_frame} z={tag_z:.3f} -> h={h:.3f}')
        return py_trees.common.Status.SUCCESS


class CompareHeightEstimates(py_trees.behaviour.Behaviour):
    """Compares 2 heights already on the blackboard (one per camera) and
    fails if they differ by more than `tolerance` (cross-validation): if
    the 2 independent estimates disagree, something is wrong (spurious
    detection, wrong tag, calibration error) and the pallet must NOT be
    picked up blindly with a number that may be wrong."""

    def __init__(self, name, key_a, key_b, tolerance=0.05):
        super().__init__(name)
        self.key_a, self.key_b = key_a, key_b
        self.tolerance = tolerance
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=key_a, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=key_b, access=py_trees.common.Access.READ)

    def update(self):
        a = getattr(self.blackboard, self.key_a)
        b = getattr(self.blackboard, self.key_b)
        diff = abs(a - b)
        if diff > self.tolerance:
            self.logger.error(
                f'{self.key_a}={a:.3f} vs {self.key_b}={b:.3f} -> '
                f'diff {diff:.3f} m > tolerància {self.tolerance} m')
            return py_trees.common.Status.FAILURE
        self.logger.info(f'{self.key_a}={a:.3f} ~ {self.key_b}={b:.3f} (diff {diff:.3f} m) OK')
        return py_trees.common.Status.SUCCESS


class CreepRelative(py_trees.behaviour.Behaviour):
    """Moves forward/backward `distance` m (signed) blindly by odometry,
    projected onto the robot heading when the leaf starts (not onto the
    world axis — valid after a servo/Nav2 with any yaw)."""

    def __init__(self, name, distance, speed=0.12, tol=0.01, kp=0.6):
        super().__init__(name)
        self.distance = distance
        self.speed = speed
        self.tol = tol
        self.kp = kp

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.x = self.y = self.yaw = None
        self.node.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z))

    def initialise(self):
        self.start = None  # captured on the first update() with valid odom

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def update(self):
        if self.x is None:
            return py_trees.common.Status.RUNNING
        if self.start is None:
            self.start = (self.x, self.y, self.yaw)
            self.logger.info(f'inici a ({self.x:.3f},{self.y:.3f}), objectiu {self.distance:+.3f} m')
        x0, y0, yaw0 = self.start
        traveled = (self.x - x0) * math.cos(yaw0) + (self.y - y0) * math.sin(yaw0)
        e = self.distance - traveled
        if abs(e) < self.tol:
            self._stop()
            return py_trees.common.Status.SUCCESS
        cmd = Twist()
        cmd.linear.x = clamp(self.kp * e, self.speed)
        self.cmd_pub.publish(cmd)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._stop()


class SetForkHeight(py_trees.behaviour.Behaviour):
    """Raises/lowers to logical height h (fork_elevator_node.py convention).

    `height`: fixed value, known when building the tree (e.g.
    `BaixarPalet` to the floor). `height_key`: reads h from the
    blackboard when the leaf starts (height determined at run time by
    `EstimateHeightFromTag`).
    Exactly one of the 2 must be given. `height_offset` is always added
    on top of either (e.g. `ElevarPalet`: confirmed insertion height + a
    fixed lift clearance)."""

    def __init__(self, name, height=None, height_key=None, height_offset=0.0,
                 tol=0.01, settle_time=1.5):
        super().__init__(name)
        assert (height is None) != (height_key is None), \
            'SetForkHeight: cal donar height O height_key, no cap ni els 2'
        self.height_key = height_key
        self.height_offset = height_offset
        self.tol = tol
        self.settle_time = settle_time
        self.mast_target = self.fork_target = None
        if height_key is not None:
            self.blackboard = py_trees.blackboard.Client(name=name)
            self.blackboard.register_key(key=height_key, access=py_trees.common.Access.READ)
        else:
            self.mast_target, self.fork_target = height_to_joints(height + height_offset)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.mast_pub = self.node.create_publisher(Float64, '/mast_cmd', 10)
        self.fork_pub = self.node.create_publisher(Float64, '/fork_cmd', 10)
        self.mast = self.fork = None
        self.node.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, m):
        d = dict(zip(m.name, m.position))
        self.mast = d.get('moving_mast_joint')
        self.fork = d.get('fork_joint')

    def initialise(self):
        if self.height_key is not None:
            h = getattr(self.blackboard, self.height_key)
            self.mast_target, self.fork_target = height_to_joints(h + self.height_offset)
        self.settled_since = None
        self.logger.info(f'h target -> mast={self.mast_target:.3f}, fork={self.fork_target:.3f}')

    def update(self):
        self.mast_pub.publish(Float64(data=self.mast_target))
        self.fork_pub.publish(Float64(data=self.fork_target))
        if self.mast is None or self.fork is None:
            return py_trees.common.Status.RUNNING
        settled = (abs(self.mast - self.mast_target) < self.tol
                   and abs(self.fork - self.fork_target) < self.tol)
        now = self.node.get_clock().now()
        if settled:
            if self.settled_since is None:
                self.settled_since = now
            elif now - self.settled_since > Duration(seconds=self.settle_time):
                return py_trees.common.Status.SUCCESS
        else:
            self.settled_since = None
        return py_trees.common.Status.RUNNING


class SetFootprint(py_trees.behaviour.Behaviour):
    """Changes the footprint of both Nav2 costmaps (with/without tines).

    The trailer's LETHAL inflation zone reaches y≈3.6-3.8 (real obstacle
    at y=3.245 + inscribed radius 0.73 m). With the T-shaped footprint
    (tines up to x=+2.43), any pose close enough to the tag to detect it
    (base-tag ~1.45-2.9 m) leaves the tine tips inside the lethal zone:
    no pose satisfies both "tines outside the lethal zone" and "tag within
    detection range", and Nav2 answers "Failed to make progress". So the
    footprint is reduced (chassis only) for the final docking approach,
    where bringing the tines close is precisely the job, and restored to
    the full one for the rest of the navigation."""

    def __init__(self, name, footprint):
        super().__init__(name)
        self.footprint = footprint

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.clients = [
            self.node.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters'),
            self.node.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters'),
        ]
        # Without waiting for DDS discovery before the first call, one of
        # the 2 requests can be lost and the leaf stays RUNNING forever
        # (`future.done()` never becomes true). setup() is called only once
        # when the tree starts: waiting here is cheap and guarantees that
        # client and server have matched when `initialise()` calls
        # `call_async`.
        for c in self.clients:
            if not c.wait_for_service(timeout_sec=10.0):
                self.node.get_logger().error(f'{c.srv_name} no disponible')

    def initialise(self):
        req = SetParameters.Request()
        req.parameters = [RclParameter(
            name='footprint',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                  string_value=self.footprint))]
        self.futures = [c.call_async(req) for c in self.clients]

    def update(self):
        if all(f.done() for f in self.futures):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class NavigateToPose(py_trees.behaviour.Behaviour):
    """Nav2 `navigate_to_pose` action."""

    def __init__(self, name, x, y, yaw=0.0, frame_id='map'):
        super().__init__(name)
        self.x, self.y, self.yaw, self.frame_id = x, y, yaw, frame_id

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.client = ActionClient(self.node, NavigateToPoseAction, 'navigate_to_pose')

    def initialise(self):
        self.goal_handle = None
        self.result_future = None
        self.send_future = None
        goal = NavigateToPoseAction.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.logger.error('navigate_to_pose no disponible')
            self.send_future = None
            return
        self.send_future = self.client.send_goal_async(goal)

    def update(self):
        if self.send_future is None:
            return py_trees.common.Status.FAILURE
        if self.goal_handle is None:
            if not self.send_future.done():
                return py_trees.common.Status.RUNNING
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.logger.error('goal rebutjat per Nav2')
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING
        # `done()` is set for ANY outcome (success, abort, cancel): the real
        # status must be checked, otherwise the BT would carry on with the
        # robot stopped halfway when Nav2 aborts the goal.
        result = self.result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.logger.error(f'navigate_to_pose no ha arribat (status={result.status})')
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if (new_status == py_trees.common.Status.INVALID
                and self.goal_handle is not None):
            self.goal_handle.cancel_goal_async()


def build_tree(tag_id, standoff, insert_distance, lift_clearance, backout_distance,
                place_height, withdraw_distance, pickup_pose, drop_pose,
                height_tolerance=0.05, label='', align_tag_id=None, place_standoff=2.6,
                settle_clearance=0.005):
    """Full cycle: navigate + pick up + navigate + put down.

    `label`: suffix added to every leaf name (e.g. '_p0') only to tell
    them apart in the logs when `build_mission()` chains several cycles;
    it does not change any behaviour.

    The leaves are grouped into 5 phases (each one a `Sequence`), run in
    order; if any leaf fails, the whole cycle fails:
      AproximacioPalet: FootprintDocking -> NavAproximacio
      LocalitzarPalet:  EstimarAlcadaMastil -> CentrarAmbMastil ->
                        PujarAAlcadaEstimada -> ServoAprilTag ->
                        ConfirmarAlcadaForquilla -> VerificarConcordanca ->
                        AjustarAlcadaConfirmada
      AgafarPalet:      InserirPues -> ElevarPalet -> RetirarPues ->
                        AllunyarDelCamio -> BaixarPerTransport
      Transport:        FootprintComplet -> NavDescarrega
      DeixarPalet:      depends on the pallet (see below)

    Pick-up notes (same for every pallet):
    - CentrarAmbMastil: Nav2 leaves the robot within its own tolerance
      (~0.15 m / ~8°), but at the 2.5 m standoff that is enough for the
      narrow fork camera to point at the gap between two neighbouring
      pallets. The mast camera (wide field of view) corrects x/y/yaw
      first, so the fork camera finds the tag almost centred.
    - `min_dist=standoff` on both servos: the fork approaches while still
      low, and the margin to the trailer is only ~9 cm (see
      VisualServoToTag). In addition, `nav2.launch.py` loads
      `magatzem_amb_trailer.yaml`, with the trailer collision painted as
      an obstacle: the lidar raycasts against the VISUAL and at its
      height only sees legs and wheels, not the front wall.

    DeixarPalet for the first pallet (`align_tag_id is None`, empty zone):
      BaixarPalet -> RetirarDefinitiva

    DeixarPalet for the following pallets (stacked on `align_tag_id`):
      BaixarPerAlinear -> AlinearAmbPila -> ReaproximarPerDeixar ->
      BaixarSobrePila -> RetirarDefinitiva
    - Nav2 only guarantees ~0.15 m of tolerance at `drop_pose`; without a
      correction each pallet would end up offset from the one below.
      `AlinearAmbPila` aligns with the tag of the pallet below (x, y and
      yaw) and `ReaproximarPerDeixar` drives blindly over the stack.
    - `NavDescarrega` stops `place_standoff` m BEFORE `drop_pose`: going
      to the literal pose would drive the robot into the pallet already
      placed there (Nav2 does not have it in its map).
    - `place_standoff=2.6` comes from tag visibility: below ~3.4 m
      base-tag, the carried pallet hides the tag of the one below. 3.9 m
      base-tag gives margin: 3.9 - 1.31 = 2.6 (1.31 m is the base-tag
      distance of the carried pallet, `standoff - insert_distance`)."""
    px, py_, pyaw = pickup_pose
    dx, dy, dyaw = drop_pose

    def phase(name, leaves):
        return py_trees.composites.Sequence(
            name=f'{name}{label}', memory=True, children=leaves)

    approach = [
        SetFootprint(f'FootprintDocking{label}', footprint=CHASSIS_FOOTPRINT),
        NavigateToPose(f'NavAproximacio{label}', x=px, y=py_, yaw=pyaw),
    ]
    locate = [
        EstimateHeightFromTag(f'EstimarAlcadaMastil{label}', tag_frame=f'mast_tag:{tag_id}',
                              blackboard_key='h_mastil', overall_timeout=120.0),
        VisualServoToTag(f'CentrarAmbMastil{label}', tag_id=tag_id, standoff=standoff,
                          tag_frame_prefix='mast_tag', min_dist=standoff,
                          tol_x=0.05, tol_y=0.05, tol_yaw=0.03,
                          overall_timeout=30.0),
        SetForkHeight(f'PujarAAlcadaEstimada{label}', height_key='h_mastil'),
        VisualServoToTag(f'ServoAprilTag{label}', tag_id=tag_id, standoff=standoff,
                          min_dist=standoff),
        EstimateHeightFromTag(f'ConfirmarAlcadaForquilla{label}', tag_frame=f'tag36h11:{tag_id}',
                              blackboard_key='h_confirmada'),
        CompareHeightEstimates(f'VerificarConcordanca{label}', key_a='h_mastil',
                               key_b='h_confirmada', tolerance=height_tolerance),
        SetForkHeight(f'AjustarAlcadaConfirmada{label}', height_key='h_confirmada'),
    ]
    pick = [
        CreepRelative(f'InserirPues{label}', distance=insert_distance),
        SetForkHeight(f'ElevarPalet{label}', height_key='h_confirmada', height_offset=lift_clearance),
        CreepRelative(f'RetirarPues{label}', distance=backout_distance),
        # Before lowering to TRANSPORT_HEIGHT, the tine tips (2.41 m ahead
        # of the body) must be clear of the trailer bed footprint: with
        # `backout_distance` alone ~30 cm are still inside and would hit it
        # when lowering.
        CreepRelative(f'AllunyarDelCamio{label}', distance=-0.7),
        SetForkHeight(f'BaixarPerTransport{label}', height=TRANSPORT_HEIGHT),
    ]
    transport = [
        SetFootprint(f'FootprintComplet{label}', footprint=FULL_FOOTPRINT),
    ]
    if align_tag_id is not None:
        # NavDescarrega stops `place_standoff` m before drop_pose, along the
        # approach direction (see docstring).
        nav_x = dx - place_standoff * math.cos(dyaw)
        nav_y = dy - place_standoff * math.sin(dyaw)
        # Base -> tag distance of the pallet BELOW when NavDescarrega ends:
        # place_standoff + the base-tag distance of the carried pallet
        # (standoff - insert_distance = 1.31 m). Servoing to
        # `place_standoff` alone would take the robot into the zone where
        # the carried pallet hides the tag.
        align_standoff = place_standoff + (standoff - insert_distance)
        transport.append(
            NavigateToPose(f'NavDescarrega{label}', x=nav_x, y=nav_y, yaw=dyaw))
        place = [
            # Align and approach blindly with vertical clearance
            # (`lift_clearance`) over the stack: at exactly `place_height`
            # any inaccuracy pushes the pallet below.
            SetForkHeight(f'BaixarPerAlinear{label}', height=place_height,
                          height_offset=lift_clearance),
            # Also corrects the distance (correct_x=True): otherwise Nav2's
            # error at drop_pose (up to ~0.15 m) would reach the stack whole.
            VisualServoToTag(f'AlinearAmbPila{label}', tag_id=align_tag_id,
                              standoff=align_standoff,
                              tol_x=0.01, tol_y=0.01, tol_yaw=0.02, overall_timeout=30.0),
            # From `align_standoff`, driving `place_standoff` forward leaves
            # the carried pallet exactly over the one below (same 1.31 m
            # base-tag relation as in the pick-up).
            CreepRelative(f'ReaproximarPerDeixar{label}', distance=place_standoff),
            # Stops `settle_clearance` (5 mm) ABOVE the stack: at 0 the tine
            # presses on the pallet below and makes it slide by friction;
            # with 2 cm the pallet does not come to rest cleanly. Gravity
            # takes care of the remaining gap when RetirarDefinitiva pulls
            # the tines out.
            SetForkHeight(f'BaixarSobrePila{label}', height=place_height,
                          height_offset=settle_clearance),
        ]
    else:
        transport.append(
            NavigateToPose(f'NavDescarrega{label}', x=dx, y=dy, yaw=dyaw))
        place = [
            SetForkHeight(f'BaixarPalet{label}', height=place_height),
        ]
    place.append(CreepRelative(f'RetirarDefinitiva{label}', distance=withdraw_distance))
    return py_trees.composites.Sequence(
        name=f'CicleDescarrega{label}', memory=True, children=[
            phase('AproximacioPalet', approach),
            phase('LocalitzarPalet', locate),
            phase('AgafarPalet', pick),
            phase('Transport', transport),
            phase('DeixarPalet', place),
        ])


# Thickness of a Euro pallet (models/palet_export/robot.sdf: 0.8x1.2x0.144 m):
# since `h` is literally elevation in metres (`tine base = 0.03 + h`, see
# height_to_joints), targeting i*PALLET_THICKNESS leaves the tine just
# above the pallet already stacked, instead of relying on the collision
# with the stack to stop the descent (which would also work through
# gravity, as on the floor, but would keep pushing against the stack).
PALLET_THICKNESS = 0.144


def build_mission(n_palets, standoff, insert_distance, lift_clearance, backout_distance,
                   withdraw_distance, drop_pose, base_pickup_pose, spacing=1.0,
                   height_tolerance=0.05):
    """Chains `n_palets` full cycles — one `build_tree()` per pallet
    (tag_id 0..n_palets-1), stacking them all at the SAME `drop_pose` with
    increasing `place_height`."""
    px, py_, pyaw = base_pickup_pose
    cicles = []
    for i in range(n_palets):
        pickup_pose = (px + i * spacing, py_, pyaw)
        cicles.append(build_tree(
            tag_id=i, standoff=standoff, insert_distance=insert_distance,
            lift_clearance=lift_clearance, backout_distance=backout_distance,
            place_height=i * PALLET_THICKNESS, withdraw_distance=withdraw_distance,
            pickup_pose=pickup_pose, drop_pose=drop_pose,
            height_tolerance=height_tolerance, label=f'_p{i}',
            # Only align with the pallet below if there is one (i>0): the
            # first pallet goes to an empty zone, nothing to align with.
            align_tag_id=(i - 1) if i > 0 else None))
    return py_trees.composites.Sequence(
        name='MissioTresPalets', memory=True, children=cicles)


def _stop_when_done(tree):
    """py_trees.composites.Sequence does NOT latch: once it reaches
    SUCCESS, the next tick restarts it from the first child (checked by
    reading Sequence.tick()). Without this handler the whole cycle would
    repeat on its own forever — and crash into the pallet it is already
    carrying. This is a one-shot mission: stop at a terminal state."""
    status = tree.root.status
    if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
        tree.node.get_logger().info(f'Cicle acabat amb estat {status}')
        tree.interrupt()
        rclpy.shutdown()


def build_default_mission():
    """The mission run by `main()` (3 pallets from the trailer at (6,3) in
    magatzem.sdf). Also used by docs/render_bt_diagram.py so the diagram
    always matches the real tree."""
    return build_mission(n_palets=3, standoff=2.5, insert_distance=1.19,
                         lift_clearance=0.10, backout_distance=-0.8,
                         withdraw_distance=-1.3,
                         base_pickup_pose=(7.0, 5.75, -math.pi / 2),
                         drop_pose=(2.0, -2.0, 0.0))


def main():
    sys.stdout.reconfigure(line_buffering=True)  # live logs when piped through `tee`
    rclpy.init()
    root = build_default_mission()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.add_post_tick_handler(_stop_when_done)
    try:
        tree.setup(timeout=15.0)
    except Exception as e:
        print(f'setup del BT fallit: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    tree.tick_tock(period_ms=100.0)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tree.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
