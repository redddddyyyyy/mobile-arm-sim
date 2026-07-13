#!/usr/bin/env python3
"""Autonomous pick-and-place mission driver.

A 5 Hz state machine: patrol search waypoints until block_detector reports
the target block, approach it with Nav2, then hand over to the arm sequence.
States the arm work hasn't reached yet log and pass through, so the mission
chain can be exercised end to end at any stage of wiring.
"""

import math
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray

# Arm poses (shoulder_pan, shoulder_lift, elbow, wrist) — the same joint
# targets the scripted pick_and_place demo uses.
REST      = [0.0, -0.5, 1.2, 0.3]
PRE_GRASP = [0.0,  0.6, 1.4, 0.5]
GRASP     = [0.0,  0.9, 1.6, 0.5]
LIFT      = [0.0,  0.0, 1.0, 0.3]
DROP      = [0.0,  0.6, 1.4, 0.5]

GRIPPER_OPEN = -0.015
GRIPPER_CLOSED = -0.005


class State(Enum):
    IDLE = auto()
    SEARCHING = auto()
    APPROACHING = auto()
    ALIGNING = auto()
    GRASPING = auto()
    CARRYING = auto()
    PLACING = auto()
    RETURNING = auto()
    DONE = auto()
    FAILED = auto()


# Plain pass-through order while states are stubs.
NEXT = {
    State.IDLE: State.SEARCHING,
    State.SEARCHING: State.APPROACHING,
    State.APPROACHING: State.ALIGNING,
    State.ALIGNING: State.GRASPING,
    State.GRASPING: State.CARRYING,
    State.CARRYING: State.PLACING,
    State.PLACING: State.RETURNING,
    State.RETURNING: State.DONE,
}

STUB_DWELL = 1.5  # seconds a stubbed state lingers before passing through

# Open-floor poses to search from, in the order visited. Each gets a full
# spin before moving on. The detector only reaches ~1.1 m (a 5 cm cube is
# a dozen pixels beyond that), so searching means going places, not just
# looking around from the start.
SEARCH_WAYPOINTS = [(0.7, 2.0), (-1.5, -0.3), (-1.6, 1.0)]

SPIN_SPEED = 0.4                                  # rad/s while scanning
SPIN_DURATION = 2 * math.pi / SPIN_SPEED * 1.2    # one revolution + slack
STANDOFF = 0.65    # m from block at approach goal: outside the camera's
                   # <0.45 m blind zone, inside detection range
FRESH_SEC = 1.0    # a detection older than this doesn't count as "in view"


class Orchestrator(Node):

    def __init__(self):
        # Mission timing and detection freshness all count in sim seconds;
        # on wall clock every timeout would silently drift. This node has
        # no life outside the sim, so force it rather than trust the CLI.
        super().__init__('autonomous_pick_place', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        self._block = None   # last PoseStamped from the detector, any age
        self.create_subscription(PoseStamped, '/target_block_pose',
                                 self._block_cb, 10)
        self._robot = None   # (x, y) from AMCL
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_status = 'idle'   # idle | active | succeeded | failed
        self._nav_gen = 0   # discards result callbacks from replaced goals

        # SEARCHING bookkeeping
        self._search_i = None       # waypoint index; None = not started
        self._search_phase = None   # 'nav' | 'spin'
        self._spin_started = None
        # APPROACHING bookkeeping
        self._target = None   # block pose frozen at the moment of sighting
        self._approach_sent = False
        self._approach_retried = False
        self._finished = False

        self._state = None
        self._t_enter = None
        self._set_state(State.IDLE)
        self.create_timer(0.2, self._tick)

    # ---------- inputs ----------

    def _block_cb(self, msg):
        self._block = msg

    def _amcl_cb(self, msg):
        p = msg.pose.pose.position
        self._robot = (p.x, p.y)

    def _block_fresh(self):
        if self._block is None:
            return False
        age = self.get_clock().now() - Time.from_msg(self._block.header.stamp)
        return age.nanoseconds * 1e-9 < FRESH_SEC

    # ---------- Nav2 plumbing (all non-blocking) ----------

    def _send_nav_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self._nav_status = 'active'
        self._nav_gen += 1
        gen = self._nav_gen
        self.get_logger().info(f'nav goal ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f} deg')
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda f: self._nav_accepted(f, gen))

    def _nav_accepted(self, future, gen):
        handle = future.result()
        if not handle.accepted:
            if gen == self._nav_gen:
                self._nav_status = 'failed'
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._nav_finished(f, gen))

    def _nav_finished(self, future, gen):
        if gen != self._nav_gen:
            return   # a goal we've since replaced; its outcome is history
        ok = future.result().status == GoalStatus.STATUS_SUCCEEDED
        self._nav_status = 'succeeded' if ok else 'failed'

    # ---------- state plumbing ----------

    def _set_state(self, new):
        if self._state is not None:
            self.get_logger().info(f'{self._state.name} -> {new.name}')
        self._state = new
        self._t_enter = self.get_clock().now()

    def _elapsed(self):
        return (self.get_clock().now() - self._t_enter).nanoseconds * 1e-9

    def _tick(self):
        getattr(self, f'_on_{self._state.name.lower()}')()

    def _stub(self):
        if self._elapsed() > STUB_DWELL:
            self._set_state(NEXT[self._state])

    # ---------- state handlers ----------

    def _on_idle(self):
        self._stub()

    def _on_searching(self):
        # A fresh detection trumps everything, including a nav goal in
        # flight — the goal keeps running but we stop feeding the search.
        if self._block_fresh():
            self.stop_base()
            # A patrol goal may still be in flight here. It is deliberately
            # not cancelled: bt_navigator runs one goal at a time, so the
            # approach goal preempts it, and the generation counter drops
            # the stale result it reports back.
            # Freeze the estimate: APPROACHING must chase one fixed target,
            # not follow the live topic wherever it wanders next.
            self._target = self._block
            self.get_logger().info(
                f'block spotted at ({self._target.pose.position.x:.2f}, '
                f'{self._target.pose.position.y:.2f})')
            self._set_state(State.APPROACHING)
            return

        if self._search_i is None:
            self._search_i = 0
            self._search_phase = 'nav'
            self._nav_status = 'idle'

        if self._search_phase == 'nav':
            if self._nav_status == 'idle':
                if not self.nav_client.server_is_ready():
                    # Say so — a stalled Nav2 bringup once cost minutes of
                    # silent head-scratching.
                    self.get_logger().warning('waiting for the Nav2 action server',
                                              throttle_duration_sec=5.0)
                    return
                x, y = SEARCH_WAYPOINTS[self._search_i]
                self._send_nav_goal(x, y, 0.0)
            elif self._nav_status == 'succeeded':
                self._search_phase = 'spin'
                self._spin_started = self.get_clock().now()
            elif self._nav_status == 'failed':
                self.get_logger().warning(
                    f'could not reach search waypoint {self._search_i}, skipping')
                self._next_waypoint()
        else:  # spin
            spun = (self.get_clock().now() - self._spin_started).nanoseconds * 1e-9
            if spun < SPIN_DURATION:
                cmd = Twist()
                cmd.angular.z = SPIN_SPEED
                self.cmd_pub.publish(cmd)
            else:
                self.stop_base()
                self._next_waypoint()

    def _next_waypoint(self):
        self._search_i += 1
        if self._search_i >= len(SEARCH_WAYPOINTS):
            self.get_logger().error('searched every waypoint, no block found')
            self._set_state(State.FAILED)
            return
        self._search_phase = 'nav'
        self._nav_status = 'idle'

    def _approach_goal(self, swing=0.0):
        """Standoff pose STANDOFF metres from the block, facing it.

        Approach along the block->robot line so the robot comes in from
        wherever it already is; `swing` rotates that line around the block
        for the retry.
        """
        bx = self._target.pose.position.x
        by = self._target.pose.position.y
        rx, ry = self._robot if self._robot else (bx - 1.0, by)
        ang = math.atan2(ry - by, rx - bx) + swing
        sx = bx + STANDOFF * math.cos(ang)
        sy = by + STANDOFF * math.sin(ang)
        return sx, sy, math.atan2(by - sy, bx - sx)

    def _on_approaching(self):
        # Works from the cached block pose on purpose: inside 0.45 m the
        # camera can't see the block at all, so detection dropping out on
        # final approach is expected, not an error.
        if not self._approach_sent:
            if not self.nav_client.server_is_ready():
                return
            x, y, yaw = self._approach_goal()
            self._send_nav_goal(x, y, yaw)
            self._approach_sent = True
        elif self._nav_status == 'succeeded':
            self.stop_base()
            self._set_state(State.ALIGNING)
        elif self._nav_status == 'failed':
            if not self._approach_retried:
                self.get_logger().warning('approach aborted, retrying from 30 deg around')
                x, y, yaw = self._approach_goal(swing=math.radians(30))
                self._send_nav_goal(x, y, yaw)
                self._approach_retried = True
            else:
                self.get_logger().error('approach failed twice')
                self._set_state(State.FAILED)

    def _on_aligning(self):
        self._stub()

    def _on_grasping(self):
        self._stub()

    def _on_carrying(self):
        self._stub()

    def _on_placing(self):
        self._stub()

    def _on_returning(self):
        self._stub()

    def _on_done(self):
        if not self._finished:
            self._finished = True
            self.stop_base()
            b = self._target.pose.position if self._target else None
            where = f'({b.x:.2f}, {b.y:.2f})' if b else 'never seen'
            self.get_logger().info(f'mission complete — target block at {where}')

    def _on_failed(self):
        if not self._finished:
            self._finished = True
            self.stop_base()
            self.get_logger().error('mission failed — see log above for the state that gave up')

    # ---------- actuators ----------

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def arm(self, positions):
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self.arm_pub.publish(msg)

    def gripper(self, opening):
        msg = Float64MultiArray()
        msg.data = [float(opening), float(opening)]
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_base()
        except Exception:
            pass   # context may already be torn down on Ctrl-C
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
