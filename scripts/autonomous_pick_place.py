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
import tf2_ros
from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
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


IDLE_DWELL = 1.5  # settle time before the mission starts

# Open-floor poses to search from, in the order visited. Each gets a full
# spin before moving on. The detector only reaches ~1.1 m (a 5 cm cube is
# a dozen pixels beyond that), so searching means going places, not just
# looking around from the start.
SEARCH_WAYPOINTS = [(0.7, 2.0), (-1.5, -0.3), (-6.9, -0.1)]

SPIN_SPEED = 0.4                                  # rad/s while scanning
SPIN_DURATION = 2 * math.pi / SPIN_SPEED * 1.2    # one revolution + slack
STANDOFF = 0.65    # m from block at approach goal: outside the camera's
                   # <0.45 m blind zone, inside detection range
FRESH_SEC = 1.0    # a detection older than this doesn't count as "in view"

# The fixed arm poses reach a block ~0.35 m ahead of base centre (the old
# scripted demo: block 1.0 m out, drove 0.65). ALIGNING creeps the gap
# between the camera standoff and that reach, blind, on odometry — the
# camera can't see the block this close anyway.
GRASP_REACH = 0.35
TABLE_XY = (1.5, -1.5)   # matches the spawn in the launch file
TABLE_STANDOFF = 0.75    # nav goal distance from table centre
TABLE_REACH = 0.40       # DROP extends over a table this far ahead
HOME_XY = (4.3, -1.5)    # open floor next to the spawn point


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
        self._robot = None   # (x, y, yaw) from AMCL
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._odom = None    # (x, y, yaw) — smooth feedback for blind creeps
        self.create_subscription(Odometry, '/odom', self._odom_cb,
                                 qos_profile_sensor_data)

        # Magic grasp: while attached, a 20 Hz timer teleports the block to
        # the gripper. The gripper is looked up in the MAP frame — the old
        # scripted demo used odom, which only equalled world because that
        # robot spawned at the origin. Ours doesn't.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        self._attached = False
        self.create_timer(0.05, self._teleport_block)

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
        self._verify_until = None
        self._finished = False
        # ALIGNING / CARRYING / arm-sequence bookkeeping
        self._align = None    # dict: phase, goal yaw / drive length, odom start
        self._carry_phase = None   # 'nav' | 'rotate' | 'creep'
        self._stages = None   # [(callable, dwell_sec), ...] for arm scripts
        self._stage_i = 0
        self._stage_t = None

        self._state = None
        self._t_enter = None
        self._set_state(State.IDLE)
        self.create_timer(0.2, self._tick)

    # ---------- inputs ----------

    def _block_cb(self, msg):
        self._block = msg

    def _amcl_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self._robot = (p.x, p.y, yaw)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self._odom = (p.x, p.y, yaw)

    def _teleport_block(self):
        if not self._attached or not self._set_state_cli.service_is_ready():
            return
        try:
            t = self._tf_buffer.lookup_transform('map', 'gripper_base',
                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        req = SetEntityState.Request()
        req.state.name = 'target_block'
        req.state.pose.position.x = t.transform.translation.x
        req.state.pose.position.y = t.transform.translation.y
        req.state.pose.position.z = t.transform.translation.z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self._set_state_cli.call_async(req)

    # ---------- staged arm scripts ----------

    def _start_stages(self, stages):
        self._stages = stages
        self._stage_i = -1
        self._stage_t = None

    def _run_stages(self):
        """Advance the staged script; returns True when the last dwell ends."""
        now = self.get_clock().now()
        if self._stage_t is None or \
           (now - self._stage_t).nanoseconds * 1e-9 >= self._stages[self._stage_i][1]:
            self._stage_i += 1
            if self._stage_i >= len(self._stages):
                self._stages = None
                return True
            fn, _ = self._stages[self._stage_i]
            fn()
            self._stage_t = now
        return False

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
        self._state_inited = False   # handlers run their entry setup once

    def _elapsed(self):
        return (self.get_clock().now() - self._t_enter).nanoseconds * 1e-9

    def _tick(self):
        getattr(self, f'_on_{self._state.name.lower()}')()

    # ---------- state handlers ----------

    def _on_idle(self):
        if self._elapsed() > IDLE_DWELL:
            self._set_state(State.SEARCHING)

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
        rx, ry = self._robot[:2] if self._robot else (bx - 1.0, by)
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
        elif self._verify_until is not None:
            # At the standoff the block sits centred at ~0.65 m — prime
            # viewing. A sighting made mid-drive can be a metre off (AMCL
            # lags while rotating), so re-freeze on a close-up if we get
            # one before lining up the grasp.
            if self._block_fresh():
                self._target = self._block
                b = self._target.pose.position
                self.get_logger().info(f'target confirmed up close at ({b.x:.2f}, {b.y:.2f})')
                self._set_state(State.ALIGNING)
            elif self.get_clock().now() > self._verify_until:
                self.get_logger().warning('no close-up confirmation, trusting the first sighting')
                self._set_state(State.ALIGNING)
        elif self._nav_status == 'succeeded':
            self.stop_base()
            self._verify_until = self.get_clock().now() + Duration(seconds=4.0)
        elif self._nav_status == 'failed':
            if not self._approach_retried:
                self.get_logger().warning('approach aborted, retrying from 30 deg around')
                x, y, yaw = self._approach_goal(swing=math.radians(30))
                self._send_nav_goal(x, y, yaw)
                self._approach_retried = True
            else:
                self.get_logger().error('approach failed twice')
                self._set_state(State.FAILED)

    # ---------- rotate-and-creep (shared by block and table lineups) ----------

    def _begin_align(self, tx, ty, reach):
        """Face (tx, ty), then creep forward until it sits `reach` m ahead.

        The rotation error is measured once against AMCL, then executed on
        odometry — odom is smooth and instant while AMCL updates arrive in
        chunky steps that would make a feedback loop hunt.
        """
        rx, ry, ryaw = self._robot
        bearing = math.atan2(ty - ry, tx - rx)
        err = (bearing - ryaw + math.pi) % (2 * math.pi) - math.pi
        self._align = {
            'phase': 'rotate',
            'goal_yaw': self._odom[2] + err,
            'drive': max(math.hypot(tx - rx, ty - ry) - reach, 0.0),
            'start': None,
            'deadline': self.get_clock().now() + Duration(seconds=40.0),
        }
        self.get_logger().info(
            f'lining up: rotate {math.degrees(err):+.0f} deg, '
            f'creep {self._align["drive"]:.2f} m')

    def _run_align(self):
        """Advance the lineup; True when done, None while working."""
        if self.get_clock().now() > self._align['deadline']:
            self.stop_base()
            self.get_logger().error('lineup timed out')
            self._set_state(State.FAILED)
            return False
        a = self._align
        if a['phase'] == 'rotate':
            err = (a['goal_yaw'] - self._odom[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(err) > 0.03:
                cmd = Twist()
                cmd.angular.z = 0.3 if err > 0 else -0.3
                self.cmd_pub.publish(cmd)
            else:
                self.stop_base()
                a['phase'] = 'creep'
                a['start'] = self._odom
            return False
        moved = math.hypot(self._odom[0] - a['start'][0],
                           self._odom[1] - a['start'][1])
        if moved < a['drive']:
            cmd = Twist()
            cmd.linear.x = 0.1
            self.cmd_pub.publish(cmd)
            return False
        self.stop_base()
        return True

    # ---------- mission states ----------

    def _on_aligning(self):
        if not self._state_inited:
            self._state_inited = True
            if self._robot is None or self._odom is None:
                self._state_inited = False
                return
            b = self._target.pose.position
            self._begin_align(b.x, b.y, GRASP_REACH)
        if self._run_align():
            self._set_state(State.GRASPING)

    def _attach(self):
        self._attached = True
        self.get_logger().info('magic grasp engaged — block follows the gripper')

    def _detach(self):
        self._attached = False
        self.get_logger().info('released')

    def _on_grasping(self):
        if not self._state_inited:
            self._state_inited = True
            self._start_stages([
                (lambda: self.gripper(GRIPPER_OPEN), 0.5),
                (lambda: self.arm(PRE_GRASP), 2.5),
                (lambda: self.arm(GRASP), 2.0),
                (lambda: self.gripper(GRIPPER_CLOSED), 0.6),
                (self._attach, 0.5),
                (lambda: self.arm(LIFT), 2.0),
            ])
        if self._run_stages():
            self._set_state(State.CARRYING)

    def _on_carrying(self):
        if not self._state_inited:
            self._state_inited = True
            self._carry_phase = 'nav'
            self._carry_retried = False
            tx, ty = TABLE_XY
            rx, ry, _ = self._robot
            ang = math.atan2(ry - ty, rx - tx)
            gx = tx + TABLE_STANDOFF * math.cos(ang)
            gy = ty + TABLE_STANDOFF * math.sin(ang)
            self._send_nav_goal(gx, gy, math.atan2(ty - gy, tx - gx))
        if self._carry_phase == 'nav':
            if self._nav_status == 'succeeded':
                self._carry_phase = 'lineup'
                self._begin_align(*TABLE_XY, TABLE_REACH)
            elif self._nav_status == 'failed':
                if not self._carry_retried:
                    self._carry_retried = True
                    tx, ty = TABLE_XY
                    rx, ry, _ = self._robot
                    ang = math.atan2(ry - ty, rx - tx) + math.radians(40)
                    gx = tx + TABLE_STANDOFF * math.cos(ang)
                    gy = ty + TABLE_STANDOFF * math.sin(ang)
                    self.get_logger().warning('table approach aborted, retrying from around')
                    self._send_nav_goal(gx, gy, math.atan2(ty - gy, tx - gx))
                else:
                    self.get_logger().error('could not reach the table')
                    self._set_state(State.FAILED)
        elif self._run_align():
            self._set_state(State.PLACING)

    def _on_placing(self):
        if not self._state_inited:
            self._state_inited = True
            self._start_stages([
                (lambda: self.arm(DROP), 3.0),
                (lambda: self.gripper(GRIPPER_OPEN), 0.4),
                (self._detach, 1.5),
                (lambda: self.arm(LIFT), 2.0),
                (lambda: self.arm(REST), 2.0),
            ])
        if self._run_stages():
            self._set_state(State.RETURNING)

    def _on_returning(self):
        if not self._state_inited:
            self._state_inited = True
            self._send_nav_goal(*HOME_XY, math.pi)
        if self._nav_status == 'succeeded':
            self._set_state(State.DONE)
        elif self._nav_status == 'failed':
            # The block is already on the table; a scuffed victory lap
            # doesn't fail the mission.
            self.get_logger().warning('could not drive home, finishing here')
            self._set_state(State.DONE)

    def _on_done(self):
        if not self._finished:
            self._finished = True
            self.stop_base()
            b = self._target.pose.position if self._target else None
            where = f'found at ({b.x:.2f}, {b.y:.2f}) and' if b else 'never seen,'
            self.get_logger().info(
                f'mission complete — target block {where} delivered to the table')

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
