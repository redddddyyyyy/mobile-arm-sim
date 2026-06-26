#!/usr/bin/env python3
"""Pick-and-place with closed-loop odom driving + magic grasp.

Highlights:
- drive_distance() drives until robot has actually moved X meters per /odom
- rotate_by() rotates until robot has actually turned X radians
- magic grasp: block teleports to gripper while attached (Gazebo gripper physics
  with small objects is unreliable, so we cheat at the contact moment)
"""
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from tf2_ros import Buffer, TransformListener, TransformException


# Arm poses: (shoulder_pan, shoulder_lift, elbow, wrist)
# Pan stays 0 throughout — base does the orienting work
REST       = [ 0.0, -0.5,  1.2,  0.3]
PRE_GRASP  = [ 0.0,  0.6,  1.4,  0.5]
GRASP      = [ 0.0,  0.9,  1.6,  0.5]
LIFT       = [ 0.0,  0.0,  1.0,  0.3]
DROP       = [ 0.0,  0.6,  1.4,  0.5]   # similar to pre_grasp but extends over table

GRIPPER_OPEN   = -0.015
GRIPPER_CLOSED = -0.005


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def shortest_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.arm_pub  = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')

        self.last_odom = None
        self.block_attached = False

    # ---------- odom ----------
    def _odom_cb(self, msg):
        self.last_odom = msg

    def _pose(self):
        if self.last_odom is None:
            return None
        p = self.last_odom.pose.pose.position
        yaw = quat_to_yaw(self.last_odom.pose.pose.orientation)
        return (p.x, p.y, yaw)

    def _wait_for_odom(self):
        while self._pose() is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ---------- arm + gripper ----------
    def arm(self, positions):
        msg = Float64MultiArray(); msg.data = list(positions)
        self.arm_pub.publish(msg)
        self.get_logger().info(f'  arm  -> [{", ".join(f"{p:+.2f}" for p in positions)}]')

    def gripper(self, opening):
        msg = Float64MultiArray(); msg.data = [opening, opening]
        self.grip_pub.publish(msg)
        self.get_logger().info(f'  grip -> {"OPEN" if opening < -0.012 else "CLOSED"}')

    # ---------- driving (closed-loop) ----------
    def drive_distance(self, distance, vx=0.15, vy=0.0, timeout=20.0):
        """Drive until robot has traveled `distance` meters. vx<0 = backward, vy = strafe."""
        self._wait_for_odom()
        start_x, start_y, _ = self._pose()
        twist = Twist(); twist.linear.x = vx; twist.linear.y = vy
        direction = 'fwd' if vx > 0 else ('back' if vx < 0 else 'strafe')
        self.get_logger().info(f'  drive {direction} {distance:.2f}m (vx={vx:+.2f}, vy={vy:+.2f})')

        end_time = time.time() + timeout
        while time.time() < end_time:
            cur = self._pose()
            if cur is not None:
                dx = cur[0] - start_x
                dy = cur[1] - start_y
                traveled = math.hypot(dx, dy)
                if traveled >= distance:
                    break
            self.cmd_pub.publish(twist)
            self._tick(0.02)
        self.cmd_pub.publish(Twist())
        self._tick(0.3)  # let it settle

    def rotate_by(self, delta_yaw, wz=0.6, timeout=15.0):
        """Rotate by `delta_yaw` radians from current heading. + = CCW (left turn)."""
        self._wait_for_odom()
        _, _, start_yaw = self._pose()
        target_yaw = shortest_angle(start_yaw + delta_yaw)
        self.get_logger().info(f'  rotate {math.degrees(delta_yaw):+.0f}deg  (target yaw {math.degrees(target_yaw):+.0f}deg)')

        end_time = time.time() + timeout
        wz_sign = 1.0 if delta_yaw > 0 else -1.0
        while time.time() < end_time:
            cur = self._pose()
            if cur is not None:
                err = shortest_angle(target_yaw - cur[2])
                if abs(err) < 0.025:   # ~1.5 deg tolerance
                    break
                # Slow down near goal
                speed = wz if abs(err) > 0.3 else wz * 0.35
                twist = Twist(); twist.angular.z = wz_sign * speed
                self.cmd_pub.publish(twist)
            self._tick(0.02)
        self.cmd_pub.publish(Twist())
        self._tick(0.3)

    # ---------- magic grasp ----------
    def attach_block(self):
        self.block_attached = True
        self.get_logger().info('  *** MAGIC GRASP: block follows gripper ***')

    def detach_block(self):
        self.block_attached = False
        self.get_logger().info('  *** RELEASED ***')

    def _teleport_block(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'gripper_base', rclpy.time.Time())
        except TransformException:
            return
        if not self.set_state_cli.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = 'target_block'
        req.state.pose.position.x = t.transform.translation.x
        req.state.pose.position.y = t.transform.translation.y
        req.state.pose.position.z = t.transform.translation.z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.set_state_cli.call_async(req)

    # ---------- helpers ----------
    def _tick(self, duration):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.block_attached:
                self._teleport_block()

    def wait(self, duration):
        self._tick(duration)


def step(node, label):
    node.get_logger().info(f'>>> {label}')


def main():
    rclpy.init()
    node = PickAndPlace()

    node.get_logger().info('Waiting for /set_entity_state service...')
    node.set_state_cli.wait_for_service(timeout_sec=5.0)
    node.get_logger().info('Waiting for /odom...')
    node._wait_for_odom()
    node.wait(1.5)  # let TF buffer fill

    node.get_logger().info('============ MOBILE PICK AND PLACE ============')

    step(node, '1. Start: rest pose, gripper open')
    node.arm(REST); node.gripper(GRIPPER_OPEN); node.wait(2)

    step(node, '2. Drive forward 0.65m to approach block')
    node.drive_distance(0.65, vx=0.15)

    step(node, '3. Arm to pre-grasp (hover above block)')
    node.arm(PRE_GRASP); node.wait(2.5)

    step(node, '4. Lower arm onto block')
    node.arm(GRASP); node.wait(2)

    step(node, '5. Close gripper + engage magic grasp')
    node.gripper(GRIPPER_CLOSED); node.wait(0.5)
    node.attach_block(); node.wait(0.7)

    step(node, '6. Lift arm with block')
    node.arm(LIFT); node.wait(2)

    step(node, '7. Pivot 90 deg LEFT in place')
    node.rotate_by(math.pi / 2)

    step(node, '8. Drive forward 0.4m to reach table')
    node.drive_distance(0.40, vx=0.13)

    step(node, '9. Arm extends out over table')
    node.arm(DROP); node.wait(3)

    step(node, '10. Open gripper - block drops onto table')
    node.gripper(GRIPPER_OPEN); node.wait(0.3)
    node.detach_block(); node.wait(2)

    step(node, '11. Lift arm back up')
    node.arm(LIFT); node.wait(2)

    step(node, '12. Drive backward 0.4m')
    node.drive_distance(0.40, vx=-0.13)

    step(node, '13. Pivot 90 deg RIGHT (back to original heading)')
    node.rotate_by(-math.pi / 2)

    step(node, '14. Drive backward 0.65m to home')
    node.drive_distance(0.65, vx=-0.15)

    step(node, '15. Return to rest pose')
    node.arm(REST); node.wait(2)

    node.get_logger().info('============ DEMO COMPLETE — back at start ============')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
