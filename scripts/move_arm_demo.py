#!/usr/bin/env python3
"""Cycle the arm through a few poses to confirm controllers are working.

Usage (after `ros2 launch mobile_arm_sim gazebo.launch.py` is running):
    ros2 run mobile_arm_sim move_arm_demo.py
        or
    python3 ~/ros2_ws/src/mobile_arm_sim/scripts/move_arm_demo.py
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Finger limits per URDF: -0.015 (open) .. 0.005 (closed-into-each-other)
GRIPPER_OPEN   = -0.015
GRIPPER_CLOSED = -0.005


class ArmDemo(Node):
    def __init__(self):
        super().__init__('arm_demo')
        self.arm_pub  = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

    def send_arm(self, positions):
        msg = Float64MultiArray()
        msg.data = list(positions)
        self.arm_pub.publish(msg)
        self.get_logger().info(f'arm  -> [{", ".join(f"{p:+.2f}" for p in positions)}]')

    def send_gripper(self, finger_pos):
        msg = Float64MultiArray()
        msg.data = [finger_pos, finger_pos]
        self.grip_pub.publish(msg)
        state = 'OPEN' if finger_pos < -0.012 else 'CLOSED'
        self.get_logger().info(f'grip -> {finger_pos:+.3f} ({state})')


def main():
    rclpy.init()
    node = ArmDemo()
    time.sleep(2.0)  # let publishers discover the controller subscribers

    # Sequence: (shoulder_pan, shoulder_lift, elbow, wrist), gripper
    sequence = [
        (( 0.0, -0.5,  1.2,  0.3), GRIPPER_OPEN),    # rest pose
        (( 1.2, -0.5,  1.2,  0.3), GRIPPER_OPEN),    # turn left
        ((-1.2, -0.5,  1.2,  0.3), GRIPPER_OPEN),    # turn right
        (( 0.0,  0.0,  0.0,  0.0), GRIPPER_OPEN),    # straight up
        (( 0.0, -1.0,  2.0,  0.5), GRIPPER_CLOSED),  # reach forward + close gripper
        (( 0.0, -0.5,  1.2,  0.3), GRIPPER_OPEN),    # back to rest
    ]

    for arm_pose, grip in sequence:
        node.send_arm(arm_pose)
        node.send_gripper(grip)
        time.sleep(3.0)

    node.get_logger().info('Demo complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
