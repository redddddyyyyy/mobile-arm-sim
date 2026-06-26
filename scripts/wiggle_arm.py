#!/usr/bin/env python3
"""Quick joint wiggler - publishes a sinusoidal joint_states message.

This bypasses joint_state_publisher_gui so you can see the arm move on its own.
Use this in RViz mode (display.launch.py) to verify the joints animate.
For Gazebo control of the arm, you'll want ros2_control instead - that's the next step.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmWiggler(Node):
    def __init__(self):
        super().__init__('arm_wiggler')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.t = 0.0
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        self.t += 0.05
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint',
            'left_finger_joint',
            'right_finger_joint',
            # wheels included so RViz doesn't complain about missing joints
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        msg.position = [
            0.8 * math.sin(self.t * 0.5),          # shoulder pan
            0.5 * math.sin(self.t * 0.7) - 0.3,    # shoulder lift
            1.0 * math.sin(self.t * 0.9),          # elbow
            0.7 * math.sin(self.t * 1.1),          # wrist
            -0.01 + 0.005 * math.sin(self.t),      # left finger
            -0.01 + 0.005 * math.sin(self.t),      # right finger
            self.t,                                # wheels spin
            self.t,
            self.t,
            self.t,
        ]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArmWiggler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
