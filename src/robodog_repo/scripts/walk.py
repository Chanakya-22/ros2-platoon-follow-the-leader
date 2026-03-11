#!/usr/bin/env python3
"""
RoboDog Trot Gait Controller

Publishes sinusoidal joint position commands to make the robo dog
walk using a trot gait (diagonal legs move in sync).

Gait phases:
  Phase A (front_left + rear_right) — move together
  Phase B (front_right + rear_left) — 180° offset from Phase A
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class WalkNode(Node):
    def __init__(self):
        super().__init__('walk_node')

        # Gait parameters (declare as ROS params for easy tuning)
        self.declare_parameter('gait_frequency', 1.5)       # Hz — step cycles per second
        self.declare_parameter('hip_amplitude', 0.35)        # rad — hip swing range
        self.declare_parameter('knee_amplitude', 0.5)        # rad — knee bend depth
        self.declare_parameter('knee_offset', -0.3)          # rad — keep knees slightly bent
        self.declare_parameter('update_rate', 50.0)          # Hz — control loop rate

        self.freq = self.get_parameter('gait_frequency').value
        self.hip_amp = self.get_parameter('hip_amplitude').value
        self.knee_amp = self.get_parameter('knee_amplitude').value
        self.knee_off = self.get_parameter('knee_offset').value
        rate = self.get_parameter('update_rate').value

        # Joint command publishers (ROS2 → bridged to Gazebo)
        self.joint_pubs = {}
        joint_names = [
            'front_left_hip_joint',
            'front_left_knee_joint',
            'front_right_hip_joint',
            'front_right_knee_joint',
            'rear_left_hip_joint',
            'rear_left_knee_joint',
            'rear_right_hip_joint',
            'rear_right_knee_joint',
            'head_joint',
        ]
        for name in joint_names:
            topic = f'/model/robodog/joint/{name}/cmd_pos'
            self.joint_pubs[name] = self.create_publisher(Float64, topic, 10)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f'Walking gait started — freq={self.freq}Hz, '
            f'hip_amp={self.hip_amp}rad, knee_amp={self.knee_amp}rad'
        )

    def publish_cmd(self, joint_name: str, value: float):
        msg = Float64()
        msg.data = value
        self.joint_pubs[joint_name].publish(msg)

    def timer_callback(self):
        omega = 2.0 * math.pi * self.freq
        phase = omega * self.t

        # ---- TROT GAIT ----
        # Phase A: front_left + rear_right
        # Phase B: front_right + rear_left  (180° offset)

        # Hip: sinusoidal swing (positive = forward, negative = backward)
        hip_a = self.hip_amp * math.sin(phase)
        hip_b = self.hip_amp * math.sin(phase + math.pi)

        # Knee: lift leg during swing phase, straighten during stance
        # Uses abs(sin) so the knee always stays in its negative range
        knee_a = self.knee_off - self.knee_amp * max(0.0, math.sin(phase))
        knee_b = self.knee_off - self.knee_amp * max(0.0, math.sin(phase + math.pi))

        # Clamp knees to joint limits [-1.2, 0.0]
        knee_a = max(-1.2, min(0.0, knee_a))
        knee_b = max(-1.2, min(0.0, knee_b))

        # Phase A legs
        self.publish_cmd('front_left_hip_joint', hip_a)
        self.publish_cmd('front_left_knee_joint', knee_a)
        self.publish_cmd('rear_right_hip_joint', hip_a)
        self.publish_cmd('rear_right_knee_joint', knee_a)

        # Phase B legs
        self.publish_cmd('front_right_hip_joint', hip_b)
        self.publish_cmd('front_right_knee_joint', knee_b)
        self.publish_cmd('rear_left_hip_joint', hip_b)
        self.publish_cmd('rear_left_knee_joint', knee_b)

        # Head stays centered
        self.publish_cmd('head_joint', 0.0)

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = WalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
