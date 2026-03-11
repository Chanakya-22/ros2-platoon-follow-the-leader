#!/usr/bin/env python3
"""
RoboDog cmd_vel Gait Controller

Listens to /cmd_vel (Twist) and publishes sinusoidal joint position commands
to make the robo dog walk using a trot gait.

Features:
- Forward/backward motion controlled by linear.x
- Turning controlled by angular.z
- Dog completely stops and returns to neutral standing when cmd_vel is zero
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class WalkCmdVelNode(Node):
    def __init__(self):
        super().__init__('walk_cmd_vel_node')

        # Connect to cmd_vel within the node's namespace (leader or follower)
        # Using simply 'cmd_vel' uses the namespace pushed by the launch file
        # But we need to make sure the topic isn't remapped away
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Base Gait Parameters
        self.declare_parameter('base_gait_frequency', 1.5)       # Hz
        self.declare_parameter('base_hip_amplitude', 0.5)        # rad (Increased stride)
        self.declare_parameter('knee_amplitude', 0.8)            # rad (Increased knee lift)
        self.declare_parameter('knee_offset', -0.3)              # rad
        self.declare_parameter('update_rate', 50.0)              # Hz

        self.base_freq = self.get_parameter('base_gait_frequency').value
        self.base_hip_amp = self.get_parameter('base_hip_amplitude').value
        self.knee_amp = self.get_parameter('knee_amplitude').value
        self.knee_off = self.get_parameter('knee_offset').value
        rate = self.get_parameter('update_rate').value

        # Current Velocity Commands
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Namespace awareness for the bridge topics
        # Assuming the robot's namespace is given or it's hardcoded to '/model/robodog/joint/'
        # Note: If we have multiple robots, we use the node's namespace.
        ns = self.get_namespace()
        if ns == '/':
            ns = ''
        
        # We need the Gazebo model name. We'll pass this as a parameter.
        self.declare_parameter('gz_model_name', 'robodog')
        self.gz_model_name = self.get_parameter('gz_model_name').value

        # Joint command publishers
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
            topic = f'/model/{self.gz_model_name}/joint/{name}/cmd_pos'
            self.joint_pubs[name] = self.create_publisher(Float64, topic, 10)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f'[{self.gz_model_name}] Walk cmd_vel controller started')

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def publish_cmd(self, joint_name: str, value: float):
        msg = Float64()
        msg.data = value
        self.joint_pubs[joint_name].publish(msg)

    def timer_callback(self):
        # 1. Check if moving
        is_moving = abs(self.linear_x) > 0.001 or abs(self.angular_z) > 0.001

        if not is_moving:
            # Stand still
            self.t = 0.0
            for name in self.joint_pubs.keys():
                if 'knee' in name:
                    self.publish_cmd(name, self.knee_off)
                else:
                    self.publish_cmd(name, 0.0)
            return

        # 2. Calculate Gait Parameters based on velocity
        # Frequency scales with linear speed
        current_freq = self.base_freq * min(abs(self.linear_x) * 2.5, 2.0)
        if current_freq < 0.5:
            current_freq = 0.5 # Minimum frequency when moving
        
        # Direction
        direction = 1.0 if self.linear_x >= 0 else -1.0

        omega = 2.0 * math.pi * current_freq
        phase = omega * self.t

        # Turn logic: Adjust hip amplitude on left vs right side
        # Make turning more aggressive for skid-steering
        turn_factor = self.angular_z * 0.8 
        
        # When spinning in place, one side goes purely backward and one purely forward
        fwd_speed = -self.base_hip_amp * direction if abs(self.linear_x) > 0.05 else 0.0
        
        hip_amp_left = fwd_speed + turn_factor
        hip_amp_right = fwd_speed - turn_factor

        # ---- TROT GAIT ----
        # Phase A: front_left + rear_right
        # Phase B: front_right + rear_left  (180° offset)

        hip_a_left = hip_amp_left * math.sin(phase)
        hip_b_left = hip_amp_left * math.sin(phase + math.pi)

        hip_a_right = hip_amp_right * math.sin(phase)
        hip_b_right = hip_amp_right * math.sin(phase + math.pi)

        # Knee
        knee_a = self.knee_off - self.knee_amp * max(0.0, math.sin(phase))
        knee_b = self.knee_off - self.knee_amp * max(0.0, math.sin(phase + math.pi))

        # Clamp knees
        knee_a = max(-1.2, min(0.0, knee_a))
        knee_b = max(-1.2, min(0.0, knee_b))

        # Phase A legs
        self.publish_cmd('front_left_hip_joint', hip_a_left)
        self.publish_cmd('front_left_knee_joint', knee_a)
        self.publish_cmd('rear_right_hip_joint', hip_a_right)
        self.publish_cmd('rear_right_knee_joint', knee_a)

        # Phase B legs
        self.publish_cmd('front_right_hip_joint', hip_b_right)
        self.publish_cmd('front_right_knee_joint', knee_b)
        self.publish_cmd('rear_left_hip_joint', hip_b_left)
        self.publish_cmd('rear_left_knee_joint', knee_b)

        # Head stays centered
        self.publish_cmd('head_joint', 0.0)

        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = WalkCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
