import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class LeaderController(Node):

    def __init__(self):
        super().__init__('leader_controller')

        self.wheel_separation = 0.30
        self.wheel_radius = 0.06

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.left_wheel_pub = self.create_publisher(
            Float64,
            '/left_wheel_joint/velocity',
            10
        )

        self.right_wheel_pub = self.create_publisher(
            Float64,
            '/right_wheel_joint/velocity',
            10
        )

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        v_left = (v - omega * self.wheel_separation / 2.0) / self.wheel_radius
        v_right = (v + omega * self.wheel_separation / 2.0) / self.wheel_radius

        self.left_wheel_pub.publish(Float64(data=v_left))
        self.right_wheel_pub.publish(Float64(data=v_right))

def main():
    rclpy.init()
    node = LeaderController()
    rclpy.spin(node)
    rclpy.shutdown()
