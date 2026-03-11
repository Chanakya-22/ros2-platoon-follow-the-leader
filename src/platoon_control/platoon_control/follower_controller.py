import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FollowerController(Node):

    def __init__(self):
        super().__init__('follower_controller')

        # Declare parameters (so they can be set from launch file)
        self.declare_parameter('lookahead_dist', 1.0)   # metres
        self.declare_parameter('stop_dist', 1.0)       # metres
        self.declare_parameter('k_v', 0.8)              # linear velocity gain
        self.declare_parameter('k_w', 1.2)              # angular velocity gain

        # Get parameter values
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.stop_dist = self.get_parameter('stop_dist').value
        self.k_v = self.get_parameter('k_v').value
        self.k_w = self.get_parameter('k_w').value

        # --- State ---
        self.path = []              # List of (x, y) tuples
        self.leader_pose = None
        self.follower_pose = None

        # --- Subscribers & Publishers ---
        # Note: Topics are remapped in the launch file
        self.create_subscription(Odometry, 'leader_odom', self.leader_callback, 10)
        self.create_subscription(Odometry, 'follower_odom', self.follower_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Follower controller started: lookahead={self.lookahead_dist}, '
            f'k_v={self.k_v}, k_w={self.k_w}'
        )

    def leader_callback(self, msg):
        self.leader_pose = msg.pose.pose
        position = self.leader_pose.position

        # Record path points
        if not self.path:
            self.path.append((position.x, position.y))
        else:
            last_x, last_y = self.path[-1]
            dist = math.sqrt((position.x - last_x)**2 + (position.y - last_y)**2)
            # Only record a new point if the leader has moved 10cm
            if dist > 0.1:
                self.path.append((position.x, position.y))

    def follower_callback(self, msg):
        self.follower_pose = msg.pose.pose

    def get_yaw(self, q):
        # Helper to convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.follower_pose is None or self.leader_pose is None or not self.path:
            return

        fx = self.follower_pose.position.x
        fy = self.follower_pose.position.y
        lx = self.leader_pose.position.x
        ly = self.leader_pose.position.y

        # 1. Prune Path: Remove points the follower has already passed
        # Find the index of the closest point on the path to the follower
        closest_idx = 0
        min_dist = float('inf')

        for i, (px, py) in enumerate(self.path):
            dist = math.sqrt((px - fx)**2 + (py - fy)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Keep points starting from the closest one
        # If the follower is reasonably close to the path or we have a long path, prune it.
        if min_dist < 2.0 or len(self.path) > 10:
            self.path = self.path[closest_idx:]

        # 2. Find Target Point (Pure Pursuit)
        target_x, target_y = self.path[-1]  # Default to the end

        for px, py in self.path:
            dist = math.sqrt((px - fx)**2 + (py - fy)**2)
            if dist > self.lookahead_dist:
                target_x, target_y = px, py
                break

        # 3. Calculate Controls
        dist_to_leader = math.sqrt((lx - fx)**2 + (ly - fy)**2)
        cmd = Twist()

        if dist_to_leader > self.stop_dist:
            # Calculate heading to target
            dx = target_x - fx
            dy = target_y - fy
            angle_to_target = math.atan2(dy, dx)
            yaw = self.get_yaw(self.follower_pose.orientation)

            # Normalize angle error (-pi to pi)
            angle_err = angle_to_target - yaw
            while angle_err > math.pi:
                angle_err -= 2 * math.pi
            while angle_err < -math.pi:
                angle_err += 2 * math.pi

            # Debug the angles (uncomment for troubleshooting)
            # self.get_logger().info(f'Target: ({target_x:.2f}, {target_y:.2f}), Follower: ({fx:.2f}, {fy:.2f})')
            # self.get_logger().info(f'Yaw: {yaw:.2f}, Target Yaw: {angle_to_target:.2f}, Err: {angle_err:.2f}')

            # Set velocities
            # Limit linear speed based on distance (slow down as we get closer)
            lin_x = self.k_v * (dist_to_leader - self.stop_dist)
            cmd.linear.x = max(0.0, min(lin_x, 1.0))

            # Constrain angular velocity to prevent wild spinning
            ang_z = self.k_w * angle_err
            cmd.angular.z = max(-2.0, min(ang_z, 2.0))

            # self.get_logger().info(f'Moving - Dist: {dist_to_leader:.2f}, Target: {cmd.linear.x:.2f}')
        else:
            # Stop if too close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # self.get_logger().info(f'Stopping - Dist: {dist_to_leader:.2f}')

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()