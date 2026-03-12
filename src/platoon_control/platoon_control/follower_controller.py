# ============================================================
#  follower_controller.py
#
#  WHAT DOES THIS FILE DO?
#  Imagine two dogs on a walk.
#  The LEADER dog walks wherever it wants.
#  The FOLLOWER dog watches exactly where the leader has stepped
#  and then walks the same path — like following footprints in the snow.
#
#  This file is the BRAIN of the follower dog.
#  It reads where both dogs are, figures out "which way should I go?",
#  and sends a speed command to make the follower dog move.
# ============================================================


# --- IMPORTS ---
# Think of imports like loading tools from a toolbox before you start work.

import rclpy                           # The main ROS 2 library — lets Python talk to ROS
from rclpy.node import Node            # A "Node" is a program that lives inside the ROS network
from geometry_msgs.msg import Twist    # Twist = a message that holds speed (forward + turn)
from nav_msgs.msg import Odometry      # Odometry = a message that holds the robot's position
import math                            # Python's math library — we need sqrt, atan2, pi, etc.


# --- CLASS DEFINITION ---
# A class is like a blueprint. Here we design our "FollowerController" robot brain.
# It inherits from "Node", meaning it IS a ROS 2 node (program) out of the box.

class FollowerController(Node):

    def __init__(self):
        # __init__ runs once when the node starts up — like the robot turning on.

        # Give this node the name 'follower_controller'.
        # This name shows up in "ros2 node list" so you can see it running.
        super().__init__('follower_controller')


        # ── PARAMETERS ─────────────────────────────────────────────────────
        # Parameters are settings you can change from outside the code (in the launch file).
        # Think of them like dials on a control panel.

        # declare_parameter means: "I want a setting called X, and if nobody gives me
        # a value, use this default."

        self.declare_parameter('lookahead_dist', 1.0)
        # lookahead_dist = How far AHEAD on the path to aim for.
        # If this is 1.0, the follower aims at a point 1 metre ahead on the trail.
        # Bigger = smoother turning. Smaller = tighter following.

        self.declare_parameter('stop_dist', 1.0)
        # stop_dist = How close to the leader before we STOP moving.
        # 1.0 means: stop when the follower is within 1 metre of the leader.
        # This prevents the follower from crashing into the leader.

        self.declare_parameter('k_v', 0.8)
        # k_v = Linear velocity GAIN.
        # A "gain" is a multiplier — it controls how aggressively we respond.
        # Higher k_v = follower speeds up faster when it's far from the leader.

        self.declare_parameter('k_w', 1.2)
        # k_w = Angular velocity GAIN.
        # Controls how sharply the follower turns to correct its heading.
        # Higher k_w = follower turns more aggressively toward the target.


        # Now actually READ those parameter values and store them in the object.
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.stop_dist      = self.get_parameter('stop_dist').value
        self.k_v            = self.get_parameter('k_v').value
        self.k_w            = self.get_parameter('k_w').value


        # ── STATE VARIABLES ─────────────────────────────────────────────────
        # These variables STORE information that the node needs to remember
        # between callbacks (like memory).

        self.path = []
        # A list of (x, y) positions.
        # Every time the leader moves 10 cm, we add a new dot here.
        # Like dropping breadcrumbs on a map — the follower will walk along them.

        self.leader_pose = None
        # Stores the LATEST position+orientation of the leader.
        # Starts as None (empty) — we haven't received any data yet.

        self.follower_pose = None
        # Stores the LATEST position+orientation of the follower.
        # Also starts as None.


        # ── SUBSCRIBERS ─────────────────────────────────────────────────────
        # A subscriber LISTENS to a topic and calls a function when new data arrives.
        # Think of it like a mailbox: whenever a letter arrives, a function is called.

        self.create_subscription(
            Odometry,           # What kind of message to listen for
            'leader_odom',      # The topic name to listen on
            self.leader_callback,  # Call this function when data arrives
            10                  # Buffer up to 10 messages if we're too slow to process
        )
        # This subscribes to the leader's position.
        # In the launch file, 'leader_odom' is remapped to /leader/odom.

        self.create_subscription(
            Odometry,
            'follower_odom',
            self.follower_callback,
            10
        )
        # This subscribes to the follower's position.
        # Remapped to /follower/odom in the launch file.


        # ── PUBLISHER ───────────────────────────────────────────────────────
        # A publisher SENDS messages on a topic.
        # Like a loudspeaker: we shout speed commands into /follower/cmd_vel.

        self.cmd_vel_pub = self.create_publisher(
            Twist,       # We'll be publishing Twist messages (forward speed + turn rate)
            'cmd_vel',   # Topic name (remapped to /follower/cmd_vel in launch file)
            10           # Buffer up to 10 messages
        )


        # ── TIMER ───────────────────────────────────────────────────────────
        # A timer calls a function automatically at a fixed rate.
        # 0.05 seconds = 20 times per second (20 Hz).
        # Every 0.05 seconds, the control_loop function runs and updates the speed command.

        self.timer = self.create_timer(0.05, self.control_loop)


        # Print a message to the terminal so we know the node started successfully.
        self.get_logger().info(
            f'Follower controller started: lookahead={self.lookahead_dist}, '
            f'k_v={self.k_v}, k_w={self.k_w}'
        )


    # ===========================================================
    # CALLBACK: leader_callback
    # Called automatically every time the leader publishes its position.
    # ===========================================================
    def leader_callback(self, msg):
        # 'msg' is the Odometry message that just arrived.

        self.leader_pose = msg.pose.pose
        # Save the leader's full pose (position + orientation) for later use.

        position = self.leader_pose.position
        # Extract just the position part (x, y, z coordinates).

        # Now decide whether to add this position to our breadcrumb trail.
        if not self.path:
            # The path list is EMPTY — this is the very first point we've received.
            # Add it unconditionally.
            self.path.append((position.x, position.y))
        else:
            # The path already has some points. Check how far the leader has moved
            # since the LAST recorded point.

            last_x, last_y = self.path[-1]
            # self.path[-1] = the most recent breadcrumb. Unpack its x and y.

            dist = math.sqrt((position.x - last_x)**2 + (position.y - last_y)**2)
            # Euclidean distance formula: how far apart are the two points?
            # sqrt( (x2-x1)² + (y2-y1)² )

            if dist > 0.1:
                # Only add a new breadcrumb if the leader moved MORE than 10 cm.
                # This prevents the list from growing huge when the leader is standing still.
                self.path.append((position.x, position.y))


    # ===========================================================
    # CALLBACK: follower_callback
    # Called automatically every time the follower publishes its position.
    # ===========================================================
    def follower_callback(self, msg):
        # 'msg' is the Odometry message from the follower robot.

        self.follower_pose = msg.pose.pose
        # Just save the follower's latest pose. Nothing else is done here —
        # the control_loop uses this data when it runs.


    # ===========================================================
    # HELPER: get_yaw
    # Converts a quaternion orientation into a simple yaw (heading) angle.
    # ===========================================================
    def get_yaw(self, q):
        # WHY DO WE NEED THIS?
        # ROS stores robot orientation as a "quaternion" (4 numbers: x, y, z, w).
        # For a flat ground robot, we only care about the HEADING angle (yaw) —
        # which direction it's facing in the horizontal plane.
        # This function does the math to extract just that angle.

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        # This is one part of the formula. (An intermediate calculation.)

        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # This is the other part of the formula.

        return math.atan2(siny_cosp, cosy_cosp)
        # atan2 gives us the angle in radians from -π to +π.
        # Positive = facing left/CCW, Negative = facing right/CW.


    # ===========================================================
    # MAIN CONTROL LOOP — runs 20 times per second
    # This is where the follower decides how fast and which way to move.
    # ===========================================================
    def control_loop(self):

        # STEP 0: SAFETY CHECK
        # If we don't have all the data we need yet, do nothing and wait.
        if self.follower_pose is None or self.leader_pose is None or not self.path:
            return
            # 'return' exits the function early — like saying "not ready yet, skip".


        # --- Extract current positions ---

        fx = self.follower_pose.position.x   # Follower's X coordinate (metres)
        fy = self.follower_pose.position.y   # Follower's Y coordinate (metres)
        lx = self.leader_pose.position.x     # Leader's X coordinate
        ly = self.leader_pose.position.y     # Leader's Y coordinate


        # ── STEP 1: PRUNE THE PATH (remove old breadcrumbs) ────────────────
        #
        # The breadcrumb trail grows as the leader walks.
        # We need to remove points that the follower has ALREADY PASSED
        # so we're not chasing old waypoints behind us.
        #
        # Think of it like: you dropped breadcrumbs, you've walked past some,
        # so you clear the ones behind you and only keep the ones ahead.

        closest_idx = 0      # Will hold the index of the closest breadcrumb
        min_dist    = float('inf')   # Start with "infinity" — any real distance will be smaller

        for i, (px, py) in enumerate(self.path):
            # Loop through EVERY breadcrumb point in the path.
            # i = the index (position in the list), px/py = the coordinates.

            dist = math.sqrt((px - fx)**2 + (py - fy)**2)
            # Calculate how far this breadcrumb is from the follower.

            if dist < min_dist:
                # If this breadcrumb is closer than the closest found so far...
                min_dist    = dist        # Update the closest distance.
                closest_idx = i           # Remember which breadcrumb is closest.

        # After the loop, closest_idx tells us which point in the path is nearest
        # to the follower's current position.

        if min_dist < 2.0 or len(self.path) > 10:
            # Prune the path IF:
            # - the follower is within 2m of the path (it's on track), OR
            # - the path has more than 10 points (getting too long to keep).
            self.path = self.path[closest_idx:]
            # Slice the list: throw away everything BEFORE closest_idx.
            # Now self.path only contains the unpassed breadcrumbs (current and ahead).


        # ── STEP 2: FIND THE TARGET POINT (Pure Pursuit Lookahead) ─────────
        #
        # We don't drive toward the NEAREST breadcrumb — that causes wobbly driving.
        # Instead, we pick a point that is a fixed distance AHEAD on the path:
        # the "lookahead point". This is the Pure Pursuit algorithm.
        #
        # Analogy: When driving a car, you look ahead at a point down the road,
        # not at the road directly under your front wheels.

        target_x, target_y = self.path[-1]
        # Default target = the LAST breadcrumb in the list (the furthest ahead).
        # We'll try to find a better (closer lookahead) point in the loop below.

        for px, py in self.path:
            # Walk through remaining breadcrumbs from closest to farthest.

            dist = math.sqrt((px - fx)**2 + (py - fy)**2)
            # How far is this breadcrumb from the follower?

            if dist > self.lookahead_dist:
                # Found a breadcrumb that is FARTHER than our lookahead distance!
                # This is our steering target.
                target_x, target_y = px, py
                break   # Stop searching — we found the first point that qualifies.


        # ── STEP 3: CALCULATE CONTROL COMMANDS ──────────────────────────────
        #
        # Now compute two things:
        #   linear.x  = how fast to go forward
        #   angular.z = how sharply to turn
        #
        # We'll put them inside a Twist message and publish it.

        dist_to_leader = math.sqrt((lx - fx)**2 + (ly - fy)**2)
        # How far is the follower from the LEADER right now? (straight-line distance)

        cmd = Twist()
        # Create a blank Twist message. All values start at 0 (zero speed).


        if dist_to_leader > self.stop_dist:
            # --- The follower is FAR ENOUGH from the leader — it should MOVE. ---

            # Find the direction from the follower to the target point.
            dx = target_x - fx   # Horizontal distance to target
            dy = target_y - fy   # Vertical distance to target

            angle_to_target = math.atan2(dy, dx)
            # atan2 gives the angle (in radians) of the line from follower to target.
            # 0 = pointing right, π/2 = pointing up, -π/2 = pointing down, etc.

            yaw = self.get_yaw(self.follower_pose.orientation)
            # What angle is the follower CURRENTLY FACING? (extracted from its quaternion)

            angle_err = angle_to_target - yaw
            # angle_err = "how many radians does the follower need to rotate?"
            # Positive = it needs to turn LEFT.
            # Negative = it needs to turn RIGHT.

            # ── Angle wrapping ───────────────────────────────────────────
            # Angles should stay between -π and +π (about -3.14 to +3.14 radians).
            # Without this correction, the robot might try to turn 350° right
            # instead of the equivalent 10° left.

            while angle_err > math.pi:
                angle_err -= 2 * math.pi   # If too big, subtract a full circle
            while angle_err < -math.pi:
                angle_err += 2 * math.pi   # If too small, add a full circle


            # ── Debug logging (uncomment these lines to see values in the terminal) ──
            # self.get_logger().info(f'Target: ({target_x:.2f}, {target_y:.2f}), Follower: ({fx:.2f}, {fy:.2f})')
            # self.get_logger().info(f'Yaw: {yaw:.2f}, Target Yaw: {angle_to_target:.2f}, Err: {angle_err:.2f}')


            # ── LINEAR (forward) SPEED ────────────────────────────────────
            lin_x = self.k_v * (dist_to_leader - self.stop_dist)
            # Formula: speed = gain × (distance - safe_gap)
            # The further away the leader is, the faster the follower goes.
            # When barely outside stop_dist, the follower barely moves.
            # When very far, the follower goes fast.

            cmd.linear.x = max(0.0, min(lin_x, 1.0))
            # Clamp the speed:
            # max(0.0, ...) = never go BACKWARD (minimum is 0)
            # min(..., 1.0) = never exceed 1.0 m/s (maximum cap)


            # ── ANGULAR (turning) SPEED ───────────────────────────────────
            ang_z = self.k_w * angle_err
            # Formula: turn rate = gain × heading_error
            # Bigger heading error = turn faster to correct it.
            # Positive angle_err → positive ang_z → turns left (correct!)
            # Negative angle_err → negative ang_z → turns right (correct!)

            cmd.angular.z = max(-0.5, min(ang_z, 0.5))
            # !! CRITICAL CLAMP — must be small or the follower falls !!
            # walk_cmd_vel computes:  turn_factor = angular_z × 0.8
            # Then:  hip_amplitude = fwd_speed ± turn_factor
            # At angular_z=2.0: turn_factor=1.6, hip reaches 1.85 rad → beyond ±0.8 limit
            # → joint slams hard stop every sharp turn → robot falls.
            # At angular_z=0.5: turn_factor=0.4, hip reaches 0.65 rad → safely within limit.

            # self.get_logger().info(f'Moving - Dist: {dist_to_leader:.2f}, vel: {cmd.linear.x:.2f}')

        else:
            # --- The follower is CLOSE ENOUGH to the leader — it should STOP. ---
            cmd.linear.x  = 0.0   # No forward motion
            cmd.angular.z = 0.0   # No turning either
            # Sending zeros tells the walk_cmd_vel node to stand still.
            # self.get_logger().info(f'Stopping - Dist: {dist_to_leader:.2f}')


        self.cmd_vel_pub.publish(cmd)
        # Send the final speed command to the /follower/cmd_vel topic.
        # The walk_cmd_vel node picks this up and converts it into walking motion.


# ============================================================
# ENTRY POINT — this runs when you start the node
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    # rclpy.init() = "turn on" the ROS 2 system inside this Python script.
    # Must be called before creating any nodes.

    node = FollowerController()
    # Create our FollowerController node. This automatically calls __init__().

    rclpy.spin(node)
    # spin() keeps the program ALIVE and LISTENING.
    # It processes incoming messages and fires the timer callback.
    # Without spin(), the node would start and immediately exit.
    # Press CTRL+C to stop it.

    rclpy.shutdown()
    # rclpy.shutdown() = "turn off" the ROS 2 system cleanly before exiting.


if __name__ == '__main__':
    # This line means: only run main() if this file is executed directly
    # (not when it's imported as a module by another file).
    main()