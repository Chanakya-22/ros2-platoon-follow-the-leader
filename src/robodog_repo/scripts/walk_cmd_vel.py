#!/usr/bin/env python3
# ============================================================
#  walk_cmd_vel.py
#
#  WHAT DOES THIS FILE DO?
#  Think of this file as a TRANSLATOR.
#
#  The follower_controller.py says things like:
#    "Move forward at 0.5 m/s and turn slightly left"
#  But Gazebo doesn't understand "move forward" — it only understands:
#    "Set the front-left hip joint to 0.3 radians"
#    "Set the front-left knee joint to -0.6 radians" ... etc.
#
#  This file translates the simple "go forward + turn" command
#  into sinusoidal (wave-shaped) joint angles that produce a
#  TROT GAIT — a natural 4-legged walking pattern.
#
#  TROT GAIT EXPLAINED:
#  In a trot, two DIAGONAL legs move together at the same time:
#    Pair A (move together): Front-Left  + Rear-Right
#    Pair B (move together): Front-Right + Rear-Left
#  While Pair A swings forward, Pair B pushes backward — then they swap.
#  This zig-zag pattern propels the robot forward stably.
#
#  HOW THE MATH WORKS:
#  We use sin() — a wave that smoothly goes from 0 → 1 → 0 → -1 → 0...
#  Hip joints:  angle = amplitude × sin(phase)   → swings leg forward/backward
#  Knee joints: angle = offset − amplitude × max(0, sin(phase))
#               → only bends when the leg is swinging FORWARD (to lift the foot off ground)
#
#  This node runs for BOTH the leader and the follower robot.
#  The launch file runs two copies — one in the 'leader' namespace,
#  one in the 'follower' namespace.
# ============================================================


# --- IMPORTS ---

import math
# Python's math library — we need sin() and pi for the gait wave calculations.

import rclpy
# The main ROS 2 library — required by every ROS 2 Python program.

from rclpy.node import Node
# Node is the base class for any ROS 2 program.
# We inherit from it to get all the subscriber/publisher/timer functionality.

from std_msgs.msg import Float64
# Float64 = a ROS message that holds a single decimal number.
# Gazebo's joint position controllers expect Float64 messages (the angle in radians).

from geometry_msgs.msg import Twist
# Twist = a ROS message for velocity commands.
# It holds: linear.x (forward speed, m/s) and angular.z (turn rate, rad/s).
# walk_cmd_vel subscribes to these and converts them into joint angles.


# ── CLASS DEFINITION ─────────────────────────────────────────────────────────

class WalkCmdVelNode(Node):
    # Our walking controller node — inherits all ROS 2 Node capabilities.

    def __init__(self):
        # __init__ runs ONCE when the node first starts up (like a constructor).

        super().__init__('walk_cmd_vel_node')
        # Register this node with ROS 2 under the name 'walk_cmd_vel_node'.
        # This name appears in "ros2 node list".


        # ── SUBSCRIBE TO cmd_vel ──────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # Listen on the 'cmd_vel' topic for velocity commands.
        # 'cmd_vel' is a RELATIVE topic name — ROS 2 prepends the namespace.
        # If this node runs under the 'leader' namespace → listens on /leader/cmd_vel
        # If this node runs under the 'follower' namespace → listens on /follower/cmd_vel
        # Queue depth 10 = hold up to 10 recent messages if we're temporarily busy.


        # ── GAIT PARAMETERS ───────────────────────────────────────────────
        # These are tunable settings that can be changed from the launch file.
        # declare_parameter = "I have this setting; use this default if no value given."

        self.declare_parameter('base_gait_frequency', 1.5)
        # base_gait_frequency = how many complete steps per second.
        # 1.5 Hz = 1.5 full walk cycles per second.
        # Higher = faster stepping cadence (but may look unnatural if too high).

        self.declare_parameter('base_hip_amplitude', 0.5)
        # base_hip_amplitude = how far the hips swing in each direction (radians).
        # 0.5 rad ≈ 28 degrees of swing.
        # Bigger = longer strides. Smaller = tiny shuffling steps.

        self.declare_parameter('knee_amplitude', 0.8)
        # knee_amplitude = how much the knee bends to LIFT the foot during each step.
        # 0.8 rad ≈ 46 degrees of bend.
        # More bend = foot lifts higher (good for uneven terrain but uses more energy).

        self.declare_parameter('knee_offset', -0.3)
        # knee_offset = the resting angle of the knee when standing still.
        # -0.3 rad = slightly bent (like a person standing with slightly bent knees).
        # This is the "neutral" pose the robot returns to when stopped.

        self.declare_parameter('update_rate', 50.0)
        # update_rate = how many times per second the gait loop runs (Hz).
        # 50 Hz = gait joint commands are sent 50 times every second.
        # Higher = smoother motion but more CPU usage. 50 Hz is a good balance.


        # Now actually READ the parameter values into variables for easy access.
        self.base_freq    = self.get_parameter('base_gait_frequency').value
        self.base_hip_amp = self.get_parameter('base_hip_amplitude').value
        self.knee_amp     = self.get_parameter('knee_amplitude').value
        self.knee_off     = self.get_parameter('knee_offset').value
        rate              = self.get_parameter('update_rate').value


        # ── CURRENT VELOCITY (updated by subscriber callback) ─────────────
        self.linear_x  = 0.0
        # Current forward speed command. 0.0 = stopped. Positive = forward. Negative = backward.

        self.angular_z = 0.0
        # Current turn rate command. 0.0 = straight. Positive = turn left. Negative = turn right.


        # ── FIGURE OUT WHICH GAZEBO MODEL TO CONTROL ──────────────────────
        ns = self.get_namespace()
        # get_namespace() returns this node's ROS namespace.
        # Example: if launched under 'leader' namespace, this returns '/leader'.

        if ns == '/':
            ns = ''
        # If there's no namespace (just the root '/'), treat it as empty string.
        # This handles the case where the node is run standalone for testing.

        self.declare_parameter('gz_model_name', 'robodog')
        # gz_model_name = the name of the robot model INSIDE Gazebo.
        # Gazebo uses this name in all joint topic paths:
        #   /model/<gz_model_name>/joint/<joint_name>/cmd_pos
        # Default 'robodog' is for single-robot mode (gazebo.launch.py).
        # Overridden to 'leader' or 'follower' by platoon_robodogs.launch.py.

        self.gz_model_name = self.get_parameter('gz_model_name').value
        # Read and store the Gazebo model name.


        # ── CREATE JOINT PUBLISHERS ───────────────────────────────────────
        # For each joint motor, we need a publisher to send position commands.
        # The topic name format must EXACTLY match what the Gazebo
        # JointPositionController plugin is listening on (defined in the URDF).

        self.joint_pubs = {}
        # Empty dictionary — we'll add one publisher per joint.
        # Key = joint name (string), Value = the publisher object.

        joint_names = [
            'front_left_hip_joint',    # Swings front-left leg forward/backward
            'front_left_knee_joint',   # Bends front-left knee (lifts foot)
            'front_right_hip_joint',   # Swings front-right leg
            'front_right_knee_joint',  # Bends front-right knee
            'rear_left_hip_joint',     # Swings rear-left leg
            'rear_left_knee_joint',    # Bends rear-left knee
            'rear_right_hip_joint',    # Swings rear-right leg
            'rear_right_knee_joint',   # Bends rear-right knee
            'head_joint',              # Pans the head left/right
        ]

        for name in joint_names:
            topic = f'/model/{self.gz_model_name}/joint/{name}/cmd_pos'
            # Build the full topic string.
            # Example: /model/leader/joint/front_left_hip_joint/cmd_pos
            # The Gazebo JointPositionController plugin is listening exactly here.

            self.joint_pubs[name] = self.create_publisher(Float64, topic, 10)
            # Create a publisher for this joint and store it in the dictionary.
            # Float64 = the message type (just a single number — the target angle).


        # ── TIMER SETUP ───────────────────────────────────────────────────
        self.t  = 0.0
        # 't' is our internal time counter, starting at 0.
        # It increases by dt every tick and is used in sin(omega * t) to create the wave.
        # Think of it as the "clock" that drives the walking rhythm.

        self.dt = 1.0 / rate
        # dt = time between each timer tick (in seconds).
        # Example: rate=50 Hz → dt = 1/50 = 0.02 seconds per tick.

        self.timer = self.create_timer(self.dt, self.timer_callback)
        # Set up a repeating timer that calls timer_callback() every dt seconds.
        # This is the heartbeat of the walking controller.

        self.get_logger().info(f'[{self.gz_model_name}] Walk cmd_vel controller started')
        # Print a confirmation message to the terminal when the node starts.


    # =========================================================================
    # CALLBACK: cmd_vel_callback
    # Called every time a new Twist (speed command) arrives on cmd_vel.
    # =========================================================================
    def cmd_vel_callback(self, msg: Twist):
        # 'msg' is the incoming Twist message.

        self.linear_x  = msg.linear.x
        # Save the forward speed command.
        # > 0 = move forward,  < 0 = move backward,  = 0 = stop.

        self.angular_z = msg.angular.z
        # Save the turn rate command.
        # > 0 = turn left (counter-clockwise),  < 0 = turn right (clockwise).

        # That's it — we just cache the values.
        # The actual gait computation happens in timer_callback() below,
        # which reads these values 50 times per second.


    # =========================================================================
    # HELPER: publish_cmd
    # Sends a position command to ONE joint.
    # =========================================================================
    def publish_cmd(self, joint_name: str, value: float):
        # joint_name = which joint to command (e.g. 'front_left_hip_joint')
        # value      = the target angle for that joint, in radians

        msg = Float64()
        # Create a blank Float64 message.

        msg.data = value
        # Set the data field to the angle we want.

        self.joint_pubs[joint_name].publish(msg)
        # Look up the publisher for this joint and send the message.
        # Gazebo's PID controller receives this and drives the joint motor to that angle.


    # =========================================================================
    # MAIN GAIT LOOP: timer_callback
    # Runs 50 times per second — this is where the walking magic happens.
    # =========================================================================
    def timer_callback(self):

        # ── CHECK IF THE ROBOT SHOULD BE MOVING ───────────────────────────
        is_moving = abs(self.linear_x) > 0.001 or abs(self.angular_z) > 0.001
        # abs() = absolute value (ignores sign, so -0.5 and 0.5 both give 0.5)
        # > 0.001 = ignore tiny floating point noise near zero (a "dead zone")
        # is_moving = True if either linear OR angular speed is non-negligible.


        # ── CASE A: ROBOT IS STATIONARY ───────────────────────────────────
        if not is_moving:
            # The robot received a stop command (or no command yet).
            # Put all joints back to their neutral standing pose.

            self.t = 0.0
            # Reset the time counter back to zero.
            # This ensures the next walking motion always starts cleanly from phase 0
            # instead of mid-stride, which could cause a jerky startup.

            for name in self.joint_pubs.keys():
                # Loop through every joint.

                if 'knee' in name:
                    self.publish_cmd(name, self.knee_off)
                    # Knee joints → set to the resting crouch angle (e.g. -0.3 rad).
                    # This keeps the robot crouched slightly (like a dog standing at rest).
                else:
                    self.publish_cmd(name, 0.0)
                    # All hip joints AND head → set to 0 (perfectly centred / upright).

            return
            # Exit the function early — no need to compute gait while standing still.


        # ── CASE B: ROBOT IS MOVING — COMPUTE THE TROT GAIT ──────────────


        # STEP 1: Calculate the current stepping frequency
        current_freq = self.base_freq * min(abs(self.linear_x) * 2.5, 2.0)
        # We make the step FASTER when the robot is moving faster — just like
        # a real dog takes faster steps when running vs walking.
        #
        # abs(self.linear_x) * 2.5 = a speed-to-frequency scale factor.
        # min(..., 2.0) = cap it at 2× the base frequency (don't step too fast).
        # base_freq=1.5 Hz × up to 2.0 = max 3.0 Hz stepping speed.

        if current_freq < 0.5:
            current_freq = 0.5
        # Minimum stepping frequency: at least 0.5 Hz.
        # Prevents ultra-slow shuffling when barely any speed is commanded.


        # STEP 2: Determine walking direction
        direction = 1.0 if self.linear_x >= 0 else -1.0
        # 1.0 = forward walk.   -1.0 = backward walk.
        # The sin() wave is symmetric — flipping direction reverses the gait.


        # STEP 3: Calculate the current phase of the gait wave
        omega = 2.0 * math.pi * current_freq
        # omega = angular frequency in radians per second.
        # A full sine wave is 2π radians of phase.
        # At 1.5 Hz: omega = 2π × 1.5 ≈ 9.42 rad/s

        phase = omega * self.t
        # phase = how far along the sine wave we currently are (in radians).
        # self.t keeps ticking upward → phase keeps growing → sin(phase) keeps cycling.
        # This is what produces the smooth, repeating wave motion of the legs.


        # STEP 4: Compute turning adjustments
        turn_factor = self.angular_z * 0.8
        # turn_factor is how much to ADD to one side and SUBTRACT from the other.
        # This is like skid-steering (tank turning):
        #   Left side gets MORE swing → moves farther per step → robot curves right.
        #   Right side gets LESS swing → robot curves right.
        # Multiplying by 0.8 prevents over-correction (trial-tuned value).

        fwd_speed = -self.base_hip_amp * direction if abs(self.linear_x) > 0.05 else 0.0
        # fwd_speed = the baseline hip swing amplitude for forward/backward motion.
        # Negative sign because of how the joint coordinate frame is oriented in the URDF
        # (positive hip angle = leg swings one way; we flip sign to go forward correctly).
        # If linear_x is tiny (< 0.05), treat it as zero — pure rotation in place.

        hip_amp_left  = fwd_speed + turn_factor
        # Left-side hips get MORE swing when turning left (angular_z > 0 → turn_factor > 0).

        hip_amp_right = fwd_speed - turn_factor
        # Right-side hips get LESS swing when turning left.
        # This difference between left and right is what makes the robot curve.


        # STEP 5: Compute hip and knee angles for BOTH gait phases
        #
        # PHASE A: front-left + rear-right legs (they move in sync)
        # PHASE B: front-right + rear-left legs (180° out of phase with A)
        # When PHASE A legs swing forward, PHASE B legs swing backward — they alternate.

        hip_a_left  = hip_amp_left  * math.sin(phase)
        # Front-left and rear-right HIP angle for Phase A.
        # sin(phase) oscillates between -1 and +1 → the hip swings back and forth.

        hip_b_left  = hip_amp_left  * math.sin(phase + math.pi)
        # For Phase B legs on the LEFT side.
        # Adding π (180°) flips the wave: when A is at +peak, B is at -peak.
        # This is what makes the two pairs alternate.

        hip_a_right = hip_amp_right * math.sin(phase)
        # Rear-right HIP angle (same phase as front-left, but with RIGHT amplitude).

        hip_b_right = hip_amp_right * math.sin(phase + math.pi)
        # Front-right HIP angle (Phase B, RIGHT side amplitude).

        knee_a = self.knee_off - self.knee_amp * max(0.0, math.sin(phase))
        # Knee angle for Phase A legs.
        # max(0.0, sin(phase)) = only take the POSITIVE half of the wave.
        #   → When sin > 0 (leg swinging forward), knee bends MORE → foot lifts UP.
        #   → When sin < 0 (leg pushing backward), max gives 0 → knee stays at offset.
        # This mimics how real animals only lift their foot during the forward swing,
        # not during the backward push-off.

        knee_b = self.knee_off - self.knee_amp * max(0.0, math.sin(phase + math.pi))
        # Same logic for Phase B knees (180° offset from Phase A).


        # STEP 6: Clamp knee angles to safe mechanical limits
        knee_a = max(-1.2, min(0.0, knee_a))
        # Keep knee_a between -1.2 rad (fully bent, max) and 0.0 rad (fully straight).
        # max(-1.2, ...) prevents over-bending. min(..., 0.0) prevents hyperextension.

        knee_b = max(-1.2, min(0.0, knee_b))
        # Same safety clamp for Phase B knees.


        # STEP 7: Send the positions to each joint
        #
        # ── Phase A leg pair (front-left  +  rear-right) ──
        self.publish_cmd('front_left_hip_joint',  hip_a_left)
        # Set front-left hip to its current wave position.

        self.publish_cmd('front_left_knee_joint', knee_a)
        # Set front-left knee — bends during the forward swing to lift the foot.

        self.publish_cmd('rear_right_hip_joint',  hip_a_right)
        # Rear-right swings with front-left (diagonal pair, same phase, right amplitude).

        self.publish_cmd('rear_right_knee_joint', knee_a)
        # Rear-right knee lifts at the same moment as front-left.

        # ── Phase B leg pair (front-right  +  rear-left) ──
        self.publish_cmd('front_right_hip_joint', hip_b_right)
        # Front-right swings at the OPPOSITE phase (180° offset from front-left).

        self.publish_cmd('front_right_knee_joint', knee_b)
        # Front-right knee bends opposite in cycle to front-left.

        self.publish_cmd('rear_left_hip_joint',   hip_b_left)
        # Rear-left is the pair of front-right (diagonal pair).

        self.publish_cmd('rear_left_knee_joint',  knee_b)
        # Rear-left knee matches front-right in the Phase B cycle.

        # ── Head stays centred ──
        self.publish_cmd('head_joint', 0.0)
        # The head is always commanded to 0 (facing straight forward).
        # In future you could add head tracking code here.


        # STEP 8: Advance time
        self.t += self.dt
        # Move the time clock forward by one timestep (e.g. 0.02 seconds).
        # This advances the phase of the sin() wave on the next tick.
        # Over time, phase keeps growing → sin keeps cycling → robot keeps walking.


# ── ENTRY POINT ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    # Turn on the ROS 2 system inside this Python process.

    node = WalkCmdVelNode()
    # Create our walk controller node — this calls __init__() and sets up everything.

    try:
        rclpy.spin(node)
        # spin() keeps the node alive and continuously processes:
        #   - incoming cmd_vel messages (calls cmd_vel_callback)
        #   - timer ticks (calls timer_callback 50x per second)
        # The program stays here until the user presses CTRL+C.

    except KeyboardInterrupt:
        pass
        # CTRL+C was pressed — this exception is caught and we exit cleanly.
        # 'pass' means "do nothing special, just continue to the finally block".

    finally:
        node.destroy_node()
        # Properly destroy the node and release its resources.

        rclpy.shutdown()
        # Turn off the ROS 2 system cleanly.


if __name__ == '__main__':
    # Only run main() if this script is run directly (not imported as a module).
    main()
