#!/usr/bin/env python3
"""
=============================================================================
teleop_wasd.py  —  ROS 2 Keyboard Teleoperation for the Leader Robodog
=============================================================================
PURPOSE:
    Lets a human operator drive the LEADER robodog using the keyboard
    (WASD + arrow keys) in a terminal.

    It publishes geometry_msgs/Twist messages to /leader/cmd_vel.
    The walk_cmd_vel node consumes those Twist messages and converts them
    into sinusoidal joint commands that make the robodog physically walk.

HOW IT WORKS:
    - Uses raw terminal I/O (termios + tty) to capture key presses without
      needing the user to press Enter.
    - select.select() polls stdin with a 100 ms timeout, so the loop keeps
      publishing the last command even when no key is pressed — this prevents
      the robot from stopping the moment you lift a key.
    - Arrow-key escape sequences (e.g. \x1b[A for Up) are also handled.

KEY BINDINGS:
    W / Up    → forward      (linear.x = +speed)
    S / Down  → backward     (linear.x = -speed)
    A / Left  → turn left    (angular.z = +turn)
    D / Right → turn right   (angular.z = -turn)
    Space / X → stop         (all zero)
    CTRL-C    → quit

CONSTANTS:
    speed = 0.5 m/s   (linear velocity magnitude)
    turn  = 1.0 rad/s (angular velocity magnitude)
=============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Standard ROS 2 velocity command message
import sys
import select   # For non-blocking stdin polling with timeout
import termios  # For saving/restoring terminal settings
import tty      # For enabling raw (character-by-character) terminal mode

# ── Help text printed on startup ──────────────────────────────────────────────
msg = """
Control Your Robodog Leader!
---------------------------
Moving around:
   W or Up Arrow    : Forward
   S or Down Arrow  : Backward
   A or Left Arrow  : Turn Left
   D or Right Arrow : Turn Right
   Space or X       : Stop

CTRL-C to quit
"""

# ── Key-binding tables ────────────────────────────────────────────────────────
# Each entry maps a key (string) → (linear_x_factor, angular_z_factor).
# The actual velocity is factor × speed  or  factor × turn  (see publish_twist).
moveBindings = {
    'w': ( 1.0,  0.0),   # W key       → forward
    's': (-1.0,  0.0),   # S key       → backward
    'a': ( 0.0,  1.0),   # A key       → turn left  (positive yaw = CCW)
    'd': ( 0.0, -1.0),   # D key       → turn right (negative yaw = CW)
    '\x1b[A': ( 1.0,  0.0),   # Up arrow    → forward
    '\x1b[B': (-1.0,  0.0),   # Down arrow  → backward
    '\x1b[C': ( 0.0, -1.0),   # Right arrow → turn right
    '\x1b[D': ( 0.0,  1.0),   # Left arrow  → turn left
}

# Keys that immediately stop the robot (publish all-zero Twist).
stopBindings = [' ', 'x']


class TeleopNode(Node):
    """
    Minimal ROS 2 node that publishes Twist velocity commands.

    Only a single publisher is needed here — all the reading and looping
    logic lives in the standalone main() function rather than in callbacks,
    because we are doing raw terminal I/O which works best in a normal loop.
    """

    def __init__(self):
        # Node name visible in `ros2 node list`
        super().__init__('custom_teleop')

        # Publish directly to /leader/cmd_vel (absolute topic, not remapped).
        # walk_cmd_vel running in the 'leader' namespace listens on this topic.
        self.publisher_ = self.create_publisher(Twist, '/leader/cmd_vel', 10)

        # Velocity magnitudes — tune these to change how fast the leader moves.
        self.speed = 0.5   # linear speed   in m/s
        self.turn  = 1.0   # angular speed  in rad/s

    def publish_twist(self, x: float, th: float):
        """
        Build and publish a Twist message.

        Parameters
        ----------
        x  : float — linear factor  (-1.0 = backward, 0.0 = stop, +1.0 = forward)
        th : float — angular factor (-1.0 = right,    0.0 = stop, +1.0 = left)
        """
        twist = Twist()
        twist.linear.x  = x  * self.speed   # scale factor → actual m/s
        twist.linear.y  = 0.0               # unused (robot can't strafe)
        twist.linear.z  = 0.0               # unused (ground robot)
        twist.angular.x = 0.0               # unused (roll)
        twist.angular.y = 0.0               # unused (pitch)
        twist.angular.z = th * self.turn    # scale factor → actual rad/s
        self.publisher_.publish(twist)


# ── Terminal I/O helpers ──────────────────────────────────────────────────────

def getKey(settings) -> str:
    """
    Read a single keypress (or arrow-key escape sequence) from stdin.

    Uses raw terminal mode so keypresses are available immediately without
    the user pressing Enter.  select.select() provides a 0.1 s timeout:
    if no key arrives within 100 ms, an empty string is returned — this lets
    the main loop re-publish the last command to keep the robot moving.

    Parameters
    ----------
    settings : list — saved terminal attributes (from termios.tcgetattr)

    Returns
    -------
    str — the key pressed, e.g. 'w', '\x1b[A' (Up arrow), or '' (timeout)
    """
    tty.setraw(sys.stdin.fileno())   # Switch terminal to char-by-char mode

    # Wait up to 100 ms for a keypress.
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)   # Read one character
        if key == '\x1b':
            # Arrow keys are 3-byte escape sequences: ESC [ A/B/C/D
            # Read the remaining two bytes to identify which arrow key.
            key += sys.stdin.read(2)
    else:
        key = ''   # Timeout — no key was pressed

    # Restore the terminal to its original (cooked) mode so future I/O works.
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    """
    Entry point:
      1. Save terminal settings so we can restore them on exit.
      2. Create the ROS 2 node.
      3. Loop:  read key → determine velocity factors → publish Twist.
      4. On CTRL-C: publish a zero Twist (safety stop) and cleanly shut down.
    """
    # Save original terminal attributes so we can restore them before exit.
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()         # Initialise the ROS 2 Python client library
    node = TeleopNode()

    # Current velocity factors (updated per keypress, held between presses).
    x  = 0.0   # linear factor
    th = 0.0   # angular factor

    print(msg)  # Show the key-binding help text in the terminal

    try:
        while True:
            key = getKey(settings)   # Non-blocking read (100 ms timeout)

            if key in moveBindings:
                # A movement key was pressed — update the velocity factors.
                x  = moveBindings[key][0]
                th = moveBindings[key][1]
                print(f"Moving: linear {x}, angular {th}", end='\r')
                node.publish_twist(x, th)

            elif key in stopBindings:
                # Stop key (Space or X) — zero everything immediately.
                x  = 0.0
                th = 0.0
                print("Stopping...                 ", end='\r')
                node.publish_twist(x, th)

            elif key == '\x03':
                # CTRL-C (ASCII 0x03 = ETX) — break out of the loop.
                break

            # If no key was pressed (key == ''), re-publish the last command.
            # This keeps the robot moving smoothly without stutter when a key
            # is held down or briefly released.
            if key == '':
                node.publish_twist(x, th)

    except Exception as e:
        print(e)

    finally:
        # ── Safety stop on exit ───────────────────────────────────────────
        # Always send a zero Twist before quitting so the robot doesn't keep
        # moving after the teleop node dies.
        twist = Twist()   # All fields default to 0.0
        node.publisher_.publish(twist)

        # Restore the terminal to its normal (cooked) mode.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
