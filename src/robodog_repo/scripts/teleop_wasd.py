#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

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

moveBindings = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    '\x1b[A': (1.0, 0.0),     # Up
    '\x1b[B': (-1.0, 0.0),    # Down
    '\x1b[C': (0.0, -1.0),    # Right
    '\x1b[D': (0.0, 1.0),     # Left
}

stopBindings = [' ', 'x']

class TeleopNode(Node):
    def __init__(self):
        super().__init__('custom_teleop')
        self.publisher_ = self.create_publisher(Twist, '/leader/cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0

    def publish_twist(self, x, th):
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * self.turn
        self.publisher_.publish(twist)

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # Wait for input
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            # read the rest of the arrow key escape sequence
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()

    x = 0.0
    th = 0.0
    print(msg)

    try:
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                print(f"Moving: linear {x}, angular {th}", end='\r')
                node.publish_twist(x, th)
            elif key in stopBindings:
                x = 0.0
                th = 0.0
                print("Stopping...                 ", end='\r')
                node.publish_twist(x, th)
            elif key == '\x03': # CTRL-C
                break
            
            # Continuously publish the last command so the robot keeps moving
            if key == '':
                node.publish_twist(x, th)

    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        node.publisher_.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
