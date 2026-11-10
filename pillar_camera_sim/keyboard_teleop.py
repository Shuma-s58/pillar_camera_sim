"""Terminal keyboard input; works without pynput, tkinter, or an RViz keyboard plugin."""
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('pillar_keyboard_teleop')
        self.pub = self.create_publisher(String, '/camera/key_command', 10)

    def send(self, key):
        self.pub.publish(String(data=key))


def run_keyboard(node):
    if not sys.stdin.isatty():
        raise RuntimeError('Run keyboard_teleop from an interactive terminal (TTY).')
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    bindings = {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}
    print('Camera only: W/S forward/back, A/D strafe, Q/E yaw, R/F up/down, Space reset, X exit')
    print('Arrow keys are also supported. Focus this terminal and press keys. Ctrl-C to exit.')
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                rclpy.spin_once(node, timeout_sec=0)
                continue
            char = os.read(fd, 1).decode('utf-8', 'ignore')
            if char == '' or char.lower() == 'x':
                break
            if char == '':
                # ANSI escape sequence for cursor keys. Never block waiting for it.
                if select.select([sys.stdin], [], [], 0.03)[0]:
                    second = os.read(fd, 1).decode('utf-8', 'ignore')
                    if second == '[' and select.select([sys.stdin], [], [], 0.03)[0]:
                        third = os.read(fd, 1).decode('utf-8', 'ignore')
                        char = bindings.get(third, '')
                    else:
                        char = ''
            key = char.lower()
            if key in ('w', 's', 'a', 'd', 'q', 'e', 'r', 'f', ' '):
                node.send(key)
                rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        run_keyboard(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().info(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
