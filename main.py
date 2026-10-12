#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_first_drone.px4_controller import PX4Backend
from my_first_drone.keyboard_control import KeyboardController

def main(args=None):
    if not sys.stdin.isatty():
        print('ERROR: Keyboard controller needs a terminal (TTY).', file=sys.stderr)
        return 1

    original_terminal_settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)

    backend = PX4Backend()
    keyboard = KeyboardController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    executor.add_node(keyboard)

    try:
        tty.setcbreak(sys.stdin.fileno())
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            original_terminal_settings,
        )
        executor.shutdown()
        keyboard.destroy_node()
        backend.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
