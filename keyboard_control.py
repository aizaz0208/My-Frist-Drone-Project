#!/usr/bin/env python3

import select
import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from my_first_drone.config import (
    HOVER_THROTTLE,
    KEY_TIMEOUT,
    MAX_THROTTLE,
    MIN_THROTTLE,
    THROTTLE_INCREMENT,
    XY_VELOCITY,
    YAW_RATE,
    make_qos,
)


class KeyboardController(Node):

    def __init__(self):
        super().__init__("my_first_drone_keyboard")

        qos = make_qos()

        # ======================================================
        # ROS 2 PUBLISHERS
        # ======================================================

        self.velocity_pub = self.create_publisher(
            Twist,
            "/offboard_velocity_cmd",
            qos,
        )

        self.arm_pub = self.create_publisher(
            Bool,
            "/arm_message",
            qos,
        )

        self.command_pub = self.create_publisher(
            String,
            "/flight_command",
            qos,
        )

        # ======================================================
        # MOTION STATE
        # ======================================================

        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        self.throttle = HOVER_THROTTLE
        self.armed = False

        # Current pressed/held keys
        self.key_held_time = {}

        # ======================================================
        # CONTROLLER INFORMATION
        # ======================================================

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("MY FIRST DRONE KEYBOARD READY")
        self.get_logger().info("=" * 60)

        self.get_logger().info("SPACE        = ARM / DISARM")
        self.get_logger().info("T            = TAKEOFF")
        self.get_logger().info("X            = EMERGENCY LAND")
        self.get_logger().info("L            = LAND")
        self.get_logger().info("R            = RETURN TO LAUNCH")

        self.get_logger().info("W            = UP")
        self.get_logger().info("S            = DOWN")

        self.get_logger().info("UP ARROW     = FORWARD")
        self.get_logger().info("DOWN ARROW   = BACKWARD")
        self.get_logger().info("LEFT ARROW   = LEFT")
        self.get_logger().info("RIGHT ARROW  = RIGHT")

        self.get_logger().info("A            = YAW LEFT")
        self.get_logger().info("D            = YAW RIGHT")

        self.get_logger().info("=" * 60)

        # 20 Hz control loop
        self.timer = self.create_timer(
            0.05,
            self.loop,
        )

    # ==========================================================
    # KEYBOARD INPUT
    # ==========================================================

    def get_key(self):
        """
        Read one keyboard character without blocking.
        """

        ready, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0,
        )

        if ready:
            return sys.stdin.read(1)

        return None

    # ==========================================================
    # MAIN LOOP
    # ==========================================================

    def loop(self):

        now = time.monotonic()

        # ------------------------------------------------------
        # Expire old held keys
        # ------------------------------------------------------

        expired_keys = [
            key_name
            for key_name, timestamp in self.key_held_time.items()
            if now - timestamp > KEY_TIMEOUT
        ]

        for key_name in expired_keys:
            del self.key_held_time[key_name]

        # ------------------------------------------------------
        # Read keyboard
        # ------------------------------------------------------

        key = self.get_key()

        if key is not None:

            # Ctrl + C
            if key == "\x03":
                raise KeyboardInterrupt

            # --------------------------------------------------
            # SPACE = ARM / DISARM
            # --------------------------------------------------

            elif key == " ":

                self.armed = not self.armed

                arm_msg = Bool()
                arm_msg.data = self.armed

                self.arm_pub.publish(arm_msg)

                if self.armed:

                    self.get_logger().info(
                        "*** ARM requested ***"
                    )

                else:

                    self.get_logger().info(
                        "*** DISARM requested ***"
                    )

                    self.throttle = HOVER_THROTTLE

            # --------------------------------------------------
            # T = TAKEOFF
            # --------------------------------------------------

            elif key.lower() == "t":

                self.publish_command("TAKEOFF")

            # --------------------------------------------------
            # X = EMERGENCY LAND
            # --------------------------------------------------

            elif key.lower() == "x":

                self.publish_command("EMERGENCY_LAND")

            # --------------------------------------------------
            # L = LAND
            # --------------------------------------------------

            elif key.lower() == "l":

                self.publish_command("LAND")

            # --------------------------------------------------
            # R = RETURN TO LAUNCH
            # --------------------------------------------------

            elif key.lower() == "r":

                self.publish_command("RTL")

            # --------------------------------------------------
            # W = UP
            # --------------------------------------------------

            elif key.lower() == "w":

                self.throttle = min(
                    MAX_THROTTLE,
                    self.throttle + THROTTLE_INCREMENT,
                )

                self.get_logger().info(
                    f"UP: {self.throttle:.2f}"
                )

            # --------------------------------------------------
            # S = DOWN
            # --------------------------------------------------

            elif key.lower() == "s":

                self.throttle = max(
                    MIN_THROTTLE,
                    self.throttle - THROTTLE_INCREMENT,
                )

                self.get_logger().info(
                    f"DOWN: {self.throttle:.2f}"
                )

            # --------------------------------------------------
            # A = YAW LEFT
            # --------------------------------------------------

            elif key.lower() == "a":

                self.key_held_time["a"] = now

            # --------------------------------------------------
            # D = YAW RIGHT
            # --------------------------------------------------

            elif key.lower() == "d":

                self.key_held_time["d"] = now

            # --------------------------------------------------
            # ARROW KEYS
            #
            # ESC [ A = UP
            # ESC [ B = DOWN
            # ESC [ D = LEFT
            # ESC [ C = RIGHT
            # --------------------------------------------------

            elif key == "\x1b":

                seq = sys.stdin.read(2)

                if seq == "[A":

                    self.key_held_time["up"] = now

                elif seq == "[B":

                    self.key_held_time["down"] = now

                elif seq == "[D":

                    self.key_held_time["left"] = now

                elif seq == "[C":

                    self.key_held_time["right"] = now

        # ======================================================
        # RESET CURRENT MOTION
        # ======================================================

        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        # ======================================================
        # BODY-FRAME MOVEMENT
        #
        # PX4 BODY_NED:
        # +X = FORWARD
        # +Y = RIGHT
        # +Z = DOWN
        #
        # Therefore:
        #
        # UP    -> +X
        # DOWN  -> -X
        # LEFT  -> -Y
        # RIGHT -> +Y
        # ======================================================

        # UP ARROW = FORWARD
        if "up" in self.key_held_time:
            self.vx = XY_VELOCITY

        # DOWN ARROW = BACKWARD
        if "down" in self.key_held_time:
            self.vx = -XY_VELOCITY

        # LEFT ARROW = LEFT
        if "left" in self.key_held_time:
            self.vy = -XY_VELOCITY

        # RIGHT ARROW = RIGHT
        if "right" in self.key_held_time:
            self.vy = XY_VELOCITY

        # ======================================================
        # YAW
        # ======================================================

        # A = YAW LEFT
        if "a" in self.key_held_time:
            self.yaw_rate = -YAW_RATE

        # D = YAW RIGHT
        if "d" in self.key_held_time:
            self.yaw_rate = YAW_RATE

        # ======================================================
        # VERTICAL VELOCITY
        # ======================================================

        # Hover = 0.5
        # Above hover = UP
        # Below hover = DOWN

        vz_ros = (
            self.throttle - HOVER_THROTTLE
        ) * 2.0

        # ======================================================
        # PUBLISH VELOCITY
        # ======================================================

        twist = Twist()

        twist.linear.x = float(self.vx)
        twist.linear.y = float(self.vy)
        twist.linear.z = float(vz_ros)

        twist.angular.z = float(self.yaw_rate)

        self.velocity_pub.publish(twist)

    # ==========================================================
    # FLIGHT COMMAND
    # ==========================================================

    def publish_command(self, command):

        msg = String()
        msg.data = command

        self.command_pub.publish(msg)

        self.get_logger().info(
            f"*** {command} command sent ***"
        )

    # ==========================================================
    # CLEANUP
    # ==========================================================

    def destroy_node(self):

        try:

            twist = Twist()

            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0

            self.velocity_pub.publish(twist)

        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
