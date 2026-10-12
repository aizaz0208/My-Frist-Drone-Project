#!/usr/bin/env python3

import json
import socket

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


UDP_HOST = "0.0.0.0"
UDP_PORT = 5005

XY_SPEED = 1.0
Z_SPEED = 0.8
YAW_SPEED = 1.0

DEADZONE = 0.12


class MobileController(Node):

    def __init__(self):
        super().__init__('my_first_drone_mobile')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            10
        )

        self.arm_pub = self.create_publisher(
            Bool,
            '/arm_message',
            10
        )

        self.flight_pub = self.create_publisher(
            String,
            '/flight_command',
            10
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.setblocking(False)

        self.move_x = 0.0
        self.move_y = 0.0

        self.flight_x = 0.0
        self.flight_y = 0.0

        self.last_arm = None
        self.last_command = None

        self.timer = self.create_timer(0.02, self.receive_data)

        print("")
        print("====================================")
        print(" MY FIRST DRONE MOBILE CONTROLLER")
        print("====================================")
        print(f" UDP SERVER: {UDP_HOST}:{UDP_PORT}")
        print("")
        print("LEFT  JOYSTICK ID : move")
        print("RIGHT JOYSTICK ID : flight")
        print("")
        print("Left:")
        print("  UP    = forward")
        print("  DOWN  = backward")
        print("  LEFT  = left")
        print("  RIGHT = right")
        print("")
        print("Right:")
        print("  UP    = up")
        print("  DOWN  = down")
        print("  LEFT  = yaw left")
        print("  RIGHT = yaw right")
        print("")
        print("Waiting for DroidPad...")
        print("")

    def dz(self, value):
        if abs(value) < DEADZONE:
            return 0.0
        return value

    def receive_data(self):

        while True:
            try:
                raw, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")
                break

            try:
                data = json.loads(raw.decode("utf-8"))
            except Exception:
                continue

            msg_type = data.get("type")
            msg_id = data.get("id")

            # =========================
            # LEFT JOYSTICK
            # =========================
            if msg_type == "JOYSTICK" and msg_id == "move":

                self.move_x = self.dz(float(data.get("x", 0.0)))
                self.move_y = self.dz(float(data.get("y", 0.0)))

                twist = Twist()

                # Body frame
                twist.linear.x = self.move_y * XY_SPEED
                twist.linear.y = self.move_x * XY_SPEED

                # Keep right-stick values
                twist.linear.z = self.flight_y * Z_SPEED
                twist.angular.z = self.flight_x * YAW_SPEED

                self.cmd_pub.publish(twist)

            # =========================
            # RIGHT JOYSTICK
            # =========================
            elif msg_type == "JOYSTICK" and msg_id == "flight":

                self.flight_x = self.dz(float(data.get("x", 0.0)))
                self.flight_y = self.dz(float(data.get("y", 0.0)))

                twist = Twist()

                # Keep left-stick values
                twist.linear.x = self.move_y * XY_SPEED
                twist.linear.y = self.move_x * XY_SPEED

                twist.linear.z = self.flight_y * Z_SPEED
                twist.angular.z = self.flight_x * YAW_SPEED

                self.cmd_pub.publish(twist)

            # =========================
            # ARM / DISARM
            # =========================
            elif msg_type == "BUTTON" and msg_id == "arm":

                state = data.get("state")

                if state == "PRESS":
                    arm_msg = Bool()
                    arm_msg.data = True
                    self.arm_pub.publish(arm_msg)
                    print("ARM")

            # =========================
            # TAKEOFF
            # =========================
            elif msg_type == "BUTTON" and msg_id == "takeoff":

                if data.get("state") == "PRESS":
                    msg = String()
                    msg.data = "TAKEOFF"
                    self.flight_pub.publish(msg)
                    print("TAKEOFF")

            # =========================
            # LAND
            # =========================
            elif msg_type == "BUTTON" and msg_id == "land":

                if data.get("state") == "PRESS":
                    msg = String()
                    msg.data = "LAND"
                    self.flight_pub.publish(msg)
                    print("LAND")

            # =========================
            # EMERGENCY LAND
            # =========================
            elif msg_type == "BUTTON" and msg_id == "emergency":

                if data.get("state") == "PRESS":
                    msg = String()
                    msg.data = "EMERGENCY_LAND"
                    self.flight_pub.publish(msg)
                    print("EMERGENCY LAND")

            # =========================
            # RTL
            # =========================
            elif msg_type == "BUTTON" and msg_id == "rtl":

                if data.get("state") == "PRESS":
                    msg = String()
                    msg.data = "RTL"
                    self.flight_pub.publish(msg)
                    print("RTL")


def main(args=None):

    rclpy.init(args=args)

    node = MobileController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
