#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from pymavlink import mavutil

from my_first_drone.config import (
    MAX_XY,
    MAX_YAW,
    MAX_Z,
    PX4_COMPONENT_ID,
    PX4_MAVLINK_PORT,
    PX4_SYSTEM_ID,
    SETPOINT_RATE,
    WARMUP_TIME,
    make_qos,
)

class PX4Backend(Node):
    """Receives ROS 2 commands and sends MAVLink commands to PX4."""

    def __init__(self):
        super().__init__('my_first_drone_backend')

        qos = make_qos()

        self.create_subscription(
            Twist, '/offboard_velocity_cmd', self.velocity_callback, qos
        )
        self.create_subscription(
            Bool, '/arm_message', self.arm_callback, qos
        )
        self.create_subscription(
            String, '/flight_command', self.flight_command_callback, qos
        )

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        self.armed = False
        self.offboard_started = False
        self.auto_takeoff_active = False
        self.last_cmd_time = time.monotonic()
        self._takeoff_timer = None

        self.mav = None
        self._connect_to_px4()

        self.timer = self.create_timer(
            1.0 / SETPOINT_RATE,
            self.control_loop,
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('MY FIRST DRONE - PX4 MAVLink BACKEND READY')
        self.get_logger().info(f'MAVLink UDP: 127.0.0.1:{PX4_MAVLINK_PORT}')
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    def _connect_to_px4(self):
        self.get_logger().info(
            f'Connecting to PX4 MAVLink on UDP {PX4_MAVLINK_PORT}...'
        )

        try:
            self.mav = mavutil.mavlink_connection(
                f'udpin:127.0.0.1:{PX4_MAVLINK_PORT}',
                source_system=2,
                source_component=191,
            )
            self.mav.wait_heartbeat(timeout=15)

            self.get_logger().info(
                'PX4 heartbeat received: '
                f'system={self.mav.target_system}, '
                f'component={self.mav.target_component}'
            )
        except Exception as exc:
            self.mav = None
            self.get_logger().error(f'PX4 MAVLink connection failed: {exc}')

    # ------------------------------------------------------------------
    def velocity_callback(self, msg: Twist):
        self.last_cmd_time = time.monotonic()

        self.vx = max(-MAX_XY, min(MAX_XY, float(msg.linear.x)))
        self.vy = max(-MAX_XY, min(MAX_XY, float(msg.linear.y)))

        # ROS +Z = UP, PX4 NED +Z = DOWN.
        self.vz = max(-MAX_Z, min(MAX_Z, -float(msg.linear.z)))
        self.yaw_rate = max(-MAX_YAW, min(MAX_YAW, float(msg.angular.z)))

    # ------------------------------------------------------------------
    def arm_callback(self, msg: Bool):
        if self.mav is None:
            self.get_logger().error('PX4 MAVLink is not connected.')
            return

        if msg.data and not self.armed:
            self.get_logger().info('ARM requested')

            # Prepare continuous offboard setpoints before arming.
            self.start_offboard()
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            self.send_arm_command(True)

            self.armed = True
            self.get_logger().info('ARM command sent')

        elif not msg.data and self.armed:
            self.get_logger().info('DISARM requested')
            self.send_arm_command(False)
            self.armed = False
            self.offboard_started = False
            self.auto_takeoff_active = False
            self._zero_velocity()

    # ------------------------------------------------------------------
    def flight_command_callback(self, msg: String):
        if self.mav is None:
            self.get_logger().error('PX4 MAVLink is not connected.')
            return

        command = msg.data.upper().strip()

        if command == 'TAKEOFF':
            self.takeoff()

        elif command in ('LAND', 'EMERGENCY_LAND'):
            self.land(emergency=(command == 'EMERGENCY_LAND'))

        elif command == 'RTL':
            self.rtl()

    # ------------------------------------------------------------------
    def takeoff(self):
        """Use PX4 AUTO_TAKEOFF rather than a fake 3-second throttle ramp."""
        self.get_logger().info('TAKEOFF requested')

        if not self.armed:
            self.send_arm_command(True)
            self.armed = True
            self.get_logger().info('ARM command sent for takeoff')

        # PX4 CUSTOM_MAIN_MODE_AUTO = 4
        # PX4 CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
        self.mav.mav.command_long_send(
            PX4_SYSTEM_ID,
            PX4_COMPONENT_ID,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,
            2,
            0,
            0,
            0,
            0,
        )

        # Give AUTO_TAKEOFF time to execute before allowing velocity/offboard
        # control to become active again.
        self.auto_takeoff_active = True
        self.offboard_started = False
        self.get_logger().info('AUTO_TAKEOFF command sent')

        # Keep the old project's ~3 second timing, then return to Offboard
        # so the arrow/W/S controls work again.
        if self._takeoff_timer is not None:
            self._takeoff_timer.cancel()
        self._takeoff_timer = self.create_timer(3.5, self._finish_takeoff_once)

    def _finish_takeoff_once(self):
        if self._takeoff_timer is not None:
            self._takeoff_timer.cancel()
            self._takeoff_timer = None

        self.auto_takeoff_active = False

        if self.mav is not None and self.armed:
            # Setpoint stream has been active throughout takeoff.
            self.send_offboard_command()
            self.offboard_started = True
            self.get_logger().info('OFFBOARD control restored after takeoff')

    # ------------------------------------------------------------------
    def land(self, emergency: bool = False):
        label = 'EMERGENCY LAND' if emergency else 'LAND'
        self.get_logger().warn(f'{label} requested')

        self._zero_velocity()
        self.auto_takeoff_active = False

        self.mav.mav.command_long_send(
            PX4_SYSTEM_ID,
            PX4_COMPONENT_ID,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

        self.offboard_started = False

    # ------------------------------------------------------------------
    def rtl(self):
        self.get_logger().info('RETURN TO LAUNCH requested')

        self._zero_velocity()
        self.auto_takeoff_active = False

        self.mav.mav.command_long_send(
            PX4_SYSTEM_ID,
            PX4_COMPONENT_ID,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

        self.offboard_started = False

    # ------------------------------------------------------------------
    def start_offboard(self):
        if self.mav is None:
            return

        start = time.monotonic()
        while time.monotonic() - start < WARMUP_TIME:
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            time.sleep(1.0 / SETPOINT_RATE)

        self.send_offboard_command()
        self.offboard_started = True
        self.get_logger().info('OFFBOARD command sent')

    # ------------------------------------------------------------------
    def send_arm_command(self, enable: bool):
        value = 1.0 if enable else 0.0

        self.mav.mav.command_long_send(
            PX4_SYSTEM_ID,
            PX4_COMPONENT_ID,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            value,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    # ------------------------------------------------------------------
    def send_offboard_command(self):
        self.mav.mav.command_long_send(
            PX4_SYSTEM_ID,
            PX4_COMPONENT_ID,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6,
            0,
            0,
            0,
            0,
            0,
        )

    # ------------------------------------------------------------------
    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        if self.mav is None:
            return

        # Ignore position (bits 0-2), acceleration (6-8), yaw (10).
        # Keep vx/vy/vz and yaw_rate active.
        type_mask = 0x5C7

        time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

        self.mav.mav.set_position_target_local_ned_send(
            time_boot_ms,
            PX4_SYSTEM_ID,
            PX4_COMPONENT_ID,
	    mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0.0, 0.0, 0.0,
            float(vx), float(vy), float(vz),
            0.0, 0.0, 0.0,
            0.0,
            float(yaw_rate),
        )

    # ------------------------------------------------------------------
    def _zero_velocity(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.mav is None:
            return

        now = time.monotonic()

        # Always maintain a high-rate setpoint stream while the ROS 2 node
        # is alive. PX4 Offboard needs continuous proof-of-life setpoints.
        if self.auto_takeoff_active:
            # AUTO_TAKEOFF itself controls the climb; send neutral setpoints.
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # Stop stale keyboard commands after a short timeout.
        if now - self.last_cmd_time > 0.30:
            self._zero_velocity()

        self.send_velocity(
            self.vx,
            self.vy,
            self.vz,
            self.yaw_rate,
        )

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if self.mav is not None:
                self.mav.close()
        except Exception:
            pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Keyboard node
# ---------------------------------------------------------------------------
