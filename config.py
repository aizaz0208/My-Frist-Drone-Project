#!/usr/bin/env python3

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# PX4 / MAVLink settings
PX4_SYSTEM_ID = 1
PX4_COMPONENT_ID = 1
PX4_MAVLINK_PORT = 14540

SETPOINT_RATE = 20.0
WARMUP_TIME = 2.0

MAX_XY = 3.0
MAX_Z = 2.0
MAX_YAW = 1.5

# Keyboard settings
XY_VELOCITY = 0.8
YAW_RATE = 1.5
THROTTLE_INCREMENT = 0.05
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0
HOVER_THROTTLE = 0.5
KEY_TIMEOUT = 0.15


def make_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )
