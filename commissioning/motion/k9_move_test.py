#!/usr/bin/env python3
"""
Simple Logo-style movement test utility for K9.

The program deliberately talks only to the standard ROS 2 differential-drive
interface.  It therefore works with both:

  * the real K9 RoboClaw ros2_control hardware; and
  * Gazebo, provided the simulated robot uses diff_drive_controller.

Default interfaces:

  /diff_drive_controller/cmd_vel   geometry_msgs/msg/TwistStamped
  /diff_drive_controller/odom      nav_msgs/msg/Odometry

Examples:

  ros2 run k9_ros2_nav logo fd 1.0
  ros2 run k9_ros2_nav logo bk 0.5
  ros2 run k9_ros2_nav logo lt 1.5708
  ros2 run k9_ros2_nav logo rt 3.14159
  ros2 run k9_ros2_nav logo arc 1.0 1.5708
  ros2 run k9_ros2_nav logo arc 1.0 -1.5708
  logo stop

Distances and radii are metres. Angles are radians.

The diff_drive_controller owns acceleration/deceleration limiting.  This
utility sends target body velocities and uses controller odometry to decide
when to command zero velocity.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Usage: k9_move_test.py [options] <command> [<arg> ...]

DEFAULT_CMD_VEL_TOPIC = "/diff_drive_controller/cmd_vel"
DEFAULT_ODOM_TOPIC = "/diff_drive_controller/odom"

# Conservative defaults for calibration/testing.  The controller itself
# enforces the authoritative K9 maximum velocity and acceleration limits.
DEFAULT_LINEAR_SPEED = 0.25       # m/s
DEFAULT_ANGULAR_SPEED = 0.50      # rad/s
DEFAULT_PUBLISH_RATE = 20.0       # Hz

# Stop a little before the geometric target so controller deceleration has
# room to act.  These can be overridden on the command line.
DEFAULT_LINEAR_TOLERANCE = 0.010  # m
DEFAULT_ANGULAR_TOLERANCE = 0.010 # rad

# Match k9_drive_pkg/config/drive_controller.yaml.  These are magnitudes.
# They are used only to estimate when to start commanding zero; the controller
# remains authoritative for actually applying its limits.
DEFAULT_LINEAR_DECEL = 0.557824       # m/s^2
DEFAULT_ANGULAR_DECEL = 5.5175469832  # rad/s^2

ODOM_WAIT_TIMEOUT = 5.0           # s
ODOM_STALE_TIMEOUT = 0.5          # s
DEFAULT_COMMAND_TIMEOUT = 120.0   # s


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def normalise_angle(angle: float) -> float:
    """Normalise an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class K9MoveTest(Node):
    def __init__(
        self,