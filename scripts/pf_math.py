#!/usr/bin/env python3

"""Math helpers shared by the motion model, sensor model, and ROS node."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    """Extract yaw from a geometry_msgs/Quaternion-like object."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_ros_pose(pose_msg):
    """Convert a ROS pose message into a lightweight Pose2D."""
    return Pose2D(
        x=pose_msg.position.x,
        y=pose_msg.position.y,
        theta=wrap_angle(yaw_from_quaternion(pose_msg.orientation)),
    )


def quaternion_xyzw_from_yaw(yaw):
    """Convert yaw into an xyzw quaternion tuple."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))
