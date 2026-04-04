#!/usr/bin/env python3

"""Odometry-based motion model utilities for particle propagation."""

import math
import random

from pf_math import Pose2D, wrap_angle


class OdometryMotionModel:
    """Implements the odometry RTR motion model from Probabilistic Robotics."""

    def __init__(
        self,
        alpha1=0.05,
        alpha2=0.05,
        alpha3=0.10,
        alpha4=0.05,
        rng=None,
    ):
        self.alpha1 = float(alpha1)
        self.alpha2 = float(alpha2)
        self.alpha3 = float(alpha3)
        self.alpha4 = float(alpha4)
        self.rng = rng if rng is not None else random.Random()

    def decompose_odom_delta(self, odom_prev, odom_curr):
        """Return the RTR decomposition of the odometry motion."""
        delta_x = odom_curr.x - odom_prev.x
        delta_y = odom_curr.y - odom_prev.y
        delta_trans = math.hypot(delta_x, delta_y)

        if delta_trans < 1.0e-9:
            delta_rot1 = 0.0
        else:
            delta_rot1 = wrap_angle(
                math.atan2(delta_y, delta_x) - odom_prev.theta
            )

        delta_rot2 = wrap_angle(
            odom_curr.theta - odom_prev.theta - delta_rot1
        )
        return delta_rot1, delta_trans, delta_rot2

    def sample_motion(self, pose, odom_prev, odom_curr):
        """Sample a noisy next pose using a particle pose and odometry delta."""
        delta_rot1, delta_trans, delta_rot2 = self.decompose_odom_delta(
            odom_prev, odom_curr
        )

        delta_rot1_hat = delta_rot1 - self._sample_noise(
            self.alpha1 * delta_rot1 * delta_rot1
            + self.alpha2 * delta_trans * delta_trans
        )
        delta_trans_hat = delta_trans - self._sample_noise(
            self.alpha3 * delta_trans * delta_trans
            + self.alpha4 * (delta_rot1 * delta_rot1 + delta_rot2 * delta_rot2)
        )
        delta_rot2_hat = delta_rot2 - self._sample_noise(
            self.alpha1 * delta_rot2 * delta_rot2
            + self.alpha2 * delta_trans * delta_trans
        )

        theta = wrap_angle(pose.theta + delta_rot1_hat)
        return Pose2D(
            x=pose.x + delta_trans_hat * math.cos(theta),
            y=pose.y + delta_trans_hat * math.sin(theta),
            theta=wrap_angle(theta + delta_rot2_hat),
        )

    def _sample_noise(self, variance):
        std_dev = math.sqrt(max(0.0, variance))
        if std_dev == 0.0:
            return 0.0
        return self.rng.gauss(0.0, std_dev)
