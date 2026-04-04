#!/usr/bin/env python3

"""Likelihood-field sensor model for 2D LiDAR scans."""

import math


class LikelihoodFieldSensorModel:
    def __init__(
        self,
        likelihood_field_map,
        sigma_hit=0.20,
        z_hit=1.0,
        z_rand=0.0,
        beam_step=8,
        max_usable_range=None,
        out_of_bounds_distance=None,
        min_probability=1.0e-9,
    ):
        self.likelihood_field_map = likelihood_field_map
        self.sigma_hit = float(sigma_hit)
        self.z_hit = float(z_hit)
        self.z_rand = float(z_rand)
        self.beam_step = max(1, int(beam_step))
        self.max_usable_range = (
            None if max_usable_range is None else float(max_usable_range)
        )
        self.out_of_bounds_distance = (
            likelihood_field_map.max_distance
            if out_of_bounds_distance is None
            else float(out_of_bounds_distance)
        )
        self.min_probability = float(min_probability)
        self._gaussian_norm = 1.0 / (
            math.sqrt(2.0 * math.pi) * max(self.sigma_hit, 1.0e-9)
        )

    def beam_probability(self, distance, usable_range):
        hit_prob = self.z_hit * self._gaussian_norm * math.exp(
            -0.5 * (distance / max(self.sigma_hit, 1.0e-9)) ** 2
        )
        if self.z_rand > 0.0 and usable_range > 0.0:
            hit_prob += self.z_rand / usable_range
        return max(hit_prob, self.min_probability)

    def scan_log_likelihood(self, pose, scan_msg):
        return self.score_scan(pose, scan_msg)["log_likelihood"]

    def score_scan(self, pose, scan_msg):
        usable_range = self.max_usable_range
        if usable_range is None:
            usable_range = float(getattr(scan_msg, "range_max", 0.0))

        log_likelihood = 0.0
        used_beams = 0
        skipped_max_range = 0

        for index in range(0, len(scan_msg.ranges), self.beam_step):
            measured_range = float(scan_msg.ranges[index])
            if not math.isfinite(measured_range):
                continue

            if measured_range < float(getattr(scan_msg, "range_min", 0.0)):
                continue

            if usable_range > 0.0 and measured_range >= usable_range:
                skipped_max_range += 1
                continue

            beam_angle = scan_msg.angle_min + index * scan_msg.angle_increment
            endpoint_x = pose.x + measured_range * math.cos(pose.theta + beam_angle)
            endpoint_y = pose.y + measured_range * math.sin(pose.theta + beam_angle)
            distance = self.likelihood_field_map.distance_at_world(
                endpoint_x,
                endpoint_y,
                default=self.out_of_bounds_distance,
            )
            probability = self.beam_probability(distance, usable_range)
            log_likelihood += math.log(probability)
            used_beams += 1

        return {
            "log_likelihood": log_likelihood if used_beams > 0 else 0.0,
            "used_beams": used_beams,
            "skipped_max_range": skipped_max_range,
        }
