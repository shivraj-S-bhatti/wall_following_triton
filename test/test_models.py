#!/usr/bin/env python3

import math
import os
import sys
import tempfile
import unittest

import numpy as np
import yaml
from PIL import Image


TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(TEST_DIR)
SCRIPTS_DIR = os.path.join(PACKAGE_DIR, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from map_utils import LikelihoodFieldMap
from motion_model import OdometryMotionModel
from particle_filter_core import (
    Particle,
    estimate_pose,
    effective_sample_size,
    initialize_global_particles,
    low_variance_resample,
    normalize_log_weights,
    normalize_particle_log_weights,
)
from pf_math import Pose2D
from sensor_model import LikelihoodFieldSensorModel


class FakeScan:
    def __init__(self, ranges, angle_min=0.0, angle_increment=0.0):
        self.ranges = list(ranges)
        self.angle_min = float(angle_min)
        self.angle_increment = float(angle_increment)
        self.range_min = 0.0
        self.range_max = 10.0


class Deliverable1ModelTests(unittest.TestCase):
    def create_temp_map(self):
        temp_dir = tempfile.TemporaryDirectory()
        image = np.full((10, 10), 255, dtype=np.uint8)
        image[4, 6] = 0
        image_path = os.path.join(temp_dir.name, "test_map.png")
        yaml_path = os.path.join(temp_dir.name, "test_map.yaml")
        Image.fromarray(image, mode="L").save(image_path)
        with open(yaml_path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(
                {
                    "image": os.path.basename(image_path),
                    "resolution": 1.0,
                    "origin": [0.0, 0.0, 0.0],
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                },
                handle,
            )
        return temp_dir, yaml_path

    def test_distance_field_matches_expected_cells(self):
        temp_dir, yaml_path = self.create_temp_map()
        try:
            field_map = LikelihoodFieldMap.load_or_create(yaml_path)
            self.assertAlmostEqual(field_map.distance_at_world(6.5, 5.5), 0.0)
            self.assertAlmostEqual(field_map.distance_at_world(5.5, 5.5), 1.0)
            self.assertAlmostEqual(
                field_map.distance_at_world(5.5, 4.5), math.sqrt(2.0)
            )
        finally:
            temp_dir.cleanup()

    def test_motion_model_without_noise_applies_relative_delta(self):
        model = OdometryMotionModel(0.0, 0.0, 0.0, 0.0)
        odom_prev = Pose2D(0.0, 0.0, 0.0)
        odom_curr = Pose2D(1.0, 0.0, 0.0)
        particle = Pose2D(0.0, 0.0, math.pi / 2.0)

        sampled = model.sample_motion(
            pose=particle,
            odom_prev=odom_prev,
            odom_curr=odom_curr,
        )
        self.assertAlmostEqual(sampled.x, 0.0, places=6)
        self.assertAlmostEqual(sampled.y, 1.0, places=6)
        self.assertAlmostEqual(sampled.theta, math.pi / 2.0, places=6)

    def test_sensor_model_prefers_pose_that_matches_obstacle_endpoint(self):
        temp_dir, yaml_path = self.create_temp_map()
        try:
            field_map = LikelihoodFieldMap.load_or_create(yaml_path)
            sensor_model = LikelihoodFieldSensorModel(
                field_map,
                sigma_hit=0.20,
                beam_step=1,
                max_usable_range=10.0,
            )
            scan = FakeScan([2.0])

            matching_pose = Pose2D(4.5, 5.5, 0.0)
            shifted_pose = Pose2D(4.5, 2.5, 0.0)

            matching_score = sensor_model.scan_log_likelihood(matching_pose, scan)
            shifted_score = sensor_model.scan_log_likelihood(shifted_pose, scan)
            self.assertGreater(matching_score, shifted_score)
        finally:
            temp_dir.cleanup()

    def test_particle_filter_core_normalizes_estimates_and_resamples(self):
        weights = normalize_log_weights([-100.0, -10.0, -10.0])
        self.assertAlmostEqual(sum(weights), 1.0)
        self.assertLess(weights[0], weights[1])

        particles = [
            Particle(Pose2D(0.0, 0.0, 0.0), 0.25),
            Particle(Pose2D(2.0, 0.0, 0.0), 0.75),
        ]
        estimated = estimate_pose(particles)
        self.assertAlmostEqual(estimated.x, 1.5)
        self.assertGreater(effective_sample_size([0.5, 0.5]), 1.9)

        resampled = low_variance_resample(particles)
        self.assertEqual(len(resampled), len(particles))
        self.assertAlmostEqual(sum(p.weight for p in resampled), 1.0)

    def test_particle_weight_update_keeps_prior_belief(self):
        particles = [
            Particle(Pose2D(0.0, 0.0, 0.0), 0.80),
            Particle(Pose2D(1.0, 0.0, 0.0), 0.20),
        ]

        equal_likelihood_weights = normalize_particle_log_weights(
            particles, [0.0, 0.0]
        )
        self.assertAlmostEqual(equal_likelihood_weights[0], 0.80)
        self.assertAlmostEqual(equal_likelihood_weights[1], 0.20)

        corrected_weights = normalize_particle_log_weights(
            particles, [math.log(0.5), math.log(2.0)]
        )
        self.assertLess(corrected_weights[0], 0.80)
        self.assertGreater(corrected_weights[1], 0.20)

    def test_global_particle_initialization_stays_in_free_space(self):
        temp_dir, yaml_path = self.create_temp_map()
        try:
            field_map = LikelihoodFieldMap.load_or_create(yaml_path)
            particles = initialize_global_particles(field_map, 20)
            self.assertEqual(len(particles), 20)
            self.assertTrue(
                all(field_map.is_free_world(p.pose.x, p.pose.y) for p in particles)
            )
        finally:
            temp_dir.cleanup()


if __name__ == "__main__":
    unittest.main()
