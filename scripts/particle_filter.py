#!/usr/bin/env python3

"""Monte Carlo localization node for COMPSCI 603 Project 3 Deliverable 2."""

import csv
import math
import os
import random
import sys
import traceback


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from map_utils import LikelihoodFieldMap
from motion_model import OdometryMotionModel
from particle_filter_core import (
    Particle,
    effective_sample_size,
    estimate_pose,
    initialize_global_particles,
    low_variance_resample,
    normalize_log_weights,
    position_error,
    yaw_error,
)
from pf_math import pose_from_ros_pose, quaternion_xyzw_from_yaw
from sensor_model import LikelihoodFieldSensorModel


class CsvRunLogger:
    def __init__(self, output_path):
        self.output_path = output_path
        self.file_handle = None
        self.writer = None
        if output_path:
            directory = os.path.dirname(output_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            self.file_handle = open(output_path, "w", newline="", encoding="utf-8")
            fieldnames = [
                "time_sec",
                "iteration",
                "true_x",
                "true_y",
                "true_theta",
                "estimate_x",
                "estimate_y",
                "estimate_theta",
                "position_error_m",
                "yaw_error_rad",
                "neff",
                "max_weight",
                "used_beams",
                "mean_log_likelihood",
            ]
            self.writer = csv.DictWriter(self.file_handle, fieldnames=fieldnames)
            self.writer.writeheader()

    def write(self, row):
        if not self.writer:
            return
        self.writer.writerow(row)
        self.file_handle.flush()

    def close(self):
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None


class ParticleFilterNode:
    """Runs a complete predict-correct-resample localization loop."""

    def __init__(self):
        self.map_yaml = self._required_path("~map_yaml")
        self.metric_map_output_png = rospy.get_param("~metric_map_output_png", "")
        self.likelihood_field_output_png = rospy.get_param(
            "~likelihood_field_output_png", ""
        )
        self.likelihood_field_cache = rospy.get_param("~likelihood_field_cache", "")
        self.regenerate_likelihood_field = bool(
            rospy.get_param("~regenerate_likelihood_field", False)
        )

        self.rng = random.Random(int(rospy.get_param("~random_seed", 603)))
        self.num_particles = int(rospy.get_param("~num_particles", 350))
        self.init_mode = rospy.get_param("~init_mode", "global")
        self.initial_pose_std_xy = float(rospy.get_param("~initial_pose_std_xy", 0.35))
        self.initial_pose_std_yaw = float(rospy.get_param("~initial_pose_std_yaw", 0.80))
        self.random_particle_ratio = float(rospy.get_param("~random_particle_ratio", 0.02))
        self.invalid_pose_log_likelihood = float(
            rospy.get_param("~invalid_pose_log_likelihood", -1000000.0)
        )
        self.publish_map_to_odom_tf = bool(
            rospy.get_param("~publish_map_to_odom_tf", True)
        )
        self.resample_enabled = bool(rospy.get_param("~resample_enabled", True))
        self.resample_min_neff_ratio = float(
            rospy.get_param("~resample_min_neff_ratio", 1.0)
        )
        self.publish_every_n_scans = max(
            1, int(rospy.get_param("~publish_every_n_scans", 1))
        )

        self.motion_model = OdometryMotionModel(
            alpha1=rospy.get_param("~motion_alpha1", 0.05),
            alpha2=rospy.get_param("~motion_alpha2", 0.05),
            alpha3=rospy.get_param("~motion_alpha3", 0.10),
            alpha4=rospy.get_param("~motion_alpha4", 0.05),
            rng=self.rng,
        )

        self.likelihood_field_map = LikelihoodFieldMap.load_or_create(
            map_yaml_path=self.map_yaml,
            cache_path=self.likelihood_field_cache or None,
            regenerate=self.regenerate_likelihood_field,
        )
        if self.metric_map_output_png:
            self.likelihood_field_map.save_metric_map_png(self.metric_map_output_png)
        if self.likelihood_field_output_png:
            self.likelihood_field_map.save_likelihood_field_png(
                self.likelihood_field_output_png
            )

        self.sensor_model = LikelihoodFieldSensorModel(
            likelihood_field_map=self.likelihood_field_map,
            sigma_hit=rospy.get_param("~sigma_hit", 0.20),
            z_hit=rospy.get_param("~z_hit", 1.0),
            z_rand=rospy.get_param("~z_rand", 0.0),
            beam_step=rospy.get_param("~beam_step", 8),
            max_usable_range=rospy.get_param("~max_usable_range", 3.5),
            out_of_bounds_distance=rospy.get_param("~out_of_bounds_distance", 2.5),
            min_probability=rospy.get_param("~likelihood_epsilon", 1.0e-9),
        )

        self.particles = []
        if self.init_mode == "global":
            self.particles = initialize_global_particles(
                self.likelihood_field_map, self.num_particles, self.rng
            )
        self.last_odom_pose = None
        self.current_odom_pose = None
        self.last_estimate = None
        self.iteration = 0
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.run_logger = CsvRunLogger(rospy.get_param("~csv_log_path", ""))
        rospy.on_shutdown(self.run_logger.close)

        self.current_pose_pub = rospy.Publisher(
            "/particle_filter/current_pose", PoseStamped, queue_size=5
        )
        self.estimated_pose_pub = rospy.Publisher(
            "/particle_filter/estimated_pose", PoseStamped, queue_size=5
        )
        self.particles_pub = rospy.Publisher(
            "/particle_filter/particles", PoseArray, queue_size=1
        )
        self.sensor_score_pub = rospy.Publisher(
            "/particle_filter/mean_log_likelihood", Float64, queue_size=10
        )
        self.neff_pub = rospy.Publisher(
            "/particle_filter/effective_sample_size", Float64, queue_size=10
        )

        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=10)

        if self.particles:
            self.publish_particles(rospy.Time.now())
        rospy.loginfo(
            (
                "Particle filter ready. init_mode=%s particles=%d beam_step=%d "
                "sigma_hit=%.3f resample_ratio=%.2f"
            ),
            self.init_mode,
            self.num_particles,
            self.sensor_model.beam_step,
            self.sensor_model.sigma_hit,
            self.resample_min_neff_ratio,
        )

    def _required_path(self, param_name):
        value = rospy.get_param(param_name, "").strip()
        if not value:
            raise ValueError(f"Required ROS parameter {param_name} is empty.")
        if not os.path.exists(value):
            raise FileNotFoundError(
                f"Required file for {param_name} does not exist: {value}"
            )
        return value

    def odom_callback(self, odom_msg):
        pose = pose_from_ros_pose(odom_msg.pose.pose)
        self.current_odom_pose = pose
        if not self.particles:
            self.initialize_particles_from_pose(pose, odom_msg.header.stamp)

        self.publish_pose(
            self.current_pose_pub,
            "/particle_filter/current_pose",
            odom_msg.header.stamp,
            pose,
        )

        if self.last_odom_pose is not None:
            self.predict(self.last_odom_pose, pose)

        self.last_odom_pose = pose

    def scan_callback(self, scan_msg):
        if self.current_odom_pose is None:
            return
        if not self.particles:
            return

        self.iteration += 1
        details = self.correct(scan_msg)
        estimate = estimate_pose(self.particles)
        self.last_estimate = estimate

        weights = [particle.weight for particle in self.particles]
        neff = effective_sample_size(weights)
        self.neff_pub.publish(Float64(data=neff))
        self.sensor_score_pub.publish(Float64(data=details["mean_log_likelihood"]))

        self.publish_pose(
            self.estimated_pose_pub,
            "/particle_filter/estimated_pose",
            scan_msg.header.stamp,
            estimate,
            frame_id="map",
        )
        if self.iteration % self.publish_every_n_scans == 0:
            self.publish_particles(scan_msg.header.stamp)

        self.log_iteration(scan_msg.header.stamp, estimate, neff, details)
        if self.publish_map_to_odom_tf:
            self.publish_map_to_odom(scan_msg.header.stamp, estimate)

        threshold = self.resample_min_neff_ratio * len(self.particles)
        if self.resample_enabled and neff <= threshold:
            self.particles = low_variance_resample(self.particles, self.rng)
            self.inject_random_particles()
            if self.iteration % self.publish_every_n_scans == 0:
                self.publish_particles(scan_msg.header.stamp)

        rospy.loginfo_throttle(
            2.0,
            (
                "PF iter=%d neff=%.1f pos_err=%.3f yaw_err=%.3f "
                "used_beams=%d"
            ),
            self.iteration,
            neff,
            position_error(estimate, self.current_odom_pose),
            yaw_error(estimate.theta, self.current_odom_pose.theta),
            details["used_beams"],
        )

    def predict(self, odom_prev, odom_curr):
        predicted = []
        for particle in self.particles:
            predicted_pose = self.motion_model.sample_motion(
                pose=particle.pose,
                odom_prev=odom_prev,
                odom_curr=odom_curr,
            )
            predicted.append(Particle(predicted_pose, particle.weight))
        self.particles = predicted

    def correct(self, scan_msg):
        log_likelihoods = []
        used_beams = 0
        skipped_max_range = 0

        for particle in self.particles:
            if not self.likelihood_field_map.is_free_world(
                particle.pose.x, particle.pose.y
            ):
                log_likelihoods.append(self.invalid_pose_log_likelihood)
                continue

            score = self.sensor_model.score_scan(particle.pose, scan_msg)
            log_likelihoods.append(score["log_likelihood"])
            used_beams = max(used_beams, score["used_beams"])
            skipped_max_range = max(skipped_max_range, score["skipped_max_range"])

        weights = normalize_log_weights(log_likelihoods)
        for particle, weight in zip(self.particles, weights):
            particle.weight = weight

        mean_log_likelihood = (
            sum(log_likelihoods) / len(log_likelihoods) if log_likelihoods else 0.0
        )
        return {
            "used_beams": used_beams,
            "skipped_max_range": skipped_max_range,
            "mean_log_likelihood": mean_log_likelihood,
            "max_weight": max(weights) if weights else 0.0,
        }

    def initialize_particles_from_pose(self, pose, stamp):
        if self.init_mode == "global":
            self.particles = initialize_global_particles(
                self.likelihood_field_map, self.num_particles, self.rng
            )
        elif self.init_mode == "odom_gaussian":
            self.particles = self.sample_gaussian_particles(pose)
        else:
            rospy.logwarn(
                "Unknown init_mode=%s; falling back to global initialization.",
                self.init_mode,
            )
            self.particles = initialize_global_particles(
                self.likelihood_field_map, self.num_particles, self.rng
            )
        self.publish_particles(stamp)

    def sample_gaussian_particles(self, center_pose):
        particles = []
        weight = 1.0 / float(self.num_particles)
        attempts = 0
        while len(particles) < self.num_particles and attempts < self.num_particles * 20:
            attempts += 1
            sample_x = center_pose.x + self.rng.gauss(0.0, self.initial_pose_std_xy)
            sample_y = center_pose.y + self.rng.gauss(0.0, self.initial_pose_std_xy)
            sample_theta = center_pose.theta + self.rng.gauss(
                0.0, self.initial_pose_std_yaw
            )
            if self.likelihood_field_map.is_free_world(sample_x, sample_y):
                particles.append(
                    Particle(
                        self.to_pose2d(sample_x, sample_y, sample_theta),
                        weight,
                    )
                )

        while len(particles) < self.num_particles:
            particles.append(Particle(center_pose, weight))
        return particles

    def inject_random_particles(self):
        if self.random_particle_ratio <= 0.0:
            return
        inject_count = int(round(self.random_particle_ratio * len(self.particles)))
        if inject_count <= 0:
            return
        new_particles = initialize_global_particles(
            self.likelihood_field_map, inject_count, self.rng
        )
        self.particles[-inject_count:] = new_particles
        equal_weight = 1.0 / float(len(self.particles))
        for particle in self.particles:
            particle.weight = equal_weight

    def publish_particles(self, stamp):
        particles_msg = PoseArray()
        particles_msg.header.stamp = stamp
        particles_msg.header.frame_id = "map"
        particles_msg.poses = [self.to_ros_pose(particle.pose) for particle in self.particles]
        self.particles_pub.publish(particles_msg)

    def publish_pose(self, publisher, topic_name, stamp, pose, frame_id="odom"):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose = self.to_ros_pose(pose)
        publisher.publish(pose_msg)

    def publish_map_to_odom(self, stamp, estimate):
        if self.current_odom_pose is None:
            return

        transform_yaw = estimate.theta - self.current_odom_pose.theta
        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)
        odom_x = self.current_odom_pose.x
        odom_y = self.current_odom_pose.y
        transform_x = estimate.x - (cos_yaw * odom_x - sin_yaw * odom_y)
        transform_y = estimate.y - (sin_yaw * odom_x + cos_yaw * odom_y)
        quat_x, quat_y, quat_z, quat_w = quaternion_xyzw_from_yaw(transform_yaw)
        self.tf_broadcaster.sendTransform(
            (transform_x, transform_y, 0.0),
            (quat_x, quat_y, quat_z, quat_w),
            stamp,
            "odom",
            "map",
        )

    def log_iteration(self, stamp, estimate, neff, details):
        if self.current_odom_pose is None:
            return
        self.run_logger.write(
            {
                "time_sec": stamp.to_sec() if hasattr(stamp, "to_sec") else 0.0,
                "iteration": self.iteration,
                "true_x": self.current_odom_pose.x,
                "true_y": self.current_odom_pose.y,
                "true_theta": self.current_odom_pose.theta,
                "estimate_x": estimate.x,
                "estimate_y": estimate.y,
                "estimate_theta": estimate.theta,
                "position_error_m": position_error(estimate, self.current_odom_pose),
                "yaw_error_rad": yaw_error(estimate.theta, self.current_odom_pose.theta),
                "neff": neff,
                "max_weight": details["max_weight"],
                "used_beams": details["used_beams"],
                "mean_log_likelihood": details["mean_log_likelihood"],
            }
        )

    @staticmethod
    def to_ros_pose(pose):
        ros_pose = Pose()
        ros_pose.position.x = pose.x
        ros_pose.position.y = pose.y
        ros_pose.position.z = 0.0
        quat_x, quat_y, quat_z, quat_w = quaternion_xyzw_from_yaw(pose.theta)
        ros_pose.orientation.x = quat_x
        ros_pose.orientation.y = quat_y
        ros_pose.orientation.z = quat_z
        ros_pose.orientation.w = quat_w
        return ros_pose

    @staticmethod
    def to_pose2d(x, y, theta):
        from pf_math import Pose2D, wrap_angle

        return Pose2D(x, y, wrap_angle(theta))


def main():
    rospy.init_node("particle_filter", anonymous=False)
    try:
        ParticleFilterNode()
        rospy.spin()
    except Exception as exc:  # pragma: no cover - ROS fatal path
        rospy.logfatal("Failed to start particle filter node: %s", exc)
        rospy.logfatal(traceback.format_exc())
        raise


if __name__ == "__main__":
    main()
