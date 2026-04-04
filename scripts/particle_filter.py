#!/usr/bin/env python3

"""Deliverable 1 runtime node and Deliverable 2 particle-filter foundation."""

import os
import traceback

import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from map_utils import LikelihoodFieldMap
from motion_model import OdometryMotionModel
from pf_math import pose_from_ros_pose, quaternion_xyzw_from_yaw
from sensor_model import LikelihoodFieldSensorModel


class Deliverable1Node:
    """Runs the motion model and sensor model live in Gazebo."""

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

        self.motion_sample_count = int(rospy.get_param("~motion_sample_count", 150))
        self.motion_model = OdometryMotionModel(
            alpha1=rospy.get_param("~motion_alpha1", 0.05),
            alpha2=rospy.get_param("~motion_alpha2", 0.05),
            alpha3=rospy.get_param("~motion_alpha3", 0.10),
            alpha4=rospy.get_param("~motion_alpha4", 0.05),
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

        self.current_pose_pub = rospy.Publisher(
            "/particle_filter/current_pose", PoseStamped, queue_size=5
        )
        self.motion_samples_pub = rospy.Publisher(
            "/motion_model_samples", PoseArray, queue_size=1
        )
        self.sensor_score_pub = rospy.Publisher(
            "/particle_filter/sensor_model_log_likelihood",
            Float64,
            queue_size=10,
        )

        self.last_odom_pose = None
        self.current_odom_pose = None

        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=10)

        rospy.loginfo(
            "Deliverable 1 node ready. map_yaml=%s beam_step=%d sigma_hit=%.3f",
            self.map_yaml,
            self.sensor_model.beam_step,
            self.sensor_model.sigma_hit,
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
        self.publish_current_pose(odom_msg.header.stamp, pose)

        if self.last_odom_pose is not None and self.motion_sample_count > 0:
            self.publish_motion_samples(
                odom_msg.header.stamp, self.last_odom_pose, pose
            )

        self.last_odom_pose = pose

    def scan_callback(self, scan_msg):
        if self.current_odom_pose is None:
            return

        score = self.sensor_model.score_scan(self.current_odom_pose, scan_msg)
        self.sensor_score_pub.publish(Float64(data=score["log_likelihood"]))
        rospy.loginfo_throttle(
            2.0,
            (
                "Sensor model log-likelihood %.3f using %d beams "
                "(skipped %d max-range beams)"
            ),
            score["log_likelihood"],
            score["used_beams"],
            score["skipped_max_range"],
        )

    def publish_current_pose(self, stamp, pose):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "odom"
        pose_msg.pose = self.to_ros_pose(pose)
        self.current_pose_pub.publish(pose_msg)

    def publish_motion_samples(self, stamp, odom_prev, odom_curr):
        samples_msg = PoseArray()
        samples_msg.header.stamp = stamp
        samples_msg.header.frame_id = "odom"

        for _ in range(self.motion_sample_count):
            sampled_pose = self.motion_model.sample_motion(
                pose=odom_prev,
                odom_prev=odom_prev,
                odom_curr=odom_curr,
            )
            samples_msg.poses.append(self.to_ros_pose(sampled_pose))

        self.motion_samples_pub.publish(samples_msg)

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


def main():
    rospy.init_node("particle_filter", anonymous=False)
    try:
        Deliverable1Node()
        rospy.spin()
    except Exception as exc:  # pragma: no cover - ROS fatal path
        rospy.logfatal("Failed to start Deliverable 1 node: %s", exc)
        rospy.logfatal(traceback.format_exc())
        raise


if __name__ == "__main__":
    main()
