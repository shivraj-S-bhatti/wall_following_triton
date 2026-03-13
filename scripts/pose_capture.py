#!/usr/bin/env python3
"""Capture Triton start poses from Gazebo while teleoperating manually."""

import math
import os
import select
import re
import sys
import termios
import tty
from typing import Dict, Optional, Tuple

import rospy
import yaml
from gazebo_msgs.msg import ModelStates


class PoseCapture:
    def __init__(self):
        self.robot_model_name = rospy.get_param("~robot_model_name", "triton")
        self.output_path = os.path.expanduser(
            str(
                rospy.get_param(
                    "~output_path",
                    "~/catkin_ws/src/wall_following_triton/artifacts/captured_start_poses.yaml",
                )
            )
        )
        self.named_output_path = os.path.expanduser(
            str(
                rospy.get_param(
                    "~named_output_path",
                    "~/catkin_ws/src/wall_following_triton/artifacts/captured_named_poses.yaml",
                )
            )
        )
        self.latest_pose: Optional[Tuple[float, float, float, float]] = None
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)

    def _on_model_states(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        yaw = math.atan2(
            2.0 * (float(pose.orientation.w) * float(pose.orientation.z) + float(pose.orientation.x) * float(pose.orientation.y)),
            1.0 - 2.0 * (float(pose.orientation.y) ** 2 + float(pose.orientation.z) ** 2),
        )
        self.latest_pose = (
            round(float(pose.position.x), 3),
            round(float(pose.position.y), 3),
            round(float(pose.position.z), 3),
            round(yaw, 3),
        )

    def _format_pose(self) -> Optional[str]:
        if self.latest_pose is None:
            return None
        x, y, z, yaw = self.latest_pose
        return f"  - {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, yaw: {yaw:.3f}}}"

    def _append_pose(self, line: str):
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        file_exists = os.path.exists(self.output_path)
        with open(self.output_path, "a", encoding="utf-8") as f:
            if not file_exists:
                f.write("captured_start_poses:\n")
            f.write(f"{line}\n")

    def _sanitize_name(self, raw_name: str) -> str:
        lowered = raw_name.strip().lower().replace(" ", "_")
        sanitized = re.sub(r"[^a-z0-9_-]+", "_", lowered)
        sanitized = re.sub(r"_+", "_", sanitized).strip("_")
        return sanitized

    def _pose_dict(self) -> Optional[Dict[str, float]]:
        if self.latest_pose is None:
            return None
        x, y, z, yaw = self.latest_pose
        return {"x": x, "y": y, "z": z, "yaw": yaw}

    def _read_named_catalog(self) -> Dict[str, Dict[str, float]]:
        if not os.path.exists(self.named_output_path):
            return {}
        with open(self.named_output_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        poses = data.get("captured_named_poses", {})
        if not isinstance(poses, dict):
            return {}
        return poses

    def _write_named_pose(self, name: str, pose: Dict[str, float]):
        os.makedirs(os.path.dirname(self.named_output_path), exist_ok=True)
        poses = self._read_named_catalog()
        poses[name] = pose
        payload = {"captured_named_poses": poses}
        with open(self.named_output_path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=True, default_flow_style=False)

    def _prompt_pose_name(self, old_attrs) -> Optional[str]:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        try:
            sys.stdout.write("\nPose name: ")
            sys.stdout.flush()
            raw_name = sys.stdin.readline()
        finally:
            tty.setcbreak(sys.stdin.fileno())
        if not raw_name:
            return None
        return self._sanitize_name(raw_name)

    def spin(self):
        rospy.loginfo("Pose capture ready")
        rospy.loginfo(
            "Press 'p' to print current pose, 'a' to append anonymous pose, "
            "'n' to save a named pose, 'q' to quit"
        )
        old_attrs = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if readable:
                    key = sys.stdin.read(1)
                    if key == "q":
                        return

                    pose_line = self._format_pose()
                    if pose_line is None:
                        rospy.logwarn("No Triton pose received yet")
                        continue

                    if key == "p":
                        print(pose_line)
                        sys.stdout.flush()
                    elif key == "a":
                        self._append_pose(pose_line)
                        rospy.loginfo("Saved pose to %s", self.output_path)
                    elif key == "n":
                        pose = self._pose_dict()
                        if pose is None:
                            rospy.logwarn("No Triton pose received yet")
                            continue
                        pose_name = self._prompt_pose_name(old_attrs)
                        if not pose_name:
                            rospy.logwarn("Skipped named save: empty pose name")
                            continue
                        self._write_named_pose(pose_name, pose)
                        rospy.loginfo("Saved named pose '%s' to %s", pose_name, self.named_output_path)
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


def main():
    rospy.init_node("pose_capture", anonymous=False)
    PoseCapture().spin()


if __name__ == "__main__":
    main()
