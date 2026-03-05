#!/usr/bin/env python3
"""Manual Q-table policy node for Deliverable 1 straight wall following."""

import math
import os
import sys
from typing import Dict

import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Support running from source tree and from catkin-installed script wrappers.
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from state_encoder import EncodedState, StateEncoder


class WallFollowingPolicyNode:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        qtable_path = rospy.get_param("~qtable_path")
        actions_path = rospy.get_param("~actions_path")

        self.collision_action = rospy.get_param("~collision_action", "turn_left_hard")
        self.default_action = rospy.get_param("~default_action", "straight")

        self.encoder = StateEncoder(
            front_too_close=float(rospy.get_param("~front_too_close", 0.55)),
            right_too_close=float(rospy.get_param("~right_too_close", 0.55)),
            right_too_far=float(rospy.get_param("~right_too_far", 0.95)),
            heading_parallel_tolerance=float(rospy.get_param("~heading_parallel_tolerance", 0.10)),
        )

        self.actions = self._load_actions(actions_path)
        self.q_table = self._load_q_table(qtable_path)

        if self.default_action not in self.actions:
            self.default_action = next(iter(self.actions.keys()))

        if self.collision_action not in self.actions:
            rospy.logwarn(
                "collision_action '%s' not in actions config. Falling back to '%s'.",
                self.collision_action,
                self.default_action,
            )
            self.collision_action = self.default_action

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)

        rospy.loginfo("WallFollowingPolicyNode started.")
        rospy.loginfo("Listening on %s, publishing on %s", self.scan_topic, self.cmd_topic)
        rospy.loginfo("Loaded %d states from Q-table and %d actions", len(self.q_table), len(self.actions))

    def _load_actions(self, actions_path: str) -> Dict[str, Dict[str, float]]:
        with open(actions_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        raw_actions = data.get("actions", {})
        if not raw_actions:
            raise ValueError(f"No 'actions' section in {actions_path}")

        parsed = {}
        for action_name, cfg in raw_actions.items():
            linear_x = float(cfg.get("linear_x", 0.0))
            angular_z = float(cfg.get("angular_z", 0.0))
            parsed[str(action_name)] = {"linear_x": linear_x, "angular_z": angular_z}

        if "default_action" in data and isinstance(data["default_action"], str):
            self.default_action = data["default_action"]

        return parsed

    def _load_q_table(self, qtable_path: str) -> Dict[str, Dict[str, float]]:
        with open(qtable_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        raw_states = data.get("states", {})
        parsed = {}
        for state_key, action_scores in raw_states.items():
            if not isinstance(action_scores, dict):
                continue
            parsed_scores = {}
            for action_name, score in action_scores.items():
                parsed_scores[str(action_name)] = float(score)
            parsed[str(state_key)] = parsed_scores

        if not parsed:
            raise ValueError(f"No valid Q-table states found in {qtable_path}")

        return parsed

    def _scan_callback(self, msg: LaserScan):
        state = self.encoder.encode(msg)
        action_name = self._select_action(state)
        self._publish_action(action_name)

        rospy.loginfo_throttle(
            1.0,
            "state=%s front=%.2f right=%.2f (rf=%.2f rr=%.2f) action=%s",
            state.key,
            state.front_min,
            state.right_min,
            state.right_front,
            state.right_rear,
            action_name,
        )

    def _select_action(self, state: EncodedState) -> str:
        # Hard safety override: if front obstacle is too close, force a hard left turn.
        if state.front_bin == "too_close":
            return self.collision_action

        state_scores = self.q_table.get(state.key)
        if state_scores:
            best = max(state_scores.items(), key=lambda kv: (kv[1], kv[0]))
            if best[0] in self.actions:
                return best[0]

        # Structured fallback if a state is missing in qtable_d1.yaml
        if state.right_bin == "too_close":
            return "turn_left_soft" if "turn_left_soft" in self.actions else self.default_action
        if state.right_bin == "too_far":
            if state.heading_bin == "away_from_wall" and "turn_right_hard" in self.actions:
                return "turn_right_hard"
            return "turn_right_soft" if "turn_right_soft" in self.actions else self.default_action
        if state.heading_bin == "toward_wall" and "turn_left_soft" in self.actions:
            return "turn_left_soft"
        if state.heading_bin == "away_from_wall" and "turn_right_soft" in self.actions:
            return "turn_right_soft"
        return "straight" if "straight" in self.actions else self.default_action

    def _publish_action(self, action_name: str):
        action_cfg = self.actions.get(action_name, self.actions[self.default_action])

        cmd = Twist()
        cmd.linear.x = float(action_cfg["linear_x"])
        cmd.angular.z = float(action_cfg["angular_z"])

        if not (math.isfinite(cmd.linear.x) and math.isfinite(cmd.angular.z)):
            rospy.logwarn("Non-finite command detected, publishing stop command instead.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rospy.init_node("wf_policy_node", anonymous=False)
    try:
        WallFollowingPolicyNode()
        rospy.spin()
    except Exception as exc:  # pragma: no cover
        rospy.logfatal("wf_policy_node failed to start: %s", exc)
        raise


if __name__ == "__main__":
    main()
