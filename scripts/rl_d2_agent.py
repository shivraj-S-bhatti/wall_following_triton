#!/usr/bin/env python3
"""Deliverable 2 RL agent with D2-specific state, evaluation, and test overrides."""

import csv
import math
import os
import random
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

import rospy
import yaml
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from state_encoder import D2EncodedState, D2StateEncoder


FRONT_BINS = ["blocked", "clear"]
RIGHT_BINS = ["too_close", "good", "too_far"]
HEADING_BINS = ["toward_wall", "parallel", "away_from_wall"]
OPEN_BINS = ["closed", "open"]


@dataclass
class EpisodeStats:
    episode_idx: int
    total_reward: float
    steps: int
    collisions: int
    traps: int


@dataclass
class EvaluationStats:
    completed_episodes: int
    score: float
    successes: int
    collisions: int
    traps: int
    failures: int


class RLWallFollowerD2:
    def __init__(self):
        self.algorithm = str(rospy.get_param("~algorithm", "q_learning")).strip().lower()
        self.mode = str(rospy.get_param("~mode", "train")).strip().lower()
        if self.algorithm not in {"q_learning", "sarsa"}:
            raise ValueError("~algorithm must be one of: q_learning, sarsa")
        if self.mode not in {"train", "test"}:
            raise ValueError("~mode must be one of: train, test")

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        self.alpha = float(rospy.get_param("~alpha", 0.15))
        self.gamma = float(rospy.get_param("~gamma", 0.95))
        self.epsilon = float(rospy.get_param("~epsilon", 0.20))
        self.epsilon_min = float(rospy.get_param("~epsilon_min", 0.03))
        self.epsilon_decay = float(rospy.get_param("~epsilon_decay", 0.994))

        self.control_hz = float(rospy.get_param("~control_hz", 6.0))
        self.max_episodes = int(rospy.get_param("~max_episodes", 420))
        self.max_steps_per_episode = int(rospy.get_param("~max_steps_per_episode", 240))
        self.training_done_stop = bool(rospy.get_param("~training_done_stop", True))
        self.checkpoint_every_episodes = max(0, int(rospy.get_param("~checkpoint_every_episodes", 1)))
        self.resume_training_state = bool(rospy.get_param("~resume_training_state", True))
        self.reset_settle_seconds = float(rospy.get_param("~reset_settle_seconds", 0.20))
        self.reset_retry_attempts = max(1, int(rospy.get_param("~reset_retry_attempts", 3)))

        self.eval_every_episodes = max(0, int(rospy.get_param("~eval_every_episodes", 20)))
        self.eval_max_steps_per_start = max(1, int(rospy.get_param("~eval_max_steps_per_start", 100)))
        self.eval_failure_penalty = float(rospy.get_param("~eval_failure_penalty", -6.0))

        self.reward_front_clear = float(rospy.get_param("~reward_front_clear", 0.30))
        self.reward_front_blocked = float(rospy.get_param("~reward_front_blocked", -1.40))
        self.reward_right_good = float(rospy.get_param("~reward_right_good", 1.20))
        self.reward_right_too_close = float(rospy.get_param("~reward_right_too_close", -1.00))
        self.reward_right_too_far = float(rospy.get_param("~reward_right_too_far", -0.80))
        self.reward_heading_parallel = float(rospy.get_param("~reward_heading_parallel", 0.35))
        self.reward_heading_off = float(rospy.get_param("~reward_heading_off", -0.10))
        self.reward_progress_scale = float(rospy.get_param("~reward_progress_scale", 0.20))
        self.reward_too_far_preferred_turn = float(
            rospy.get_param("~reward_too_far_preferred_turn", 0.35)
        )
        self.reward_too_far_wrong_turn = float(rospy.get_param("~reward_too_far_wrong_turn", -0.25))
        self.reward_too_far_straight_toward = float(
            rospy.get_param("~reward_too_far_straight_toward", 0.20)
        )
        self.reward_too_far_straight_other = float(
            rospy.get_param("~reward_too_far_straight_other", -0.10)
        )
        self.reward_blocked_preferred_turn = float(
            rospy.get_param("~reward_blocked_preferred_turn", 0.90)
        )
        self.reward_blocked_wrong_turn = float(rospy.get_param("~reward_blocked_wrong_turn", -1.10))
        self.reward_blocked_straight = float(rospy.get_param("~reward_blocked_straight", -1.40))

        self.collision_distance = float(rospy.get_param("~collision_distance", 0.30))
        self.collision_penalty = float(rospy.get_param("~collision_penalty", -8.00))
        self.trapped_position_epsilon = float(rospy.get_param("~trapped_position_epsilon", 0.008))
        self.trapped_yaw_epsilon = float(rospy.get_param("~trapped_yaw_epsilon", 0.08))
        self.trapped_steps_threshold = int(rospy.get_param("~trapped_steps_threshold", 12))
        self.trapped_penalty = float(rospy.get_param("~trapped_penalty", -4.00))
        self.tilt_termination_deg = float(rospy.get_param("~tilt_termination_deg", 55.0))
        self.z_termination_height = float(rospy.get_param("~z_termination_height", 0.25))

        self.acquire_wall_enabled = bool(rospy.get_param("~acquire_wall_enabled", False))
        self.acquire_wall_distance = float(rospy.get_param("~acquire_wall_distance", 0.95))
        self.acquire_front_turn_distance = float(rospy.get_param("~acquire_front_turn_distance", 0.85))

        self.test_start_enabled = bool(rospy.get_param("~test_start_enabled", False))
        self.test_start_pose = {
            "x": float(rospy.get_param("~test_start_x", 0.0)),
            "y": float(rospy.get_param("~test_start_y", 0.0)),
            "z": float(rospy.get_param("~test_start_z", 0.0)),
            "yaw": float(rospy.get_param("~test_start_yaw", 0.0)),
        }

        self.actions_path = rospy.get_param("~actions_path")
        self.qtable_input_path = rospy.get_param("~qtable_input_path")
        self.qtable_output_path = rospy.get_param("~qtable_output_path")
        latest_qtable_output_path = str(rospy.get_param("~latest_qtable_output_path", "")).strip()
        self.latest_qtable_output_path = latest_qtable_output_path or self._derive_latest_qtable_path(
            self.qtable_output_path
        )
        self.metrics_csv_path = rospy.get_param("~metrics_csv_path")
        self.eval_csv_path = str(rospy.get_param("~eval_csv_path", "")).strip() or self._derive_eval_csv_path(
            self.metrics_csv_path
        )

        self.random_seed = int(rospy.get_param("~random_seed", 603))
        random.seed(self.random_seed)

        self.start_poses = rospy.get_param("~start_poses", [])
        self.eval_start_poses = rospy.get_param("~eval_start_poses", [])
        if not isinstance(self.start_poses, list):
            raise ValueError("~start_poses must be a list")
        if not isinstance(self.eval_start_poses, list):
            raise ValueError("~eval_start_poses must be a list")
        self.robot_model_name = rospy.get_param("~robot_model_name", "triton")

        self.encoder = D2StateEncoder(
            front_blocked_threshold=float(rospy.get_param("~front_blocked_threshold", 0.55)),
            right_too_close=float(rospy.get_param("~right_too_close", 0.55)),
            right_too_far=float(rospy.get_param("~right_too_far", 0.95)),
            heading_parallel_tolerance=float(rospy.get_param("~heading_parallel_tolerance", 0.10)),
            opening_distance_threshold=float(rospy.get_param("~opening_distance_threshold", 1.20)),
            front_sector_start_deg=float(rospy.get_param("~front_sector_start_deg", 75.0)),
            front_sector_end_deg=float(rospy.get_param("~front_sector_end_deg", 105.0)),
            right_front_sector_start_deg=float(rospy.get_param("~right_front_sector_start_deg", 15.0)),
            right_front_sector_end_deg=float(rospy.get_param("~right_front_sector_end_deg", 65.0)),
            right_rear_sector_start_deg=float(rospy.get_param("~right_rear_sector_start_deg", -70.0)),
            right_rear_sector_end_deg=float(rospy.get_param("~right_rear_sector_end_deg", -20.0)),
            front_left_open_sector_start_deg=float(
                rospy.get_param("~front_left_open_sector_start_deg", 100.0)
            ),
            front_left_open_sector_end_deg=float(
                rospy.get_param("~front_left_open_sector_end_deg", 155.0)
            ),
            front_right_open_sector_start_deg=float(
                rospy.get_param("~front_right_open_sector_start_deg", 25.0)
            ),
            front_right_open_sector_end_deg=float(
                rospy.get_param("~front_right_open_sector_end_deg", 85.0)
            ),
        )

        self.actions = self._load_actions(self.actions_path)
        self.loaded_qtable_metadata: Dict[str, object] = {}
        self.q_table = self._load_q_table(self.qtable_input_path)
        self._ensure_dense_q_table()

        self.run_id = self._resolve_run_id()
        self.run_started_at_utc = self._resolve_run_started_at()
        self.training_finished_at_utc: Optional[str] = None
        configured_run_dir = str(rospy.get_param("~run_artifact_dir", "")).strip()
        self.run_artifact_dir = configured_run_dir or self._default_run_artifact_dir()
        self.run_qtable_output_path = os.path.join(
            self.run_artifact_dir, os.path.basename(self.qtable_output_path)
        )
        self.run_latest_qtable_output_path = os.path.join(
            self.run_artifact_dir, os.path.basename(self.latest_qtable_output_path)
        )
        self.run_metrics_csv_path = os.path.join(
            self.run_artifact_dir, os.path.basename(self.metrics_csv_path)
        )
        self.run_eval_csv_path = os.path.join(
            self.run_artifact_dir, os.path.basename(self.eval_csv_path)
        )
        self._prime_run_artifacts()

        self.latest_scan: Optional[LaserScan] = None
        self.latest_robot_pose: Optional[Tuple[float, float, float, float, float, float]] = None
        self.prev_robot_pose: Optional[Tuple[float, float, float, float, float, float]] = None
        self.stationary_steps = 0

        self.prev_state_key: Optional[str] = None
        self.prev_action: Optional[str] = None

        self.episode_idx = 0
        self.episode_step = 0
        self.episode_reward = 0.0
        self.episode_collisions = 0
        self.episode_traps = 0
        self.best_eval_score = float("-inf")
        self.episode_history: List[EpisodeStats] = []
        self.evaluation_history: List[EvaluationStats] = []
        self._restore_training_state_if_requested()

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1
        )

        self.set_model_state_srv = None
        self.pause_physics_srv = None
        self.unpause_physics_srv = None
        self.reset_world_srv = None
        self._configure_gazebo_services()

        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.control_hz, 1.0)), self._on_control_tick
        )

        rospy.loginfo("RLWallFollowerD2 started")
        rospy.loginfo("algorithm=%s mode=%s", self.algorithm, self.mode)
        rospy.loginfo("alpha=%.3f gamma=%.3f epsilon=%.3f", self.alpha, self.gamma, self.epsilon)
        rospy.loginfo("qtable_input=%s", self.qtable_input_path)
        rospy.loginfo("qtable_output=%s", self.qtable_output_path)
        rospy.loginfo(
            "latest_qtable_output=%s eval_csv=%s",
            self.latest_qtable_output_path,
            self.eval_csv_path,
        )
        rospy.loginfo("run_id=%s run_artifact_dir=%s", self.run_id, self.run_artifact_dir)

        if self.mode == "train":
            self._reset_for_next_episode(reason="initial_start")
        elif self.test_start_enabled:
            self._apply_test_start_pose()

    def _configure_gazebo_services(self):
        needs_services = self.mode == "train" or self.test_start_enabled
        if not needs_services:
            return

        rospy.wait_for_service("/gazebo/set_model_state", timeout=20.0)
        rospy.wait_for_service("/gazebo/pause_physics", timeout=20.0)
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=20.0)
        rospy.wait_for_service("/gazebo/reset_world", timeout=20.0)

        self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.pause_physics_srv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_physics_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)

    def _apply_test_start_pose(self):
        try:
            self._reset_robot_pose_safely(self.test_start_pose)
            rospy.loginfo(
                "Applied manual test start pose x=%.3f y=%.3f yaw=%.3f",
                self.test_start_pose["x"],
                self.test_start_pose["y"],
                self.test_start_pose["yaw"],
            )
        except Exception as exc:  # pragma: no cover - ROS runtime only
            rospy.logwarn("Failed to apply manual test start pose: %s", exc)

    def _load_actions(self, path: str) -> Dict[str, Dict[str, float]]:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        raw_actions = data.get("actions", {})
        if not raw_actions:
            raise ValueError(f"No actions found in {path}")

        parsed: Dict[str, Dict[str, float]] = {}
        for name, cfg in raw_actions.items():
            parsed[str(name)] = {
                "linear_x": float(cfg.get("linear_x", 0.0)),
                "linear_y": float(cfg.get("linear_y", 0.0)),
                "angular_z": float(cfg.get("angular_z", 0.0)),
            }
        return parsed

    def _load_q_table(self, path: str) -> Dict[str, Dict[str, float]]:
        if not os.path.exists(path):
            rospy.logwarn("Q-table file %s not found. Starting from zeros.", path)
            self.loaded_qtable_metadata = {}
            return {}

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        metadata = data.get("metadata", {})
        self.loaded_qtable_metadata = metadata if isinstance(metadata, dict) else {}

        raw_states = data.get("states", {})
        q_table: Dict[str, Dict[str, float]] = {}
        for state_key, values in raw_states.items():
            if not isinstance(values, dict):
                continue
            q_table[str(state_key)] = {str(a): float(v) for a, v in values.items()}

        if not q_table:
            return {}

        sample_key = next(iter(q_table.keys()))
        if len(sample_key.split("|")) == 3:
            rospy.loginfo("Expanding legacy 18-state Q-table into D2 72-state space")
            return self._expand_legacy_q_table(q_table)
        return q_table

    def _expand_legacy_q_table(
        self, legacy_q_table: Dict[str, Dict[str, float]]
    ) -> Dict[str, Dict[str, float]]:
        expanded: Dict[str, Dict[str, float]] = {}
        for state_key, action_values in legacy_q_table.items():
            parts = state_key.split("|")
            if len(parts) != 3:
                continue
            for front_left_open in OPEN_BINS:
                for front_right_open in OPEN_BINS:
                    expanded_key = (
                        f"{parts[0]}|{parts[1]}|{parts[2]}|"
                        f"{front_left_open}|{front_right_open}"
                    )
                    expanded[expanded_key] = dict(action_values)
        return expanded

    def _ensure_dense_q_table(self):
        for state_key in self._all_state_keys():
            self.q_table.setdefault(state_key, {})
            for action_name in self.actions.keys():
                self.q_table[state_key].setdefault(action_name, 0.0)

    @staticmethod
    def _derive_latest_qtable_path(best_path: str) -> str:
        root, ext = os.path.splitext(best_path)
        if ext:
            return f"{root}_latest{ext}"
        return f"{best_path}_latest"

    @staticmethod
    def _derive_eval_csv_path(metrics_csv_path: str) -> str:
        if metrics_csv_path.endswith("_metrics.csv"):
            return metrics_csv_path.replace("_metrics.csv", "_eval.csv")
        root, ext = os.path.splitext(metrics_csv_path)
        return f"{root}_eval{ext or '.csv'}"

    @staticmethod
    def _utc_now() -> datetime:
        return datetime.now(timezone.utc)

    @classmethod
    def _utc_stamp(cls) -> str:
        return cls._utc_now().strftime("%Y%m%d_%H%M%S")

    @classmethod
    def _utc_iso(cls) -> str:
        return cls._utc_now().replace(microsecond=0).isoformat().replace("+00:00", "Z")

    def _resolve_run_id(self) -> str:
        run_id = str(self.loaded_qtable_metadata.get("run_id", "")).strip()
        return run_id or self._utc_stamp()

    def _resolve_run_started_at(self) -> str:
        started = str(self.loaded_qtable_metadata.get("run_started_at_utc", "")).strip()
        return started or self._utc_iso()

    def _default_run_artifact_dir(self) -> str:
        base_artifact_dir = os.path.dirname(self.metrics_csv_path) or "."
        return os.path.join(base_artifact_dir, "runs", self.algorithm, self.run_id)

    def _prime_run_artifacts(self):
        os.makedirs(self.run_artifact_dir, exist_ok=True)

        self._copy_if_missing(self.metrics_csv_path, self.run_metrics_csv_path)
        self._copy_if_missing(self.eval_csv_path, self.run_eval_csv_path)
        self._copy_if_missing(self.qtable_output_path, self.run_qtable_output_path)
        self._copy_if_missing(self.latest_qtable_output_path, self.run_latest_qtable_output_path)

    @staticmethod
    def _copy_if_missing(src: str, dest: str):
        if os.path.exists(src) and not os.path.exists(dest):
            shutil.copyfile(src, dest)

    def _restore_training_state_if_requested(self):
        if self.mode != "train" or not self.resume_training_state:
            return
        if "_latest" not in os.path.basename(self.qtable_input_path):
            return

        resumed_episode_idx = 0
        if os.path.exists(self.metrics_csv_path):
            try:
                with open(self.metrics_csv_path, "r", encoding="utf-8", newline="") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            resumed_episode_idx = max(resumed_episode_idx, int(row["episode"]) + 1)
                        except (KeyError, TypeError, ValueError):
                            continue
            except OSError as exc:
                rospy.logwarn("Failed to inspect metrics CSV for resume state: %s", exc)

        metadata_completed = self.loaded_qtable_metadata.get("completed_episodes")
        try:
            if metadata_completed is not None:
                resumed_episode_idx = max(resumed_episode_idx, int(metadata_completed))
        except (TypeError, ValueError):
            pass

        metadata_epsilon = self.loaded_qtable_metadata.get("epsilon")
        try:
            if metadata_epsilon is not None:
                self.epsilon = max(self.epsilon_min, float(metadata_epsilon))
        except (TypeError, ValueError):
            pass

        if os.path.exists(self.eval_csv_path):
            try:
                with open(self.eval_csv_path, "r", encoding="utf-8", newline="") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            self.best_eval_score = max(self.best_eval_score, float(row["score"]))
                        except (KeyError, TypeError, ValueError):
                            continue
            except OSError as exc:
                rospy.logwarn("Failed to inspect eval CSV for resume state: %s", exc)

        metadata_best_eval = self.loaded_qtable_metadata.get("best_eval_score")
        try:
            if metadata_best_eval is not None:
                self.best_eval_score = max(self.best_eval_score, float(metadata_best_eval))
        except (TypeError, ValueError):
            pass

        self.episode_idx = resumed_episode_idx
        rospy.loginfo(
            "Resuming training state: episode_idx=%d epsilon=%.3f best_eval=%.3f",
            self.episode_idx,
            self.epsilon,
            self.best_eval_score,
        )

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _model_states_callback(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        position = pose.position
        orientation = pose.orientation
        qx = float(orientation.x)
        qy = float(orientation.y)
        qz = float(orientation.z)
        qw = float(orientation.w)

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self.latest_robot_pose = (float(position.x), float(position.y), float(position.z), roll, pitch, yaw)

    def _on_control_tick(self, _event):
        if self.latest_scan is None:
            return

        state = self.encoder.encode(self.latest_scan)

        if self.mode == "test":
            action = self._select_test_action(state)
            self._publish_action(action)
            rospy.loginfo_throttle(
                1.0,
                "[test] state=%s action=%s front=%.2f right=%.2f",
                state.key,
                action,
                state.front_min,
                state.right_min,
            )
            return

        self._training_step(state)

    def _training_step(self, current_state: D2EncodedState):
        if self.episode_idx >= self.max_episodes:
            if self.training_done_stop:
                self._publish_stop()
            return

        current_key = current_state.key

        if self.prev_state_key is None or self.prev_action is None:
            initial_action = self._select_behavior_action(current_key)
            self._publish_action(initial_action)
            self.prev_state_key = current_key
            self.prev_action = initial_action
            self.episode_step = 1
            return

        reward = self._compute_reward(current_state, self.prev_action)
        termination_reason = self._termination_reason(current_state)
        done = termination_reason is not None
        previous_action = self.prev_action

        if termination_reason in {"collision", "flipped"}:
            reward += self.collision_penalty
            self.episode_collisions += 1
        elif termination_reason == "trapped":
            reward += self.trapped_penalty
            self.episode_traps += 1

        self.episode_reward += reward

        next_action = self._select_behavior_action(current_key)
        old_q = self.q_table[self.prev_state_key][self.prev_action]
        if done:
            bootstrap = 0.0
        elif self.algorithm == "sarsa":
            bootstrap = self.q_table[current_key][next_action]
        else:
            bootstrap = max(self.q_table[current_key].values())

        target = reward + self.gamma * bootstrap
        self.q_table[self.prev_state_key][self.prev_action] = old_q + self.alpha * (target - old_q)

        if done or self.episode_step >= self.max_steps_per_episode:
            self._finish_episode(reason=termination_reason or "max_steps")
            return

        self._publish_action(next_action)
        self.prev_state_key = current_key
        self.prev_action = next_action
        self.episode_step += 1

        rospy.loginfo_throttle(
            1.0,
            "[train] ep=%d step=%d state=%s reward_from=%s reward=%.3f next_action=%s eps=%.3f",
            self.episode_idx,
            self.episode_step,
            current_key,
            previous_action,
            reward,
            next_action,
            self.epsilon,
        )

    def _select_behavior_action(self, state_key: str) -> str:
        if self.mode == "train" and random.random() < self.epsilon:
            return random.choice(list(self.actions.keys()))
        return self._select_greedy_action(state_key)

    def _select_greedy_action(self, state_key: str) -> str:
        state_scores = self.q_table[state_key]
        return max(state_scores.items(), key=lambda kv: (kv[1], kv[0]))[0]

    def _select_test_action(self, state: D2EncodedState) -> str:
        acquisition_action = self._select_test_acquisition_action(state)
        if acquisition_action is not None:
            return acquisition_action
        return self._select_greedy_action(state.key)

    def _select_test_acquisition_action(self, state: D2EncodedState) -> Optional[str]:
        if not self.acquire_wall_enabled:
            return None
        if state.right_bin != "too_far":
            return None
        if state.front_bin == "clear" and "straight" in self.actions:
            return "straight"
        if state.front_right_open_bin == "open" and "turn_right_hard" in self.actions:
            return "turn_right_hard"
        if state.front_left_open_bin == "open" and "turn_left_hard" in self.actions:
            return "turn_left_hard"
        if "turn_right_hard" in self.actions:
            return "turn_right_hard"
        return "straight" if "straight" in self.actions else None

    def _compute_reward(self, state: D2EncodedState, action_name: str) -> float:
        reward = 0.0
        turn_direction = self._action_turn_direction(action_name)

        reward += self.reward_front_blocked if state.front_bin == "blocked" else self.reward_front_clear

        if state.right_bin == "good":
            reward += self.reward_right_good
        elif state.right_bin == "too_close":
            reward += self.reward_right_too_close
        else:
            reward += self.reward_right_too_far

        if state.heading_bin == "parallel":
            reward += self.reward_heading_parallel
        else:
            reward += self.reward_heading_off

        reward += self.reward_progress_scale * self._action_progress_speed(action_name)

        if state.front_bin == "clear" and state.right_bin == "too_far":
            if turn_direction == "right":
                reward += self.reward_too_far_preferred_turn
            elif turn_direction == "left":
                reward += self.reward_too_far_wrong_turn
            elif state.heading_bin == "toward_wall":
                reward += self.reward_too_far_straight_toward
            else:
                reward += self.reward_too_far_straight_other

        if state.front_bin == "blocked":
            preferred_turn = self._preferred_blocked_turn(state)
            if turn_direction == preferred_turn:
                reward += self.reward_blocked_preferred_turn
            elif turn_direction == "straight":
                reward += self.reward_blocked_straight
            else:
                reward += self.reward_blocked_wrong_turn

        return reward

    def _preferred_blocked_turn(self, state: D2EncodedState) -> str:
        # For right-wall following, a blocked front should usually turn left
        # unless the right wall has actually disappeared and a right opening is
        # the clear reacquisition path.
        if state.front_right_open_bin == "open" and state.right_bin == "too_far":
            return "right"
        return "left"

    def _action_progress_speed(self, action_name: str) -> float:
        cfg = self.actions[action_name]
        return math.hypot(float(cfg["linear_x"]), float(cfg["linear_y"]))

    def _action_turn_direction(self, action_name: str) -> str:
        angular_z = float(self.actions[action_name]["angular_z"])
        if angular_z > 1e-6:
            return "left"
        if angular_z < -1e-6:
            return "right"
        return "straight"

    def _termination_reason(self, state: D2EncodedState) -> Optional[str]:
        if state.front_min < self.collision_distance:
            return "collision"
        if self._is_flipped():
            return "flipped"
        if self._is_trapped():
            return "trapped"
        return None

    def _is_trapped(self) -> bool:
        if self.latest_robot_pose is None:
            return False
        if self.prev_robot_pose is None:
            self.prev_robot_pose = self.latest_robot_pose
            self.stationary_steps = 0
            return False

        dx = self.latest_robot_pose[0] - self.prev_robot_pose[0]
        dy = self.latest_robot_pose[1] - self.prev_robot_pose[1]
        distance = math.hypot(dx, dy)
        yaw_delta = abs(
            math.atan2(
                math.sin(self.latest_robot_pose[5] - self.prev_robot_pose[5]),
                math.cos(self.latest_robot_pose[5] - self.prev_robot_pose[5]),
            )
        )
        self.prev_robot_pose = self.latest_robot_pose

        if distance <= self.trapped_position_epsilon and yaw_delta <= self.trapped_yaw_epsilon:
            self.stationary_steps += 1
        else:
            self.stationary_steps = 0

        return self.stationary_steps >= self.trapped_steps_threshold

    def _is_flipped(self) -> bool:
        if self.latest_robot_pose is None:
            return False

        z = self.latest_robot_pose[2]
        roll = abs(self.latest_robot_pose[3])
        pitch = abs(self.latest_robot_pose[4])
        tilt_limit_rad = math.radians(self.tilt_termination_deg)

        if z > self.z_termination_height:
            return True
        if roll >= tilt_limit_rad or pitch >= tilt_limit_rad:
            return True
        return False

    def _finish_episode(self, reason: str):
        stats = EpisodeStats(
            episode_idx=self.episode_idx,
            total_reward=self.episode_reward,
            steps=self.episode_step,
            collisions=self.episode_collisions,
            traps=self.episode_traps,
        )
        self.episode_history.append(stats)

        rospy.loginfo(
            "Episode %d finished (%s): reward=%.3f steps=%d collisions=%d traps=%d",
            stats.episode_idx,
            reason,
            stats.total_reward,
            stats.steps,
            stats.collisions,
            stats.traps,
        )

        completed_episodes = self.episode_idx + 1
        self._append_metrics_row(stats)

        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        self._save_latest_checkpoint(completed_episodes)

        ran_eval = False
        if self.eval_every_episodes > 0 and completed_episodes % self.eval_every_episodes == 0:
            self._run_evaluation(completed_episodes)
            ran_eval = True

        self.episode_idx = completed_episodes

        if self.episode_idx >= self.max_episodes:
            if not ran_eval:
                self._run_evaluation(completed_episodes)
            self.training_finished_at_utc = self._utc_iso()
            self._save_q_table(
                self.latest_qtable_output_path,
                checkpoint_kind="latest",
                completed_episodes=self.episode_idx,
            )
            if self.run_latest_qtable_output_path != self.latest_qtable_output_path:
                self._save_q_table(
                    self.run_latest_qtable_output_path,
                    checkpoint_kind="latest",
                    completed_episodes=self.episode_idx,
                )
            rospy.loginfo("Training finished at episode %d", self.episode_idx)
            if self.training_done_stop:
                self._publish_stop()
            return

        self._reset_for_next_episode(reason=reason)

    def _run_evaluation(self, completed_episodes: int):
        if self.mode != "train" or not self.eval_start_poses:
            return

        rospy.loginfo("Running greedy evaluation at episode %d", completed_episodes)
        self._publish_stop()

        total_score = 0.0
        successes = 0
        collisions = 0
        traps = 0
        failures = 0

        for pose_cfg in self.eval_start_poses:
            score, success, collision_count, trap_count = self._evaluate_single_start(pose_cfg)
            total_score += score
            successes += int(success)
            collisions += collision_count
            traps += trap_count
            failures += int(not success)

        stats = EvaluationStats(
            completed_episodes=completed_episodes,
            score=total_score,
            successes=successes,
            collisions=collisions,
            traps=traps,
            failures=failures,
        )
        self.evaluation_history.append(stats)
        self._append_eval_row(stats)

        rospy.loginfo(
            "Evaluation after episode %d: score=%.3f successes=%d/%d collisions=%d traps=%d",
            completed_episodes,
            total_score,
            successes,
            len(self.eval_start_poses),
            collisions,
            traps,
        )

        if total_score > self.best_eval_score:
            self.best_eval_score = total_score
            self._save_q_table(
                self.qtable_output_path,
                checkpoint_kind="best",
                completed_episodes=completed_episodes,
            )
            if self.run_qtable_output_path != self.qtable_output_path:
                self._save_q_table(
                    self.run_qtable_output_path,
                    checkpoint_kind="best",
                    completed_episodes=completed_episodes,
                )
            rospy.loginfo("New best policy saved from evaluation (score=%.3f)", self.best_eval_score)

    def _evaluate_single_start(self, pose_cfg: dict) -> Tuple[float, bool, int, int]:
        self.prev_robot_pose = None
        self.stationary_steps = 0
        self._reset_robot_pose_safely(pose_cfg)
        self._publish_stop()

        score = 0.0
        prev_action: Optional[str] = None
        collisions = 0
        traps = 0
        success = True

        for _ in range(self.eval_max_steps_per_start):
            state = self._wait_for_encoded_state(timeout=1.5)
            if state is None:
                score += self.eval_failure_penalty
                success = False
                break

            current_key = state.key
            if prev_action is None:
                action = self._select_greedy_action(current_key)
                self._publish_action(action)
                prev_action = action
                rospy.sleep(1.0 / max(self.control_hz, 1.0))
                continue

            reward = self._compute_reward(state, prev_action)
            termination_reason = self._termination_reason(state)
            if termination_reason in {"collision", "flipped"}:
                reward += self.collision_penalty
                collisions += 1
            elif termination_reason == "trapped":
                reward += self.trapped_penalty
                traps += 1

            score += reward

            if termination_reason is not None:
                score += self.eval_failure_penalty
                success = False
                break

            action = self._select_greedy_action(current_key)
            self._publish_action(action)
            prev_action = action
            rospy.sleep(1.0 / max(self.control_hz, 1.0))

        self._publish_stop()
        return score, success, collisions, traps

    def _wait_for_encoded_state(self, timeout: float) -> Optional[D2EncodedState]:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown():
            if self.latest_scan is not None:
                return self.encoder.encode(self.latest_scan)
            if rospy.Time.now() >= deadline:
                break
            rospy.sleep(0.02)
        return None

    def _reset_for_next_episode(self, reason: str):
        self.prev_state_key = None
        self.prev_action = None
        self.episode_step = 0
        self.episode_reward = 0.0
        self.episode_collisions = 0
        self.episode_traps = 0
        self.latest_scan = None
        self.prev_robot_pose = None
        self.stationary_steps = 0
        self._publish_stop()

        if self.mode != "train" or self.set_model_state_srv is None or not self.start_poses:
            return

        try:
            pose_pool = list(self.start_poses)
            random.shuffle(pose_pool)
            pose_pool = pose_pool[: self.reset_retry_attempts]
            reset_ok = False
            for pose_cfg in pose_pool:
                self._reset_robot_pose_safely(pose_cfg)
                if not self._is_flipped():
                    reset_ok = True
                    break
                rospy.logwarn("Reset pose rejected due to 3D instability: %s", pose_cfg)

            if not reset_ok:
                rospy.logwarn("All reset attempts failed after '%s'; keeping last pose.", reason)
        except Exception as exc:  # pragma: no cover - ROS runtime only
            rospy.logwarn("Episode reset pose failed after '%s': %s", reason, exc)

    def _reset_robot_pose_safely(self, pose_cfg: dict):
        self.latest_scan = None
        self.prev_robot_pose = None
        self.stationary_steps = 0
        for _ in range(3):
            self._publish_stop()

        if self.pause_physics_srv is not None:
            self.pause_physics_srv()
        if self.reset_world_srv is not None:
            self.reset_world_srv()

        self._set_robot_pose(pose_cfg)

        for _ in range(3):
            self._publish_stop()

        if self.unpause_physics_srv is not None:
            self.unpause_physics_srv()
        if self.reset_settle_seconds > 0.0:
            rospy.sleep(self.reset_settle_seconds)
        self._publish_stop()

    def _set_robot_pose(self, pose_cfg: dict):
        x = float(pose_cfg.get("x", 0.0))
        y = float(pose_cfg.get("y", 0.0))
        z = float(pose_cfg.get("z", 0.0))
        yaw = float(pose_cfg.get("yaw", 0.0))

        state = ModelState()
        state.model_name = self.robot_model_name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        if self.set_model_state_srv is None:
            raise RuntimeError("set_model_state service is not available")
        self.set_model_state_srv(state)

    def _save_q_table(self, path: str, checkpoint_kind: str, completed_episodes: Optional[int] = None):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        saved_at = self._utc_iso()
        payload = {
            "metadata": {
                "algorithm": self.algorithm,
                "mode": self.mode,
                "alpha": self.alpha,
                "gamma": self.gamma,
                "epsilon": self.epsilon,
                "random_seed": self.random_seed,
                "completed_episodes": int(
                    self.episode_idx if completed_episodes is None else completed_episodes
                ),
                "run_id": self.run_id,
                "run_started_at_utc": self.run_started_at_utc,
                "last_saved_at_utc": saved_at,
                "run_finished_at_utc": self.training_finished_at_utc,
                "source_qtable_path": self.qtable_input_path,
                "checkpoint_kind": checkpoint_kind,
                "best_eval_score": None if self.best_eval_score == float("-inf") else self.best_eval_score,
            },
            "states": self.q_table,
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, sort_keys=True)

    def _save_latest_checkpoint(self, completed_episodes: int):
        if self.checkpoint_every_episodes <= 0:
            return
        if completed_episodes % self.checkpoint_every_episodes != 0:
            return
        self._save_q_table(
            self.latest_qtable_output_path,
            checkpoint_kind="latest",
            completed_episodes=completed_episodes,
        )
        if self.run_latest_qtable_output_path != self.latest_qtable_output_path:
            self._save_q_table(
                self.run_latest_qtable_output_path,
                checkpoint_kind="latest",
                completed_episodes=completed_episodes,
            )

    def _append_metrics_row(self, stats: EpisodeStats):
        self._append_metrics_row_to_path(self.metrics_csv_path, stats)
        if self.run_metrics_csv_path != self.metrics_csv_path:
            self._append_metrics_row_to_path(self.run_metrics_csv_path, stats)

    def _append_metrics_row_to_path(self, path: str, stats: EpisodeStats):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        file_exists = os.path.exists(path)
        with open(path, "a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["episode", "reward", "steps", "collisions", "epsilon"])
            writer.writerow(
                [
                    stats.episode_idx,
                    f"{stats.total_reward:.6f}",
                    stats.steps,
                    stats.collisions,
                    f"{self.epsilon:.6f}",
                ]
            )

    def _append_eval_row(self, stats: EvaluationStats):
        self._append_eval_row_to_path(self.eval_csv_path, stats)
        if self.run_eval_csv_path != self.eval_csv_path:
            self._append_eval_row_to_path(self.run_eval_csv_path, stats)

    def _append_eval_row_to_path(self, path: str, stats: EvaluationStats):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        file_exists = os.path.exists(path)
        with open(path, "a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(
                    ["completed_episodes", "score", "successes", "failures", "collisions", "traps"]
                )
            writer.writerow(
                [
                    stats.completed_episodes,
                    f"{stats.score:.6f}",
                    stats.successes,
                    stats.failures,
                    stats.collisions,
                    stats.traps,
                ]
            )

    def _publish_action(self, action_name: str):
        cfg = self.actions[action_name]
        cmd = Twist()
        cmd.linear.x = float(cfg["linear_x"])
        cmd.linear.y = float(cfg["linear_y"])
        cmd.angular.z = float(cfg["angular_z"])
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _all_state_keys() -> List[str]:
        state_keys = []
        for front_bin in FRONT_BINS:
            for right_bin in RIGHT_BINS:
                for heading_bin in HEADING_BINS:
                    for front_left_open in OPEN_BINS:
                        for front_right_open in OPEN_BINS:
                            state_keys.append(
                                f"{front_bin}|{right_bin}|{heading_bin}|"
                                f"{front_left_open}|{front_right_open}"
                            )
        return state_keys


def main():
    rospy.init_node("rl_d2_agent", anonymous=False)
    try:
        RLWallFollowerD2()
        rospy.spin()
    except Exception as exc:  # pragma: no cover
        rospy.logfatal("rl_d2_agent failed to start: %s", exc)
        raise


if __name__ == "__main__":
    main()
