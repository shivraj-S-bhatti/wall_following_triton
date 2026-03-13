#!/usr/bin/env python3
"""Deliverable 2 reinforcement learning agent for Triton wall following.

This script is intentionally comment-heavy for learning purposes.
It supports both required algorithms:
- Q-learning (off-policy TD control)
- SARSA (on-policy TD control)

And both required modes:
- train: update Q-values online in Gazebo
- test: load a learned Q-table and run greedy policy only
"""

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

# Keep local imports working both in source tree and installed catkin wrappers.
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from state_encoder import EncodedState, StateEncoder


# The state space is fixed by our Deliverable 1 discretization.
FRONT_BINS = ["too_close", "safe"]
RIGHT_BINS = ["too_close", "good", "too_far"]
HEADING_BINS = ["toward_wall", "parallel", "away_from_wall"]


@dataclass
class EpisodeStats:
    episode_idx: int
    total_reward: float
    steps: int
    collisions: int
    traps: int


class RLWallFollowerD2:
    """Main reinforcement learning node.

    Core design choice for educational clarity:
    - run one control/update step at fixed timer frequency
    - keep the update logic explicit (not hidden in callbacks)
    - store every learned transition through a small set of clearly named fields
    """

    def __init__(self):
        # -----------------------------
        # 1) Read high-level run config
        # -----------------------------
        self.algorithm = str(rospy.get_param("~algorithm", "q_learning")).strip().lower()
        self.mode = str(rospy.get_param("~mode", "train")).strip().lower()

        if self.algorithm not in {"q_learning", "sarsa"}:
            raise ValueError("~algorithm must be one of: q_learning, sarsa")
        if self.mode not in {"train", "test"}:
            raise ValueError("~mode must be one of: train, test")

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.acquire_wall_enabled = bool(rospy.get_param("~acquire_wall_enabled", False))
        self.acquire_wall_distance = float(rospy.get_param("~acquire_wall_distance", 0.95))
        self.acquire_front_turn_distance = float(rospy.get_param("~acquire_front_turn_distance", 0.90))

        # -----------------------------
        # 2) RL hyperparameters
        # -----------------------------
        self.alpha = float(rospy.get_param("~alpha", 0.20))
        self.gamma = float(rospy.get_param("~gamma", 0.95))
        self.epsilon = float(rospy.get_param("~epsilon", 0.20))
        self.epsilon_min = float(rospy.get_param("~epsilon_min", 0.05))
        self.epsilon_decay = float(rospy.get_param("~epsilon_decay", 0.995))

        # A control tick corresponds to one RL step.
        self.control_hz = float(rospy.get_param("~control_hz", 6.0))

        # Episode controls (used in train mode).
        self.max_episodes = int(rospy.get_param("~max_episodes", 320))
        self.max_steps_per_episode = int(rospy.get_param("~max_steps_per_episode", 500))
        self.training_done_stop = bool(rospy.get_param("~training_done_stop", True))
        self.checkpoint_every_episodes = max(0, int(rospy.get_param("~checkpoint_every_episodes", 1)))
        self.reset_settle_seconds = float(rospy.get_param("~reset_settle_seconds", 0.20))
        self.reset_retry_attempts = max(1, int(rospy.get_param("~reset_retry_attempts", 3)))

        # -----------------------------
        # 3) Reward shaping config
        # -----------------------------
        # The reward is intentionally decomposed into terms so you can tune each piece.
        self.reward_right_good = float(rospy.get_param("~reward_right_good", 1.00))
        self.reward_right_too_close = float(rospy.get_param("~reward_right_too_close", -0.80))
        self.reward_right_too_far = float(rospy.get_param("~reward_right_too_far", -0.60))

        self.reward_heading_parallel = float(rospy.get_param("~reward_heading_parallel", 0.60))
        self.reward_heading_off = float(rospy.get_param("~reward_heading_off", -0.20))

        self.reward_front_safe = float(rospy.get_param("~reward_front_safe", 0.20))
        self.reward_front_too_close = float(rospy.get_param("~reward_front_too_close", -2.50))

        # Reward for commanded forward motion to prefer making progress.
        self.reward_forward_scale = float(rospy.get_param("~reward_forward_scale", 0.50))
        self.reward_front_blocked_correct_turn = float(
            rospy.get_param("~reward_front_blocked_correct_turn", 1.20)
        )
        self.reward_front_blocked_wrong_turn = float(
            rospy.get_param("~reward_front_blocked_wrong_turn", -1.20)
        )
        self.reward_front_blocked_straight = float(
            rospy.get_param("~reward_front_blocked_straight", -1.60)
        )

        # Terminal collision condition and penalty.
        self.collision_distance = float(rospy.get_param("~collision_distance", 0.30))
        self.collision_penalty = float(rospy.get_param("~collision_penalty", -8.00))
        self.trapped_position_epsilon = float(rospy.get_param("~trapped_position_epsilon", 0.008))
        self.trapped_yaw_epsilon = float(rospy.get_param("~trapped_yaw_epsilon", 0.08))
        self.trapped_steps_threshold = int(rospy.get_param("~trapped_steps_threshold", 12))
        self.trapped_penalty = float(rospy.get_param("~trapped_penalty", -4.00))
        self.tilt_termination_deg = float(rospy.get_param("~tilt_termination_deg", 55.0))
        self.z_termination_height = float(rospy.get_param("~z_termination_height", 0.25))

        # -----------------------------
        # 4) Files and reproducibility
        # -----------------------------
        self.actions_path = rospy.get_param("~actions_path")
        self.qtable_input_path = rospy.get_param("~qtable_input_path")
        self.qtable_output_path = rospy.get_param("~qtable_output_path")
        latest_qtable_output_path = str(rospy.get_param("~latest_qtable_output_path", "")).strip()
        self.metrics_csv_path = rospy.get_param("~metrics_csv_path")
        self.latest_qtable_output_path = latest_qtable_output_path or self._derive_latest_qtable_path(self.qtable_output_path)
        self.resume_training_state = bool(rospy.get_param("~resume_training_state", True))

        # In test mode, we always act greedily. In train mode, we start with epsilon-greedy.
        self.exploration_enabled = self.mode == "train"

        # Determinism helper for reproducible experiments.
        self.random_seed = int(rospy.get_param("~random_seed", 603))
        random.seed(self.random_seed)

        # -----------------------------
        # 5) Start poses for episodes
        # -----------------------------
        # Each entry is a dict with x,y,z,yaw. We sample one per episode reset.
        self.start_poses = rospy.get_param("~start_poses", [])
        if not isinstance(self.start_poses, list):
            raise ValueError("~start_poses must be a list of pose dictionaries")

        self.robot_model_name = rospy.get_param("~robot_model_name", "triton")

        # -----------------------------
        # 6) Perception and action maps
        # -----------------------------
        self.encoder = StateEncoder(
            front_too_close=float(rospy.get_param("~front_too_close", 0.55)),
            right_too_close=float(rospy.get_param("~right_too_close", 0.55)),
            right_too_far=float(rospy.get_param("~right_too_far", 0.95)),
            heading_parallel_tolerance=float(rospy.get_param("~heading_parallel_tolerance", 0.10)),
            front_sector_start_deg=float(rospy.get_param("~front_sector_start_deg", 75.0)),
            front_sector_end_deg=float(rospy.get_param("~front_sector_end_deg", 105.0)),
            right_front_sector_start_deg=float(rospy.get_param("~right_front_sector_start_deg", 20.0)),
            right_front_sector_end_deg=float(rospy.get_param("~right_front_sector_end_deg", 70.0)),
            right_rear_sector_start_deg=float(rospy.get_param("~right_rear_sector_start_deg", -70.0)),
            right_rear_sector_end_deg=float(rospy.get_param("~right_rear_sector_end_deg", -20.0)),
        )

        self.actions = self._load_actions(self.actions_path)

        # Initialize Q-table from file and fill any missing state-action entries.
        self.loaded_qtable_metadata: Dict[str, object] = {}
        self.q_table = self._load_q_table(self.qtable_input_path)
        self._ensure_dense_q_table()

        # Every training run gets a stable run_id so checkpoints can live in a
        # timestamped folder that git will never touch. If we resume from a
        # modern checkpoint, keep using its run_id.
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
        self._prime_run_artifacts()

        # -----------------------------
        # 7) Runtime state variables
        # -----------------------------
        self.latest_scan: Optional[LaserScan] = None

        # Transition memory for TD updates.
        self.prev_state_key: Optional[str] = None
        self.prev_action: Optional[str] = None

        # Episode counters.
        self.episode_idx = 0
        self.episode_step = 0
        self.episode_reward = 0.0
        self.episode_collisions = 0
        self.episode_traps = 0

        # Trap detection state.
        self.latest_robot_pose: Optional[Tuple[float, float, float, float, float, float]] = None
        self.prev_robot_pose: Optional[Tuple[float, float, float, float, float, float]] = None
        self.stationary_steps = 0

        # Keep best reward and persist best policy.
        self.best_episode_reward = float("-inf")

        # For diagnostics and plotting.
        self.episode_history: List[EpisodeStats] = []
        self._restore_training_state_if_requested()

        # -----------------------------
        # 8) ROS publishers/subscribers/services
        # -----------------------------
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1)

        self.set_model_state_srv = None
        self.pause_physics_srv = None
        self.unpause_physics_srv = None
        self.reset_world_srv = None
        if self.mode == "train":
            # We use Gazebo reset-by-pose to start new episodes from varied initial states.
            rospy.wait_for_service("/gazebo/set_model_state", timeout=20.0)
            rospy.wait_for_service("/gazebo/pause_physics", timeout=20.0)
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=20.0)
            rospy.wait_for_service("/gazebo/reset_world", timeout=20.0)
            self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            self.pause_physics_srv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            self.unpause_physics_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            self.reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        # Fixed control loop timer drives both acting and learning updates.
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / max(self.control_hz, 1.0)), self._on_control_tick)

        rospy.loginfo("RLWallFollowerD2 started")
        rospy.loginfo("algorithm=%s mode=%s", self.algorithm, self.mode)
        rospy.loginfo("alpha=%.3f gamma=%.3f epsilon=%.3f", self.alpha, self.gamma, self.epsilon)
        rospy.loginfo("qtable_input=%s", self.qtable_input_path)
        rospy.loginfo("qtable_output=%s", self.qtable_output_path)
        rospy.loginfo(
            "latest_qtable_output=%s checkpoint_every_episodes=%d",
            self.latest_qtable_output_path,
            self.checkpoint_every_episodes,
        )
        rospy.loginfo("run_id=%s run_artifact_dir=%s", self.run_id, self.run_artifact_dir)

        # Begin from a known start pose in train mode.
        if self.mode == "train":
            self._reset_for_next_episode(reason="initial_start")

    # ---------------------------------------------------------------------
    # Data loading and Q-table setup helpers
    # ---------------------------------------------------------------------
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
        return q_table

    def _ensure_dense_q_table(self):
        """Guarantee Q(s,a) exists for every discrete state and action.

        This prevents key errors and makes algorithm behavior easy to reason about.
        Missing entries are initialized to zero.
        """
        all_state_keys = []
        for f in FRONT_BINS:
            for r in RIGHT_BINS:
                for h in HEADING_BINS:
                    all_state_keys.append(f"{f}|{r}|{h}")

        for state_key in all_state_keys:
            if state_key not in self.q_table:
                self.q_table[state_key] = {}
            for action_name in self.actions.keys():
                self.q_table[state_key].setdefault(action_name, 0.0)

    @staticmethod
    def _derive_latest_qtable_path(best_path: str) -> str:
        root, ext = os.path.splitext(best_path)
        if ext:
            return f"{root}_latest{ext}"
        return f"{best_path}_latest"

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

        if (
            self.mode == "train"
            and os.path.exists(self.metrics_csv_path)
            and not os.path.exists(self.run_metrics_csv_path)
        ):
            shutil.copyfile(self.metrics_csv_path, self.run_metrics_csv_path)

        if (
            self.mode == "train"
            and "_latest" in os.path.basename(self.qtable_input_path)
            and os.path.exists(self.qtable_input_path)
            and not os.path.exists(self.run_latest_qtable_output_path)
        ):
            shutil.copyfile(self.qtable_input_path, self.run_latest_qtable_output_path)

        if (
            self.mode == "train"
            and os.path.exists(self.qtable_output_path)
            and not os.path.exists(self.run_qtable_output_path)
        ):
            shutil.copyfile(self.qtable_output_path, self.run_qtable_output_path)

    def _restore_training_state_if_requested(self):
        if self.mode != "train" or not self.resume_training_state:
            return
        if "_latest" not in os.path.basename(self.qtable_input_path):
            return

        resumed_episode_idx = 0
        best_reward = float("-inf")

        if os.path.exists(self.metrics_csv_path):
            try:
                with open(self.metrics_csv_path, "r", encoding="utf-8", newline="") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            resumed_episode_idx = max(resumed_episode_idx, int(row["episode"]) + 1)
                            best_reward = max(best_reward, float(row["reward"]))
                        except (KeyError, TypeError, ValueError):
                            continue
            except OSError as exc:
                rospy.logwarn("Failed to inspect metrics CSV for resume state: %s", exc)

        metadata_epsilon = self.loaded_qtable_metadata.get("epsilon")
        try:
            if metadata_epsilon is not None:
                self.epsilon = max(self.epsilon_min, float(metadata_epsilon))
        except (TypeError, ValueError):
            pass

        metadata_completed = self.loaded_qtable_metadata.get("completed_episodes")
        try:
            if metadata_completed is not None:
                resumed_episode_idx = max(resumed_episode_idx, int(metadata_completed))
        except (TypeError, ValueError):
            pass

        if best_reward > float("-inf"):
            self.best_episode_reward = best_reward
        self.episode_idx = resumed_episode_idx

        rospy.loginfo(
            "Resuming training state: episode_idx=%d epsilon=%.3f best_reward=%.3f",
            self.episode_idx,
            self.epsilon,
            self.best_episode_reward,
        )

    # ---------------------------------------------------------------------
    # ROS callbacks
    # ---------------------------------------------------------------------
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
        """Single periodic step for either testing or training.

        The control loop only advances when we have a recent scan.
        """
        if self.latest_scan is None:
            return

        encoded = self.encoder.encode(self.latest_scan)
        state_key = encoded.key

        if self.mode == "test":
            action = self._select_test_action(encoded)
            self._publish_action(action)
            rospy.loginfo_throttle(
                1.0,
                "[test] state=%s action=%s front=%.2f right=%.2f",
                state_key,
                action,
                encoded.front_min,
                encoded.right_min,
            )
            return

        # Train mode path.
        self._training_step(encoded)

    # ---------------------------------------------------------------------
    # Training loop
    # ---------------------------------------------------------------------
    def _training_step(self, current_state: EncodedState):
        if self.episode_idx >= self.max_episodes:
            if self.training_done_stop:
                self._publish_stop()
            return

        current_key = current_state.key

        # First step in an episode: choose and execute an initial action.
        if self.prev_state_key is None or self.prev_action is None:
            next_action = self._select_behavior_action(current_key)
            self._publish_action(next_action)

            self.prev_state_key = current_key
            self.prev_action = next_action
            self.episode_step = 1
            return

        # Build reward from *resulting* state after previous action.
        reward = self._compute_reward(current_state, self.prev_action)
        termination_reason = self._termination_reason(current_state)
        done = termination_reason is not None

        if termination_reason in {"collision", "flipped"}:
            reward += self.collision_penalty
            self.episode_collisions += 1
        elif termination_reason == "trapped":
            reward += self.trapped_penalty
            self.episode_traps += 1

        self.episode_reward += reward

        # Train mode is pure RL. Search/acquisition heuristics do not feed TD updates.
        next_behavior_action = self._select_behavior_action(current_key)

        # TD target depends on algorithm choice.
        old_q = self.q_table[self.prev_state_key][self.prev_action]

        if done:
            bootstrap = 0.0
        elif self.algorithm == "sarsa":
            # SARSA: bootstrap with Q(s', a') where a' is policy's next action.
            bootstrap = self.q_table[current_key][next_behavior_action]
        else:
            # Q-learning: bootstrap with max_a' Q(s', a').
            bootstrap = max(self.q_table[current_key].values())

        target = reward + self.gamma * bootstrap
        new_q = old_q + self.alpha * (target - old_q)
        self.q_table[self.prev_state_key][self.prev_action] = new_q

        # Episode stop criteria.
        if done or self.episode_step >= self.max_steps_per_episode:
            reason = termination_reason or "max_steps"
            self._finish_episode(reason=reason)
            return

        # Advance to next step.
        self._publish_action(next_behavior_action)
        self.prev_state_key = current_key
        self.prev_action = next_behavior_action
        self.episode_step += 1

        rospy.loginfo_throttle(
            1.0,
            "[train] ep=%d step=%d state=%s action=%s reward=%.3f eps=%.3f",
            self.episode_idx,
            self.episode_step,
            current_key,
            next_behavior_action,
            reward,
            self.epsilon,
        )

    # ---------------------------------------------------------------------
    # RL policy helpers
    # ---------------------------------------------------------------------
    def _select_greedy_action(self, state_key: str) -> str:
        state_scores = self.q_table[state_key]
        # Tie-break on action name for deterministic behavior when values equal.
        return max(state_scores.items(), key=lambda kv: (kv[1], kv[0]))[0]

    def _select_behavior_action(self, state_key: str) -> str:
        if self.exploration_enabled and random.random() < self.epsilon:
            return random.choice(list(self.actions.keys()))
        return self._select_greedy_action(state_key)

    def _select_test_action(self, state: EncodedState) -> str:
        acquisition_action = self._select_test_acquisition_action(state)
        if acquisition_action is not None:
            return acquisition_action
        return self._select_greedy_action(state.key)

    def _select_test_acquisition_action(self, state: EncodedState) -> Optional[str]:
        if not self.acquire_wall_enabled:
            return None
        if state.right_min <= self.acquire_wall_distance:
            return None
        if state.front_min > self.acquire_front_turn_distance and "straight" in self.actions:
            return "straight"
        if "turn_right_hard" in self.actions:
            return "turn_right_hard"
        if "turn_right_soft" in self.actions:
            return "turn_right_soft"
        return "straight" if "straight" in self.actions else None

    # ---------------------------------------------------------------------
    # Reward and termination
    # ---------------------------------------------------------------------
    def _compute_reward(self, state: EncodedState, action_name: str) -> float:
        reward = 0.0

        # 1) Front clearance term.
        if state.front_bin == "too_close":
            reward += self.reward_front_too_close
        else:
            reward += self.reward_front_safe

        # 2) Wall-distance term.
        if state.right_bin == "good":
            reward += self.reward_right_good
        elif state.right_bin == "too_close":
            reward += self.reward_right_too_close
        else:
            reward += self.reward_right_too_far

        # 3) Heading-alignment term.
        if state.heading_bin == "parallel":
            reward += self.reward_heading_parallel
        else:
            reward += self.reward_heading_off

        # 4) Small progress term from commanded translation.
        reward += self.reward_forward_scale * self._action_progress_speed(action_name)

        # 5) Action-aware corner shaping.
        # If the front is blocked and the right side is still occupied, we are in
        # a dead-end/left-corner style situation for a right-wall follower and
        # should commit left. If the right side has opened up, prefer turning
        # right into the opening.
        if state.front_bin == "too_close":
            turn_direction = self._action_turn_direction(action_name)
            preferred_turn = "right" if state.right_bin == "too_far" else "left"
            if turn_direction == preferred_turn:
                reward += self.reward_front_blocked_correct_turn
            elif turn_direction == "straight":
                reward += self.reward_front_blocked_straight
            else:
                reward += self.reward_front_blocked_wrong_turn

        return reward

    def _action_progress_speed(self, action_name: str) -> float:
        cfg = self.actions[action_name]
        return max(abs(float(cfg["linear_x"])), abs(float(cfg["linear_y"])))

    def _action_turn_direction(self, action_name: str) -> str:
        angular_z = float(self.actions[action_name]["angular_z"])
        if angular_z > 1e-6:
            return "left"
        if angular_z < -1e-6:
            return "right"
        return "straight"

    def _termination_reason(self, state: EncodedState) -> Optional[str]:
        # Terminal if robot gets dangerously close ahead.
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
        yaw_delta = abs(math.atan2(
            math.sin(self.latest_robot_pose[5] - self.prev_robot_pose[5]),
            math.cos(self.latest_robot_pose[5] - self.prev_robot_pose[5]),
        ))
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

    # ---------------------------------------------------------------------
    # Episode transitions and persistence
    # ---------------------------------------------------------------------
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

        # Track best policy by reward and persist immediately.
        if stats.total_reward > self.best_episode_reward:
            self.best_episode_reward = stats.total_reward
            self._save_q_table(self.qtable_output_path)
            if self.run_qtable_output_path != self.qtable_output_path:
                self._save_q_table(self.run_qtable_output_path)
            rospy.loginfo("New best policy saved (reward=%.3f)", self.best_episode_reward)

        # Append one row to metrics CSV after every episode.
        self._append_metrics_row(stats)

        # Exponential epsilon decay after each episode.
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

        self.episode_idx += 1
        self._save_latest_checkpoint(self.episode_idx)

        if self.episode_idx >= self.max_episodes:
            # Preserve the separately tracked best policy and only refresh the
            # resumable/latest checkpoint at training end.
            self.training_finished_at_utc = self._utc_iso()
            self._save_q_table(self.latest_qtable_output_path)
            if self.run_latest_qtable_output_path != self.latest_qtable_output_path:
                self._save_q_table(self.run_latest_qtable_output_path)
            rospy.loginfo("Training finished at episode %d", self.episode_idx)
            if self.training_done_stop:
                self._publish_stop()
            return

        self._reset_for_next_episode(reason=reason)

    def _reset_for_next_episode(self, reason: str):
        """Reset runtime step state and optionally reset robot pose in Gazebo."""
        self.prev_state_key = None
        self.prev_action = None
        self.episode_step = 0
        self.episode_reward = 0.0
        self.episode_collisions = 0
        self.episode_traps = 0
        self.prev_robot_pose = None
        self.stationary_steps = 0

        self._publish_stop()

        if self.set_model_state_srv is not None and self.start_poses:
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
            except Exception as exc:
                rospy.logwarn("Episode reset pose failed after '%s': %s", reason, exc)

    def _reset_robot_pose_safely(self, pose_cfg: dict):
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
        """Set robot pose using /gazebo/set_model_state.

        pose_cfg example:
          {x: 0.0, y: 0.0, z: 0.00, yaw: 1.57}
        """
        x = float(pose_cfg.get("x", 0.0))
        y = float(pose_cfg.get("y", 0.0))
        z = float(pose_cfg.get("z", 0.00))
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

        self.set_model_state_srv(state)

    def _save_q_table(self, path: str):
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
                "completed_episodes": self.episode_idx,
                "run_id": self.run_id,
                "run_started_at_utc": self.run_started_at_utc,
                "last_saved_at_utc": saved_at,
                "run_finished_at_utc": self.training_finished_at_utc,
                "source_qtable_path": self.qtable_input_path,
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
        self._save_q_table(self.latest_qtable_output_path)
        if self.run_latest_qtable_output_path != self.latest_qtable_output_path:
            self._save_q_table(self.run_latest_qtable_output_path)

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
            writer.writerow([
                stats.episode_idx,
                f"{stats.total_reward:.6f}",
                stats.steps,
                stats.collisions,
                f"{self.epsilon:.6f}",
            ])

    # ---------------------------------------------------------------------
    # Motion publishing helpers
    # ---------------------------------------------------------------------
    def _publish_action(self, action_name: str):
        cfg = self.actions[action_name]
        cmd = Twist()
        cmd.linear.x = float(cfg["linear_x"])
        cmd.linear.y = float(cfg["linear_y"])
        cmd.angular.z = float(cfg["angular_z"])
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())


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
