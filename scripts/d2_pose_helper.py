#!/usr/bin/env python3
"""List named D2 test poses and print ready-to-run test commands."""

import argparse
import os
import sys

import yaml


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_POSES_PATH = os.path.join(ROOT, "config", "d2_named_test_poses.yaml")
ARTIFACT_POSES_PATH = os.path.join(ROOT, "artifacts", "captured_named_poses.yaml")


def load_pose_catalog():
    poses = {}
    if os.path.exists(CONFIG_POSES_PATH):
        with open(CONFIG_POSES_PATH, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        config_poses = data.get("named_test_poses", {})
        if isinstance(config_poses, dict):
            poses.update(config_poses)

    if os.path.exists(ARTIFACT_POSES_PATH):
        with open(ARTIFACT_POSES_PATH, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        captured_poses = data.get("captured_named_poses", {})
        if isinstance(captured_poses, dict):
            for name, pose in captured_poses.items():
                if isinstance(pose, dict):
                    merged_pose = dict(pose)
                    merged_pose.setdefault("description", "captured manually")
                    merged_pose.setdefault("use_for_training", False)
                    poses[name] = merged_pose

    if not poses:
        raise ValueError("No named poses found in config or artifacts")
    return poses


def resolve_checkpoint_path(algorithm, checkpoint_kind):
    algo_tag = "qlearning" if algorithm == "q_learning" else algorithm
    if checkpoint_kind == "latest":
        return f"$(rospack find wall_following_triton)/artifacts/qtable_d2_{algo_tag}_latest.yaml"
    return f"$(rospack find wall_following_triton)/artifacts/qtable_d2_{algo_tag}_best.yaml"


def build_test_command(name, pose, algorithm, checkpoint_kind):
    launch_tag = "qlearning" if algorithm == "q_learning" else algorithm
    launch = f"wf_d2_{launch_tag}_test.launch"
    qtable = resolve_checkpoint_path(algorithm, checkpoint_kind)
    return (
        "./scripts/clean_roslaunch.sh wall_following_triton "
        f"{launch} "
        f"qtable_input_path:={qtable} "
        "test_start_enabled:=true "
        f"test_start_x:={pose['x']:.3f} "
        f"test_start_y:={pose['y']:.3f} "
        f"test_start_yaw:={pose['yaw']:.3f} "
        f"test_start_z:={pose['z']:.2f}"
    )


def list_poses(poses):
    for name, pose in poses.items():
        desc = pose.get("description", "")
        training = "yes" if pose.get("use_for_training", False) else "no"
        print(
            f"{name:20s} "
            f"x={pose['x']:6.3f} y={pose['y']:6.3f} yaw={pose['yaw']:6.3f} "
            f"train={training}  {desc}"
        )


def show_pose(name, pose):
    print(f"{name}:")
    print(f"  x: {pose['x']:.3f}")
    print(f"  y: {pose['y']:.3f}")
    print(f"  z: {pose['z']:.2f}")
    print(f"  yaw: {pose['yaw']:.3f}")
    print(f"  use_for_training: {bool(pose.get('use_for_training', False))}")
    if pose.get("description"):
        print(f"  description: {pose['description']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List all named poses")

    show_parser = subparsers.add_parser("show", help="Show one named pose")
    show_parser.add_argument("name")

    cmd_parser = subparsers.add_parser("cmd", help="Print a ready-to-run test command")
    cmd_parser.add_argument("name")
    cmd_parser.add_argument(
        "--algorithm",
        choices=["q_learning", "sarsa"],
        default="q_learning",
        help="Algorithm test launch to use",
    )
    cmd_parser.add_argument(
        "--checkpoint-kind",
        choices=["best", "latest"],
        default="latest",
        help="Which learned checkpoint to test",
    )

    args = parser.parse_args()
    poses = load_pose_catalog()

    if args.command == "list":
        list_poses(poses)
        return 0

    if args.name not in poses:
        print(f"Unknown pose: {args.name}", file=sys.stderr)
        return 2

    pose = poses[args.name]

    if args.command == "show":
        show_pose(args.name, pose)
        return 0

    print(build_test_command(args.name, pose, args.algorithm, args.checkpoint_kind))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
