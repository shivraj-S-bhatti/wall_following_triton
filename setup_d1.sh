#!/usr/bin/env bash
set -euo pipefail

# One-command setup for COMPSCI 603 Project 2 Deliverable 1.
# Installs required Ubuntu/ROS dependencies, clones simulator package,
# places wall_following_triton in catkin workspace src, and builds.

CATKIN_WS="${1:-$HOME/catkin_ws}"
THIS_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_PKG_DIR="$CATKIN_WS/src/wall_following_triton"
STINGRAY_DIR="$CATKIN_WS/src/stingray_sim"

echo "[INFO] Catkin workspace: $CATKIN_WS"
echo "[INFO] Repo source: $THIS_REPO_DIR"

if [[ "${EUID}" -eq 0 ]]; then
  echo "[ERROR] Do not run as root. Run as normal user with sudo access."
  exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "[ERROR] sudo is required for apt installation."
  exit 1
fi

if ! command -v apt >/dev/null 2>&1; then
  echo "[ERROR] apt not found. This script expects Ubuntu 20.04."
  exit 1
fi

echo "[INFO] Installing system dependencies..."
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-gmapping \
  python3-catkin-tools \
  python3-pip \
  python3-yaml

echo "[INFO] Installing Python dependency..."
pip3 install --user pynput

echo "[INFO] Creating workspace directories..."
mkdir -p "$CATKIN_WS/src"

if [[ ! -d "$STINGRAY_DIR/.git" ]]; then
  echo "[INFO] Cloning stingray_sim..."
  git clone https://gitlab.com/HCRLab/stingray-robotics/stingray_sim.git "$STINGRAY_DIR"
else
  echo "[INFO] Updating stingray_sim..."
  git -C "$STINGRAY_DIR" pull --ff-only
fi

if [[ "$THIS_REPO_DIR" != "$TARGET_PKG_DIR" ]]; then
  if [[ -e "$TARGET_PKG_DIR" && ! -L "$TARGET_PKG_DIR" ]]; then
    echo "[WARN] Existing $TARGET_PKG_DIR found; leaving it unchanged."
    echo "[WARN] If needed, remove it and rerun this script."
  else
    echo "[INFO] Linking wall_following_triton into workspace..."
    rm -f "$TARGET_PKG_DIR"
    ln -s "$THIS_REPO_DIR" "$TARGET_PKG_DIR"
  fi
fi

echo "[INFO] Building workspace..."
source /opt/ros/noetic/setup.bash
cd "$CATKIN_WS"
catkin_make

# shellcheck disable=SC1091
source "$CATKIN_WS/devel/setup.bash"
rospack profile

echo "[INFO] Verifying packages..."
rospack find stingray_sim >/dev/null
rospack find wall_following_triton >/dev/null

echo

echo "[SUCCESS] Setup complete."
echo "Run demo with:"
echo "  source $CATKIN_WS/devel/setup.bash"
echo "  roslaunch wall_following_triton wf_d1_demo.launch"
