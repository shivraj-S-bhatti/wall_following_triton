# wall_following_triton (Project 2 Deliverable 1)

Manual Q-table wall-following policy for Triton in ROS Noetic/Gazebo.

This package implements Deliverable 1 requirements:
- state discretization from `/scan`
- manually defined Q-table policy
- right-side straight-wall following
- one launch file to run simulation + policy

## Package Layout

- `scripts/wf_policy_node.py`: policy node (subscribes `/scan`, publishes `/cmd_vel`)
- `scripts/state_encoder.py`: LiDAR-to-state encoder
- `config/qtable_d1.yaml`: manually defined Q-values
- `config/actions.yaml`: discrete actions and velocities
- `launch/wf_d1_demo.launch`: one-command demonstration launch

## Decision Defaults (D1)

- Wall side: right
- Desired wall distance: `0.75 m`
- State bins:
  - `front_bin`: `too_close`, `safe`
  - `right_bin`: `too_close`, `good`, `too_far`
  - `heading_bin`: `toward_wall`, `parallel`, `away_from_wall`
- Thresholds:
  - `front too_close < 0.55`
  - `right too_close < 0.55`
  - `right too_far > 0.95`
  - `parallel if abs(right_front - right_rear) <= 0.10`

## Host-Edit + VM-Run Workflow

You edit code on host and run ROS/Gazebo in your Ubuntu VM.

### 1) Push code from host

```bash
cd /Users/apple/Downloads/robotics/wall_following_triton
git add .
git commit -m "Implement D1 manual Q-table wall-following package"
git push origin main
```

### 2) Pull and place package in VM catkin workspace

```bash
cd ~/catkin_ws/src
git clone https://github.com/shivraj-S-bhatti/wall_following_triton.git
# or, if already cloned:
cd ~/catkin_ws/src/wall_following_triton
git pull
```

### 3) Build in VM

```bash
cd ~/catkin_ws
catkin_make --pkg wall_following_triton
source devel/setup.bash
```

## Minimal Triton Simulator Setup (Low Disk)

If ROS/Gazebo is installed but Triton sim is missing:

```bash
cd ~/catkin_ws/src
git clone --depth 1 https://gitlab.com/HCRLab/stingray-robotics/stingray_sim.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Optional disk cleanup after install/build:

```bash
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*
rm -rf ~/.cache/pip
```

## Run Deliverable 1 Demo

Default world from `stingray_sim`:

```bash
roslaunch wall_following_triton wf_d1_demo.launch
```

Use a custom world:

```bash
roslaunch wall_following_triton wf_d1_demo.launch world_file:=/absolute/path/to/world.world
```

Run only the policy node (if Gazebo is already launched elsewhere):

```bash
roslaunch wall_following_triton wf_d1_demo.launch start_sim:=false
```

## Quick Validation Commands (VM)

Check command publish rate:

```bash
rostopic hz /cmd_vel
```

Inspect command values:

```bash
rostopic echo -n 10 /cmd_vel
```

## Deliverable 1 Acceptance Checklist

- [ ] `roslaunch wall_following_triton wf_d1_demo.launch` starts Gazebo + node
- [ ] no crash during 5 x 60s straight-wall runs
- [ ] robot keeps moving while following right straight wall
- [ ] front obstacle triggers visible left recovery turn
- [ ] include demo video (or public link), straight-wall behavior shown clearly
- [ ] submit `P2D1_firstname_lastname.tar(.gz)` containing:
  - ROS package
  - demo video or video link reference

## Demo Video Compression (<20 MB)

```bash
ffmpeg -i raw_demo.mp4 -vf "scale=1280:-2,fps=20" -c:v libx264 -crf 30 -preset veryfast -an demo_d1_compressed.mp4
```

## Packaging for Canvas

From directory containing package + demo video:

```bash
tar -czf P2D1_firstname_lastname.tar.gz wall_following_triton demo_d1_compressed.mp4
```

## Notes

- Update maintainer name/email in `package.xml` before submission.
- This package intentionally targets straight-wall following only (D1 scope).
