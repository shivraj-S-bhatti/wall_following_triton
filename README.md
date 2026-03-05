# wall_following_triton (COMPSCI 603 Project 2 - Deliverable 1)

This package implements a manually defined Q-table policy for Triton straight-wall following in ROS Noetic + Gazebo.

## What this package provides

- `scripts/wf_policy_node.py`: subscribes to `/scan`, publishes `geometry_msgs/Twist` on `/cmd_vel`
- `scripts/state_encoder.py`: discretizes LiDAR into `(front_bin, right_bin, heading_bin)`
- `config/qtable_d1.yaml`: manual Q-values for D1 policy
- `config/actions.yaml`: discrete action set and velocities
- `launch/wf_d1_demo.launch`: one-command demo launch (starts Gazebo + policy)

## Required environment (Ubuntu 20.04)

Install ROS Noetic desktop full and simulator dependencies (as in stingray guide):

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-gmapping \
  python3-catkin-tools \
  python3-pip
pip3 install --user pynput
```

Create catkin workspace (if needed):

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

Clone required simulator package:

```bash
cd ~/catkin_ws/src
git clone https://gitlab.com/HCRLab/stingray-robotics/stingray_sim.git
```

Clone this package:

```bash
cd ~/catkin_ws/src
git clone https://github.com/shivraj-S-bhatti/wall_following_triton.git
```

Build:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run demo (TA command)

```bash
roslaunch wall_following_triton wf_d1_demo.launch
```

This launch file automatically starts:
- `stingray_sim/launch/wall_following.launch`
- `wall_following_triton/scripts/wf_policy_node.py`

## Quick validation

```bash
rostopic info /cmd_vel
rostopic hz /cmd_vel
```

Expected:
- `/wf_policy_node` publishes to `/cmd_vel`
- Gazebo has a subscriber on `/cmd_vel`

## Demo video workflow (Ubuntu)

### Option 1: GUI screen recorder (easy)

Use built-in GNOME recorder:
- `Ctrl` + `Alt` + `Shift` + `R` to start/stop recording
- file is saved in `~/Videos`

### Option 2: ffmpeg (scriptable)

Record desktop at lower fps to keep size smaller:

```bash
ffmpeg -video_size 1280x720 -framerate 15 -f x11grab -i :0.0 \
  -c:v libx264 -preset veryfast -crf 30 raw_demo.mp4
```

Then speed up 4x and compress for Canvas (<20 MB target):

```bash
ffmpeg -i raw_demo.mp4 -filter:v "setpts=0.25*PTS,scale=960:-2,fps=15" \
  -c:v libx264 -preset veryfast -crf 33 -an demo_d1_fast.mp4
```

## Submission packaging (P2-D1)

Submit a single tarball named `P2D1_firstname_lastname.tar` or `.tar.gz` with exactly two items:
1. ROS package folder `wall_following_triton`
2. Demo video file (or a text file containing public video link)

Example:

```bash
tar -czf P2D1_firstname_lastname.tar.gz wall_following_triton demo_d1_fast.mp4
```

## Important notes for submission

- Do **not** bundle ROS itself or Ubuntu packages inside your tarball.
- A setup script is optional, not required by D1 rubric.
- This package is intentionally scoped to straight-wall following (D1 requirement).
- Update maintainer name/email in `package.xml` before final upload.
