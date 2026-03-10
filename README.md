# wall_following_triton (COMPSCI 603 Project 2)

ROS Noetic package for Triton right-wall following.

This repository contains:
- Deliverable 1: a manual discrete wall-following policy
- Deliverable 2: tabular Q-learning and SARSA with separate training and testing modes

## Package Layout

- `scripts/wf_policy_node.py`: D1 policy node
- `scripts/rl_d2_agent.py`: D2 learning/testing node
- `scripts/state_encoder.py`: LiDAR discretization into `(front_bin, right_bin, heading_bin)`
- `config/actions.yaml`: discrete action set
- `config/qtable_d1.yaml`: D1 manual prior table
- `config/qtable_d2_qlearning_best.yaml`: chosen final Q-learning policy for testing/submission
- `config/qtable_d2_sarsa_best.yaml`: chosen final SARSA policy for testing/submission
- `launch/wf_d1_demo.launch`: D1 demo
- `launch/wf_d2.launch`: generic D2 entrypoint
- `launch/wf_d2_*`: train/test wrappers for both algorithms

## Environment

Ubuntu 20.04 + ROS Noetic + `stingray_sim`.

Install common dependencies:

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

Clone and build:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://gitlab.com/HCRLab/stingray-robotics/stingray_sim.git
git clone https://github.com/shivraj-S-bhatti/wall_following_triton.git

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Deliverable 1

Run the manual policy demo:

```bash
roslaunch wall_following_triton wf_d1_demo.launch
```

## Deliverable 2

Deliverable 2 supports:
- `algorithm:=q_learning` or `algorithm:=sarsa`
- `mode:=train` or `mode:=test`

The convenience launches are:

```bash
roslaunch wall_following_triton wf_d2_qlearning_train.launch
roslaunch wall_following_triton wf_d2_qlearning_test.launch
roslaunch wall_following_triton wf_d2_sarsa_train.launch
roslaunch wall_following_triton wf_d2_sarsa_test.launch
```

### Training Behavior

- Train launches start from near-wall seeded poses.
- Train launches disable the acquisition heuristic so TD updates come only from RL actions.
- Live learned outputs are written to ignored `artifacts/` files:
  - `artifacts/qtable_d2_qlearning_best.yaml`
  - `artifacts/qtable_d2_qlearning_latest.yaml`
  - `artifacts/d2_qlearning_metrics.csv`
  - `artifacts/qtable_d2_sarsa_best.yaml`
  - `artifacts/qtable_d2_sarsa_latest.yaml`
  - `artifacts/d2_sarsa_metrics.csv`

### Testing Behavior

- Test launches load a chosen learned Q-table and run greedily.
- Test launches enable a simple test-only acquisition pre-controller for arbitrary spawn points.
- The submitted package should include the final chosen greedy tables in:
  - `config/qtable_d2_qlearning_best.yaml`
  - `config/qtable_d2_sarsa_best.yaml`

### Promote A Chosen Learned Table Into `config/`

Do this after evaluating `artifacts/*best.yaml` or `artifacts/*latest.yaml` in Gazebo:

```bash
cd ~/catkin_ws/src/wall_following_triton
cp artifacts/qtable_d2_qlearning_best.yaml config/qtable_d2_qlearning_best.yaml
cp artifacts/qtable_d2_sarsa_best.yaml config/qtable_d2_sarsa_best.yaml
```

### Generate Reward-Curve Figure

```bash
cd ~/catkin_ws/src/wall_following_triton
python3 - <<'PY'
import csv
import matplotlib.pyplot as plt

series = [
    ("Q-learning", "artifacts/d2_qlearning_metrics.csv"),
    ("SARSA", "artifacts/d2_sarsa_metrics.csv"),
]

for label, path in series:
    episodes = []
    rewards = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            episodes.append(int(row["episode"]))
            rewards.append(float(row["reward"]))
    if episodes:
        plt.plot(episodes, rewards, label=label)

plt.xlabel("Episode")
plt.ylabel("Accumulated Reward")
plt.title("Deliverable 2 Learning Curves")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("artifacts/d2_learning_curves.png", dpi=160)
print("saved artifacts/d2_learning_curves.png")
PY
```

## Quick Validation

Check command publisher/subscriber wiring:

```bash
rostopic info /cmd_vel
rostopic hz /cmd_vel
```

## Video Capture

GNOME recorder:
- `Ctrl` + `Alt` + `Shift` + `R` to start/stop
- file is saved in `~/Videos`

Or use `ffmpeg`:

```bash
ffmpeg -video_size 1280x720 -framerate 15 -f x11grab -i :0.0 \
  -c:v libx264 -preset veryfast -crf 30 raw_demo.mp4
```

Speed up / compress:

```bash
ffmpeg -i raw_demo.mp4 -filter:v "setpts=0.25*PTS,scale=960:-2,fps=15" \
  -c:v libx264 -preset veryfast -crf 33 -an demo_fast.mp4
```

## Submission Notes

Deliverable 2 final submission should contain:
- the ROS package folder
- the final chosen Q-learning best table in `config/`
- the final chosen SARSA best table in `config/`
- one Q-learning demo video
- one SARSA demo video
- one reward-vs-episode figure

Helpful operator docs:
- `VM_README.md`: VM commands, backups, resume/test commands
- `D2_GUIDE.md`: step-by-step learning guide
