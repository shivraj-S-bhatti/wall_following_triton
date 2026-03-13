# wall_following_triton (COMPSCI 603 Project 2)

ROS Noetic package for Triton right-wall following.

This repository contains:
- Deliverable 1: a manual discrete wall-following policy
- Deliverable 2: tabular Q-learning and SARSA with separate training and testing modes

## Package Layout

- `scripts/wf_policy_node.py`: D1 policy node
- `scripts/rl_d2_agent.py`: D2 learning/testing node
- `scripts/state_encoder.py`: D1 encoder plus D2 72-state encoder
- `scripts/plot_d2_metrics.py`: reward/eval plotting helper
- `config/actions.yaml`: D1 discrete action set
- `config/actions_d2.yaml`: D2 discrete action set
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

### D2 Design

- D1 keeps the original 18-state encoding.
- D2 uses a richer 72-state local geometry encoding:
  - `front_bin`
  - `right_bin`
  - `heading_bin`
  - `front_left_open_bin`
  - `front_right_open_bin`
- D2 `best.yaml` is selected by fixed greedy evaluation, not by the single highest training-episode reward.

### Training Behavior

- Train launches start from near-wall seeded poses.
- Train launches disable the acquisition heuristic so TD updates come only from RL actions.
- Live learned outputs are written to ignored `artifacts/` files:
  - `artifacts/qtable_d2_qlearning_best.yaml`
  - `artifacts/qtable_d2_qlearning_latest.yaml`
  - `artifacts/d2_qlearning_metrics.csv`
  - `artifacts/d2_qlearning_eval.csv`
  - `artifacts/qtable_d2_sarsa_best.yaml`
  - `artifacts/qtable_d2_sarsa_latest.yaml`
  - `artifacts/d2_sarsa_metrics.csv`
  - `artifacts/d2_sarsa_eval.csv`
- Every train run is also mirrored into a timestamped ignored run folder:
  - `artifacts/runs/q_learning/<run_id>/...`
  - `artifacts/runs/sarsa/<run_id>/...`
- Q-table metadata now includes:
  - `run_id`
  - `completed_episodes`
  - `run_started_at_utc`
  - `last_saved_at_utc`
  - `run_finished_at_utc`
  - `checkpoint_kind`
  - `best_eval_score`

### State Management

Use the helper script instead of manual copies:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/d2_state.sh status
./scripts/d2_state.sh save pre_pull_$(date +%Y%m%d_%H%M%S)
./scripts/d2_state.sh load pre_pull_20260312_203000
./scripts/d2_state.sh discard current
./scripts/d2_state.sh adopt-legacy q_learning
```

This script:
- snapshots the live D2 state into `artifacts/checkpoints/<label>/`
- restores a checkpoint back into live `artifacts/`
- archives discarded live state into `artifacts/discarded/<timestamp>/`
- migrates any legacy `config/qtable_d2_*_latest.yaml` checkpoint into `artifacts/` on load

### Testing Behavior

- Test launches load a chosen learned Q-table and run greedily.
- Test launches enable a simple test-only acquisition pre-controller for arbitrary spawn points.
- Test launches can optionally teleport Triton to a manual edge-case pose:
  - `test_start_enabled:=true`
  - `test_start_x:=...`
  - `test_start_y:=...`
  - `test_start_yaw:=...`
  - optional `test_start_z:=...`
- The submitted package should include the final chosen greedy tables in:
  - `config/qtable_d2_qlearning_best.yaml`
  - `config/qtable_d2_sarsa_best.yaml`

Example manual edge-case test:

```bash
roslaunch wall_following_triton wf_d2_sarsa_test.launch \
  qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_sarsa_best.yaml \
  test_start_enabled:=true \
  test_start_x:=1.46 \
  test_start_y:=-1.223 \
  test_start_yaw:=0.079
```

### Named Manual Test Poses

The repo now includes a named pose catalog in:

- `config/d2_named_test_poses.yaml`
- `artifacts/captured_named_poses.yaml`

Use the helper script to list poses, inspect one, or print a ready-to-run test
command:

```bash
cd ~/catkin_ws/src/wall_following_triton
python3 scripts/d2_pose_helper.py list
python3 scripts/d2_pose_helper.py show lower_right_u_turn
python3 scripts/d2_pose_helper.py cmd lower_right_u_turn --algorithm sarsa --checkpoint-kind latest
```

The current catalog includes safe manually validated poses for:

- lower and upper right U-turn probes
- a mid-right corridor / I-beam style probe
- lower corridor turn probing
- two shifted-inward west-side reacquisition starts

The helper reads both tracked config poses and any manually captured named poses
from `artifacts/captured_named_poses.yaml`, so ad hoc pose captures become
available immediately without editing tracked files.

These named poses are also reflected in the D2 training seed set in
`config/d2_params.yaml`, so future training runs see broader map coverage than
the original four safe seeds.

### Promote A Chosen Learned Table Into `config/`

Do this after evaluating `artifacts/*best.yaml` or `artifacts/*latest.yaml` in Gazebo:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/d2_state.sh promote q_learning best
./scripts/d2_state.sh promote sarsa best
```

### Generate Reward Figures

```bash
cd ~/catkin_ws/src/wall_following_triton
python3 scripts/plot_d2_metrics.py --output artifacts/d2_converge_plots.png
```

Add `--include-eval` if you also want the greedy-evaluation subplot.

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
