# VM Runbook

Quick commands for the Ubuntu VM.

## Pull Safely And Rebuild

```bash
cd ~/catkin_ws/src/wall_following_triton

STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/wf_backups/$STAMP

cp -v artifacts/qtable_d2_qlearning_latest.yaml ~/wf_backups/$STAMP/ 2>/dev/null || true
cp -v artifacts/qtable_d2_qlearning_best.yaml   ~/wf_backups/$STAMP/ 2>/dev/null || true
cp -v artifacts/d2_qlearning_metrics.csv        ~/wf_backups/$STAMP/ 2>/dev/null || true
cp -v artifacts/qtable_d2_sarsa_latest.yaml     ~/wf_backups/$STAMP/ 2>/dev/null || true
cp -v artifacts/qtable_d2_sarsa_best.yaml       ~/wf_backups/$STAMP/ 2>/dev/null || true
cp -v artifacts/d2_sarsa_metrics.csv            ~/wf_backups/$STAMP/ 2>/dev/null || true
cp -v artifacts/captured_start_poses.yaml       ~/wf_backups/$STAMP/ 2>/dev/null || true

git fetch origin
git checkout codex/d2
git pull --ff-only origin codex/d2

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## One-Time Spawn Seed Capture

Use this once to manually find good start poses for training.

Terminal 1:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh wall_following_triton wf_seed_capture.launch
```

Terminal 2:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun wall_following_triton pose_capture.py
```

Terminal 3:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun wall_following_triton triton_key_teleop.py
```

Teleop keys:

- `w/s`: forward/back on `+Y/-Y`
- `a/d`: rotate left/right
- `j/l`: strafe left/right on `-X/+X`
- `space` or `x`: stop
- `q`: quit

Pose capture keys:

- `p`: print current pose
- `a`: append current pose to `artifacts/captured_start_poses.yaml`
- `q`: quit

## Resume Q-Learning Headless

Headless mode keeps Gazebo physics running and kills the GUI client after launch.

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh --headless wall_following_triton wf_d2_qlearning_train.launch qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_qlearning_latest.yaml
```

If no latest checkpoint exists yet, start from D1:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh --headless wall_following_triton wf_d2_qlearning_train.launch
```

## Run SARSA Headless

Fresh SARSA run:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh --headless wall_following_triton wf_d2_sarsa_train.launch
```

Resume SARSA:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh --headless wall_following_triton wf_d2_sarsa_train.launch qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_sarsa_latest.yaml
```

## Quick Progress Checks

Last few Q-learning episodes:

```bash
cd ~/catkin_ws/src/wall_following_triton
tail -n 5 artifacts/d2_qlearning_metrics.csv
```

Last few SARSA episodes:

```bash
cd ~/catkin_ws/src/wall_following_triton
tail -n 5 artifacts/d2_sarsa_metrics.csv
```

Quick Q-learning summary:

```bash
cd ~/catkin_ws/src/wall_following_triton
python3 - <<'PY'
import csv
from statistics import mean

path = "artifacts/d2_qlearning_metrics.csv"
rows = list(csv.DictReader(open(path)))
last = rows[-20:] if len(rows) >= 20 else rows
print("episodes_completed:", len(rows))
print("best_reward:", max(float(r["reward"]) for r in rows))
print("avg_reward_last20:", round(mean(float(r["reward"]) for r in last), 2))
print("avg_steps_last20:", round(mean(int(r["steps"]) for r in last), 1))
print("collision_rate_last20:", round(mean(int(r["collisions"]) for r in last), 3))
PY
```

Quick SARSA summary:

```bash
cd ~/catkin_ws/src/wall_following_triton
python3 - <<'PY'
import csv
from statistics import mean

path = "artifacts/d2_sarsa_metrics.csv"
rows = list(csv.DictReader(open(path)))
last = rows[-20:] if len(rows) >= 20 else rows
print("episodes_completed:", len(rows))
print("best_reward:", max(float(r["reward"]) for r in rows))
print("avg_reward_last20:", round(mean(float(r["reward"]) for r in last), 2))
print("avg_steps_last20:", round(mean(int(r["steps"]) for r in last), 1))
print("collision_rate_last20:", round(mean(int(r["collisions"]) for r in last), 3))
PY
```

## Show What The Current Policy Does

Stop training first. Do not run this while a training launch is still publishing to `/cmd_vel`.

Test the current Q-learning policy from the live latest checkpoint:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh wall_following_triton wf_d2_qlearning_test.launch qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_qlearning_latest.yaml
```

Test the current SARSA policy from the live latest checkpoint:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh wall_following_triton wf_d2_sarsa_test.launch qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_sarsa_latest.yaml
```

## Greedy Policy Tests

Test Q-learning using the learned artifact table:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh wall_following_triton wf_d2_qlearning_test.launch qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_qlearning_best.yaml
```

Test SARSA using the learned artifact table:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/clean_roslaunch.sh wall_following_triton wf_d2_sarsa_test.launch qtable_input_path:=$(rospack find wall_following_triton)/artifacts/qtable_d2_sarsa_best.yaml
```

## Manually Promote A Learned Table Into `config/`

Do this only if you intentionally want to commit a learned table.

Promote Q-learning:

```bash
cd ~/catkin_ws/src/wall_following_triton
cp artifacts/qtable_d2_qlearning_best.yaml config/qtable_d2_qlearning_best.yaml
```

Promote SARSA:

```bash
cd ~/catkin_ws/src/wall_following_triton
cp artifacts/qtable_d2_sarsa_best.yaml config/qtable_d2_sarsa_best.yaml
```

## Current Practical Defaults

- `control_hz: 6.0`
- `max_episodes: 320`
- `max_steps_per_episode: 700`
- live training outputs go to ignored `artifacts/`
- tracked `config/*best.yaml` files are only for deliberate manual promotion
