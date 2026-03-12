# Deliverable 2 Learning Guide (Comment + Instruction Heavy)

This guide explains **what to run**, **why each command matters**, and **what code path executes**.

## 1) What changed from D1 to D2

D1 used a fixed/manual policy (`wf_policy_node.py`) and never updated Q-values.

D2 adds `scripts/rl_d2_agent.py`, which can run in two modes:
- `train`: learns Q-values online with TD updates
- `test`: loads a learned Q-table and runs greedy policy

Algorithms supported:
- `q_learning`
- `sarsa`

## 2) Files you should know first

- `scripts/rl_d2_agent.py`
  - Main learning loop
  - Reward function
  - Q-learning / SARSA update logic
  - Episode reset and Q-table save
- `launch/wf_d2.launch`
  - Single launch entrypoint with `algorithm` and `mode` args
- `launch/wf_d2_*`
  - Convenience wrappers for each train/test algorithm variant
- `config/d2_params.yaml`
  - Hyperparameters, reward weights, start poses, thresholds
- `artifacts/qtable_d2_qlearning_best.yaml`
- `artifacts/qtable_d2_sarsa_best.yaml`
  - Live learned policies during training
- `artifacts/runs/<algorithm>/<run_id>/`
  - Timestamped per-run snapshots mirrored automatically during training
- `scripts/d2_state.sh`
  - One-command state management for save/load/discard/promote
- `config/qtable_d2_qlearning_best.yaml`
- `config/qtable_d2_sarsa_best.yaml`
  - Final promoted policies used in testing/submission mode

## 3) Build

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 4) Run training

### Q-learning training

```bash
roslaunch wall_following_triton wf_d2_qlearning_train.launch
```

Outputs:
- learned policy saved to: `artifacts/qtable_d2_qlearning_best.yaml`
- metrics CSV saved to: `artifacts/d2_qlearning_metrics.csv`
- mirrored run snapshot saved to: `artifacts/runs/q_learning/<run_id>/`

### SARSA training

```bash
roslaunch wall_following_triton wf_d2_sarsa_train.launch
```

Outputs:
- learned policy saved to: `artifacts/qtable_d2_sarsa_best.yaml`
- metrics CSV saved to: `artifacts/d2_sarsa_metrics.csv`
- mirrored run snapshot saved to: `artifacts/runs/sarsa/<run_id>/`

## 5) Run testing (learned policy only)

### Q-learning testing

```bash
roslaunch wall_following_triton wf_d2_qlearning_test.launch
```

### SARSA testing

```bash
roslaunch wall_following_triton wf_d2_sarsa_test.launch
```

In test mode, the node does not update Q-values. It chooses greedy action from the loaded Q-table.

## 6) How the learning update works (step-by-step)

Inside each control tick in `train` mode:

1. Encode latest `/scan` into discrete state key `s_t`.
2. If this is first step in episode, pick action `a_t` with epsilon-greedy and publish.
3. On next tick, compute reward `r_{t+1}` from new state.
4. Update previous Q entry:
   - Q-learning:
     - target = `r + gamma * max_a Q(s', a)`
   - SARSA:
     - target = `r + gamma * Q(s', a')`
5. Apply TD update:
   - `Q(s,a) <- Q(s,a) + alpha * (target - Q(s,a))`
6. End episode if collision or max steps reached.
7. Save best Q-table to `artifacts/` and append CSV metrics row.

Important design choice:
- training is pure RL
- the acquisition pre-controller is reserved for `test` mode only
- this prevents search heuristics from polluting TD updates

## 6.5) Save / load / discard state without manual copying

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/d2_state.sh status
./scripts/d2_state.sh save before_pull_$(date +%Y%m%d_%H%M%S)
./scripts/d2_state.sh load before_pull_20260312_203000
./scripts/d2_state.sh discard current
```

Use this instead of ad hoc copying between `artifacts/` and `config/`.

## 7) Most important tuning knobs

Edit in `config/d2_params.yaml`:

- Learning dynamics:
  - `alpha`, `gamma`
- Exploration:
  - `epsilon`, `epsilon_min`, `epsilon_decay`
- Episode structure:
  - `max_episodes`, `max_steps_per_episode`, `control_hz`
- Reward shaping:
  - `reward_right_good`, `reward_heading_parallel`, etc.
- Collision sensitivity:
  - `collision_distance`, `collision_penalty`
- Start poses:
  - `start_poses` list for better generalization from near-wall reset states

## 8) Generate the reward-vs-episode figure for report

Quick Python plot from CSV:

```bash
python3 - <<'PY'
import csv
import matplotlib.pyplot as plt

paths = [
    'artifacts/d2_qlearning_metrics.csv',
    'artifacts/d2_sarsa_metrics.csv',
]
for p in paths:
    ep, rw = [], []
    with open(p, 'r', encoding='utf-8') as f:
        r = csv.DictReader(f)
        for row in r:
            ep.append(int(row['episode']))
            rw.append(float(row['reward']))
    plt.plot(ep, rw, label=p.split('/')[-1].replace('_metrics.csv', ''))

plt.xlabel('Episode')
plt.ylabel('Accumulated reward')
plt.title('D2 Learning Curves')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('artifacts/d2_learning_curves.png', dpi=160)
print('saved artifacts/d2_learning_curves.png')
PY
```

## 9) Common failure modes and fixes

- Robot does not move:
  - verify `/cmd_vel` has Gazebo subscriber:
    - `rostopic info /cmd_vel`
- Package not found:
  - source order must be:
    - `source /opt/ros/noetic/setup.bash`
    - `source ~/catkin_ws/devel/setup.bash`
- Training seems unstable:
  - reduce `epsilon`
  - lower `alpha`
  - increase `reward_right_good`
  - simplify start poses initially

## 10) Suggested D2 workflow for learning

1. Train Q-learning for 60-100 episodes.
2. Test Q-learning policy and record short video.
3. Train SARSA with same reward config.
4. Test SARSA policy and record short video.
5. Plot both reward curves from CSV.
6. Iterate on rewards/start poses if behavior is weak.
