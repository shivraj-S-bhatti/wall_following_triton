# Deliverable 2 Submission Checklist

Use this as the final checklist for a full-credit submission.

## Required deliverables

- ROS package folder
- Final chosen Q-learning policy in `config/qtable_d2_qlearning_best.yaml`
- Final chosen SARSA policy in `config/qtable_d2_sarsa_best.yaml`
- One Q-learning demo video that includes:
  - the I-beam scenario
  - the outer-wall scenario
- One SARSA demo video that includes:
  - the I-beam scenario
  - the outer-wall scenario
- One reward-vs-episode figure, tracked at `submission/d2_reward_vs_episode.png`

## Final recording starts

- I-beam: `i_beam_facing_north`
  - `x=0.731 y=-2.815 yaw=1.232`
- Outer wall: `south_corridor_entry`
  - `x=1.286 y=1.052 yaw=0.022`

## Generate the tracked reward figure

```bash
cd ~/catkin_ws/src/wall_following_triton
MPLCONFIGDIR=/tmp/mpl python3 scripts/plot_d2_metrics.py --output submission/d2_reward_vs_episode.png
```

## Promote the chosen policies

If `latest` is the winner:

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/d2_state.sh promote q_learning latest
./scripts/d2_state.sh promote sarsa latest
```

If `best` is the winner, replace `latest` with `best`.

## VM to GitHub final push

```bash
cd ~/catkin_ws/src/wall_following_triton
./scripts/d2_state.sh save pre_submission_$(date +%Y%m%d_%H%M%S)
./scripts/d2_state.sh status

MPLCONFIGDIR=/tmp/mpl python3 scripts/plot_d2_metrics.py --output submission/d2_reward_vs_episode.png

git status --short
git add config/qtable_d2_qlearning_best.yaml \
        config/qtable_d2_sarsa_best.yaml \
        submission/d2_reward_vs_episode.png
git commit -m "Promote final D2 policies and submission figure"
git push origin codex/d2-converge
```

## Final sanity checks

- `git status` is clean after the final commit
- no checkpoints, run artifacts, CSVs, or videos are staged
- manual test starts work at both recording poses
- Q-learning demo is pure greedy
- SARSA demo uses `test_deadlock_breaker_enabled:=true` only if the pure test still locally sticks
