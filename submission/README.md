# Submission Outputs

Track the final reward-vs-episode figure here:

- `submission/d2_reward_vs_episode.png`

Keep videos outside the repo or in an ignored location. Do not commit recordings,
checkpoints, run artifacts, or other VM-generated files.

## Final Deliverable Links

- Q-learning demo: `https://youtu.be/eG3-Nfrn2DY`
- SARSA demo: `https://youtu.be/jf38wAeKkVA`

## Upload Set

Upload these for the final submission:

- the ROS package folder or archive
- the final promoted Q-learning table at `config/qtable_d2_qlearning_best.yaml`
- the final promoted SARSA table at `config/qtable_d2_sarsa_best.yaml`
- the reward figure at `submission/d2_reward_vs_episode.png`
- the Q-learning video link above
- the SARSA video link above

## Optional Archive Command

From the repo root:

```bash
git archive --format=zip --prefix=wall_following_triton/ HEAD -o submission/wall_following_triton_d2_submission.zip
```
