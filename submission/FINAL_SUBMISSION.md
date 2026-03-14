# Final Submission

## Upload these items

1. `submission/wall_following_triton_d2_submission.zip`
2. Q-learning video link: `https://youtu.be/eG3-Nfrn2DY`
3. SARSA video link: `https://youtu.be/jf38wAeKkVA`

## What is inside the package archive

- full ROS package source
- final promoted Q-learning policy:
  - `config/qtable_d2_qlearning_best.yaml`
- final promoted SARSA policy:
  - `config/qtable_d2_sarsa_best.yaml`
- final reward plot:
  - `submission/d2_reward_vs_episode.png`
- operator docs and findings:
  - `README.md`
  - `VM_README.md`
  - `D2_FINDINGS.md`
  - `CHECKLIST_D2.md`

## Recording references

- I-beam start: `i_beam_facing_north`
- Outer-wall start: `south_corridor_entry`

## Notes

- Q-learning is the pure greedy baseline.
- SARSA has an opt-in, test-only deadlock breaker for demo fallback if the pure
  greedy test still sticks locally.
