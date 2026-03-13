# D2 Findings

Concise log of the important engineering lessons from the D2 iterations.
This is intentionally short and decision-focused. It is not a dump of every
experiment or artifact.

## Confirmed Facts

- Triton project frame for Project 2 uses `+Y` as forward.
- `LaserScan` angle `0` points along `+X` and increases counterclockwise.
- Positive `angular.z` is a left turn, negative `angular.z` is a right turn.
- D1 behavior looked good even when our earlier frame assumptions were wrong
  because the old controller was internally self-consistent, not because the
  frame interpretation was correct.

## What Failed And Why

- The original D2 18-state representation was too coarse for dead ends, I-turns,
  and U-turns.
- Multiple physically different situations aliased into the same state:
  ordinary corridor, blocked-front right opening, blocked-front left escape,
  and dead-end/U-turn-like geometry.
- Choosing `best.yaml` from the single highest training-episode reward was too
  noisy and often selected a worse policy than the late-stage average.
- Test-time acquisition logic mixed into the training loop polluted TD updates.
- Resetting a dynamic Gazebo robot without carefully clearing physics state could
  produce flips, vertical displacement, or unstable roll/pitch.
- Some manually captured start poses were too close to walls and caused collision
  overlap at reset time.

## Decisions We Are Keeping

- D1 stays unchanged.
- D2 uses a separate state encoder and a separate action file.
- D2 training is a near-wall wall-following problem, not an open-space wall-search
  problem.
- Test-time acquisition remains outside the learning loop.
- Hard turns for D2 should behave more like committed turn actions than forward
  arcs.
- Checkpoint selection for D2 should be based on fixed greedy evaluation, not
  single-episode reward spikes.
- Manual test-start coordinates are useful and should be supported directly in
  launch files.

## Current Branch Direction

- Branch: `codex/d2-converge`
- D2 state:
  - `front_bin`
  - `right_dist_bin`
  - `heading_bin`
  - `front_left_open_bin`
  - `front_right_open_bin`
- Total D2 state count target: `72`
- D2-only actions live in `config/actions_d2.yaml`
- D2 evaluation writes dedicated CSVs and drives `best.yaml`

## Hyperparameter Direction

- Favor more episodes and shorter episodes over fewer long episodes.
- Lower `alpha` to reduce oscillation in a noisier but better-defined state space.
- Keep `gamma` high for turn sequences.
- Keep `control_hz` stable at `6.0` instead of chasing speed through a changed
  MDP.

## Operational Lessons

- Artifacts and tracked config files must not share ambiguous checkpoint roles.
- Before risky pulls or experiments, save state explicitly with `scripts/d2_state.sh`.
- VM backup logic must protect the actual live checkpoint paths, not just metrics.
- Headless Gazebo helps wall-clock training speed; RViz can be killed once the
  visual sanity check is done.

## What To Ignore

- Raw per-episode reward alone is not a reliable convergence signal in this
  project.
- A noisy training curve does not automatically mean the learner forgot
  everything.
- The most important validation is greedy-policy behavior on the required
  scenarios plus fixed-start evaluation results.
