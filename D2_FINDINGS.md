# D2 Findings

Concise log of the important engineering lessons from the D2 iterations.
This is intentionally short and decision-focused. It is not a dump of every
experiment or artifact.

For a fuller report/youtube-oriented narrative, see `D2_REPORT_NOTES.md`.

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
- Raw episodic reward was a misleading optimization target on its own; survival
  and locally decent wall-tracking could still look good while the policy failed
  the actual dead-end and U-turn scenarios.
- The first evaluation pass was too weak because it treated "survived the
  horizon" as success, which let open-space drift and skipped U-turns count as
  wins.
- Reward in `too_far` states initially tolerated `straight` too much, which made
  "leave the right wall and keep going" locally attractive.
- Reward in `too_far` states also treated all right turns as equally good, which
  let `turn_right_hard` pivot back into the corridor instead of letting
  `turn_right_soft` carry the robot forward across a U-turn mouth.
- Reward in good states was too action-flat, which made orbiting, jittering, and
  over-correction profitable enough to persist.
- Hard-turn rewards in otherwise good states also stayed too high for too long,
  which encouraged pivoting and "baby-step" forward progress instead of confident
  corridor tracking.
- Training logs were initially misleading because they printed the next action
  next to the previous action's reward, which made reward debugging harder than
  it needed to be.
- The shared D2 launch still carried stale defaults (`max_episodes=420`,
  `eval_every_episodes=20`) after the converge branch moved to a tighter budget,
  which made some VM runs behave like the old setup even after the reward logic
  had changed.
- Manual test-start coordinates were flaky because `set_model_state` could fire
  before the `triton` model existed in Gazebo, which is why some early "spawn at
  I-beam" commands silently landed at the default world start instead.
- The SARSA table was not corrupted during fine-tuning; the ugly late-stage
  failure was a test-time local minimum in `clear|too_close|...` states where
  the greedy policy alternated hard turns with zero forward motion and stayed in
  place.

## What The Debugging Sequence Actually Taught Us

- Reward hacking was real, but it was not the only issue.
- First, the reward function overpaid `straight` in `too_far` states, which let
  the robot abandon the right wall and skip the intended U-turn recovery.
- Second, once we fixed that, the reward still overpaid hard turns in otherwise
  good states, so the robot learned jittery pivot-heavy behavior instead of
  forward wall tracking.
- Third, our initial evaluation definition was too weak and made "survived for
  long enough" look like success even when the required scenario behavior was
  wrong.
- Fourth, some "policy looks broken" moments were actually stale-launch or stale
  spawn issues rather than new learning failures.
- Fifth, manual testing mattered more than the raw curves. Several times,
  `latest` looked better than `best` because `best` had been selected under an
  older evaluation regime or before the narrow reward fixes settled.

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
- Q-learning remains the pure greedy baseline for submission demos.
- SARSA is allowed a documented, opt-in, test-only local deadlock escape hatch
  for recording if the pure greedy policy still sticks in one place.

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
- D2 evaluation now requires the robot to reacquire and hold a valid
  right-wall-following state for consecutive steps instead of merely surviving.

## Hyperparameter Direction

- Favor more episodes and shorter episodes over fewer long episodes.
- Lower `alpha` to reduce oscillation in a noisier but better-defined state space.
- Keep `gamma` high for turn sequences.
- Keep `control_hz` stable at `6.0` instead of chasing speed through a changed
  MDP.
- Keep the bounded run under roughly 5 hours by shortening the total episode
  budget and the per-start evaluation horizon instead of raising `control_hz`.

## Operational Lessons

- Artifacts and tracked config files must not share ambiguous checkpoint roles.
- Before risky pulls or experiments, save state explicitly with `scripts/d2_state.sh`.
- VM backup logic must protect the actual live checkpoint paths, not just metrics.
- Headless Gazebo helps wall-clock training speed; RViz can be killed once the
  visual sanity check is done.
- A named pose catalog is worth keeping so we can reproduce edge cases quickly
  without retyping coordinates or relying on memory.
- Manual teleop, pose capture, and test-mode probing should be read-only with
  respect to RL checkpoints and metrics.
- Final submission prep needs its own pass. The package, promoted tables,
  tracked figure, ignored runtime junk, and video links are separate concerns
  and should not be improvised at the last minute.

## What To Ignore

- Raw per-episode reward alone is not a reliable convergence signal in this
  project.
- A noisy training curve does not automatically mean the learner forgot
  everything.
- The most important validation is greedy-policy behavior on the required
  scenarios plus fixed-start evaluation results.

## Narrow Fix Rationale

- The current 72-state encoder is good enough to justify one narrow fix pass;
  the remaining dominant issue is reward/evaluation mismatch, not a need for a
  third state redesign.
- The intended stable local preferences are:
  - `clear|good|parallel -> straight`
  - `clear|good|toward_wall -> turn_left_soft`
  - `clear|good|away_from_wall -> turn_right_soft`
- The intended recovery preference when `right_bin == too_far` is:
  - prefer reacquiring the wall with right turns
  - stop rewarding `straight` through open-space / skipped-U-turn behavior

## Final SARSA Deadlock Fix

- The final SARSA-specific fix is **not** a training hack and does **not**
  rewrite the learned table.
- It is a test-only, opt-in deadlock breaker used only when:
  - `mode == test`
  - `algorithm == sarsa`
  - `test_deadlock_breaker_enabled == true`
- Logic:
  - if the policy revisits the same `clear|too_close|...` state several times
    in a row, we treat that as a local pivot loop rather than useful behavior
  - when the heading is `parallel` or `toward_wall`, force `turn_left_soft`
  - when the heading is `away_from_wall`, force `straight`
- Why it works:
  - hard turns in D2 have zero forward motion
  - the stuck SARSA loop was just alternating hard turns while staying inside
    the same bad local state
  - the escape action injects just enough forward progress to leave the loop and
    let the learned policy take over again
- Q-learning test mode stays pure greedy; the helper is deliberately not shared.

## Final Submission State

- Branch: `codex/d2-converge`
- Final promoted policies live in:
  - `config/qtable_d2_qlearning_best.yaml`
  - `config/qtable_d2_sarsa_best.yaml`
- Tracked submission figure:
  - `submission/d2_reward_vs_episode.png`
- Public demo links:
  - Q-learning: `https://youtu.be/eG3-Nfrn2DY`
  - SARSA: `https://youtu.be/jf38wAeKkVA`
