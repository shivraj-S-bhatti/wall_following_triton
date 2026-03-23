# D2 Report Notes

This document is the long-form companion to `D2_FINDINGS.md`.
It is meant to be useful for:

- Deliverable 3 report writing
- a YouTube retrospective
- future carry-forward work on the same wall-following setup

It keeps the full engineering story in one place: what we built, what failed,
why it failed, what alternatives we considered, and why we chose the final
solution.

## 1. Final D2 Design

### Objective

The goal stayed the same throughout Deliverable 2:

- follow the **right wall**
- handle the five required scenarios
- support both **Q-learning** and **SARSA**
- support both **training mode** and **testing mode**

### Final state representation

The original D2 design eventually converged on a **72-state** local-geometry
encoding:

- `front_bin`: `blocked` or `clear`
- `right_bin`: `too_close`, `good`, or `too_far`
- `heading_bin`: `toward_wall`, `parallel`, or `away_from_wall`
- `front_left_open_bin`: `open` or `closed`
- `front_right_open_bin`: `open` or `closed`

Why this was chosen:

- it is still compact enough for tabular RL
- it distinguishes corridor following from dead-end / U-turn geometry
- it exposes whether the robot is drifting away from the right wall
- it adds enough context to reason about openings without going to a continuous
  state space

Why the earlier state failed:

- the earlier **18-state** version aliased too many physically different
  situations into the same state
- ordinary corridor motion, blocked-front with escape options, and U-turn
  transitions looked too similar
- the learner could not assign meaningfully different Q-values to those cases

### Final action representation

The final D2 action set has 5 actions:

- `turn_left_hard`
- `turn_left_soft`
- `straight`
- `turn_right_soft`
- `turn_right_hard`

Action semantics:

- `straight` has the largest forward motion
- soft turns combine forward motion with moderate angular correction
- hard turns are near-pivot actions with **zero forward linear speed**

This action set was a compromise:

- soft turns let the policy correct while still making progress
- hard turns let it commit to a corner or escape a blocked front
- keeping only 5 actions kept the tabular problem manageable

The most important side effect of this action design:

- because hard turns had no forward motion, repeated hard turns could create a
  **test-time local deadlock** even though there was no explicit “stop” action

### Final reward design

The final reward function combines:

- front clearance
- right-wall distance quality
- heading quality
- a forward-progress term
- action-conditioned bonuses and penalties

The final intended local preferences were:

- `clear|good|parallel -> straight`
- `clear|good|toward_wall -> turn_left_soft`
- `clear|good|away_from_wall -> turn_right_soft`
- `clear|too_far|* -> reacquire the right wall with a right correction, but do
  not reward skipping the U-turn or drifting through open space`
- `blocked|* -> turn according to right-wall-following geometry, usually left`

Final reward shaping themes:

- `straight` should be strongly best only in truly stable tracking
- soft corrective turns should beat hard corrective turns in non-emergency states
- in `too_far` states, `turn_right_soft` should be better than `turn_right_hard`
  because it actually carries the robot across the U-turn mouth instead of
  pivoting back into the corridor

### Final training / evaluation design

Training:

- `alpha = 0.15`
- `gamma = 0.95`
- `epsilon` decays from `0.20` to `0.03`
- `control_hz = 6.0`
- bounded episode budget with short episodes and many resets

Evaluation:

- early on, we over-trusted “did not crash”
- later, we switched to a stricter notion of success:
  - reacquire the wall
  - hold a valid right-wall-following state for consecutive steps

This was one of the most important fixes in the project. Without it, the system
could exploit the metric while failing the real task.

## 2. What Went Wrong

### A. The policy loved going straight when it lost the wall

One of the first serious failures was:

- the robot lost the right wall
- found open space
- and kept going straight because that still paid well enough

This showed up most clearly in U-turn transitions:

- instead of turning right through the U-turn opening and reacquiring the wall
- it drifted through open space
- then recovered late and badly

Root cause:

- the reward in `right_bin == too_far` still tolerated `straight` too much

### B. SARSA learned a “270-degree right turn” style hack

This was one of the nastier behaviors.

Observed behavior:

- instead of taking the obvious left correction or smooth U-turn path
- SARSA would sometimes keep selecting right-turn behavior
- orbit around geometry
- effectively do a large right-hand sweep instead of the clean wall-following
  recovery we wanted

Why that happened:

- in `too_far` states, we initially rewarded **any** right turn too generously
- that made `turn_right_hard` attractive even when the better physical move was
  a forward-carrying `turn_right_soft`
- because `turn_right_hard` pivots in place, the policy could keep “doing the
  right direction in the abstract” while physically failing the scenario

In plain language:

- the reward function said “right-ish is good”
- but the geometry actually required “right while moving forward through the
  U-turn mouth”

That distinction mattered a lot.

### C. The robot got “stuck” even though there was no stop action

This confused things at first because the agent had no literal no-op action.

Observed behavior:

- SARSA in test mode would sit near one point and keep oscillating
- the logged states repeated `clear|too_close|...`
- the selected actions alternated between hard turns

Why this happened:

- hard turns had zero linear motion
- zero-linear hard turns can create an **effective stop** when repeated
- so the agent did not need a “stop” action to get stuck

This is an important report point:

- action semantics matter just as much as the action set labels
- a “turn” action can still behave like a no-progress trap if it has no forward
  component and the learned local Q-values reinforce it

### D. Early evaluation selected misleading checkpoints

We saw several cases where:

- `best.yaml` looked worse than `latest`
- or `best.yaml` got stuck in a local behavior that `latest` did not

Why:

- early evaluation criteria were not strong enough
- some `best` checkpoints were chosen before the reward / evaluation corrections
  stabilized
- manual testing revealed issues that the earlier evaluation pass did not

Important lesson:

- for this project, final checkpoint selection cannot rely on reward curves
  alone
- fixed-start manual scenario testing is essential

### E. Spawn/testing bugs masqueraded as policy bugs

At one point, manual test commands appeared to “ignore” the requested pose.

Root cause:

- the test-start pose was being applied before the `triton` model was guaranteed
  to exist in Gazebo

So some apparent policy failures were actually:

- a correct test command
- applied to the wrong real spawn

This matters for the report because it is an example of:

- infrastructure bugs contaminating policy evaluation

## 3. Alternatives We Considered

### Option 1: redesign the state space again

Possible direction:

- go beyond 72 states
- add more geometry bins
- add stronger opening semantics

Why we did not do it:

- the 72-state encoder already separated the major failure modes far better than
  the 18-state version
- the dominant remaining issues were reward/evaluation mismatch and local action
  semantics, not total lack of observability
- another redesign would have burned time and invalidated more of the training

### Option 2: increase action velocities or change the control regime

Possible direction:

- make the robot much faster
- add forward velocity to hard turns
- change simulation timing

Why we mostly rejected it:

- it changes the MDP, not just the learned values
- current Q-tables stop meaning the same thing when action semantics change a lot
- larger forward motion risks overshoot, clipping corners, and unstable control

We did discuss tiny forward motion for hard turns, but did not choose it for the
final branch because it would have turned into a new run configuration.

### Option 3: increase epsilon late in training

Possible direction:

- add more exploration late so the learner can escape bad local rankings

Why we did not choose it:

- the bad behavior was not mainly caused by “never trying alternatives”
- it was caused by the reward still making wrong actions locally acceptable
- more exploration would mostly add noise instead of correcting the ranking

### Option 4: add a deterministic test-only controller everywhere

Possible direction:

- hard-code dead-end and U-turn escape logic in testing

Why we rejected the broad version:

- it would hide the actual learned policy behavior
- it would contaminate the Q-learning demo, which we wanted to keep pure

What we did instead:

- a **narrow SARSA-only, opt-in, test-only deadlock breaker**
- only for the specific `too_close` pivot-loop failure
- only as a recording fallback

### Option 5: keep patching reward forever

Possible direction:

- continue micro-tuning reward terms indefinitely

Why we stopped:

- eventually the highest-value work became:
  - deterministic test starts
  - checkpoint selection
  - packaging
  - submission quality
- there is a point where report-quality analysis matters more than one more
  reward tweak

## 4. Why We Chose The Final Solution

The final solution was not “theoretically perfect.” It was the best engineering
choice under real constraints.

We chose to:

- keep the 72-state encoder
- keep the 5-action D2 action set
- narrow the reward so that:
  - `straight` stops paying in the wrong `too_far` situations
  - `turn_right_soft` beats `turn_right_hard` in U-turn reacquisition
  - soft corrections beat hard corrections in non-emergency good states
- strengthen evaluation so survival is not mistaken for success
- keep Q-learning test mode pure
- allow a SARSA-only opt-in deadlock escape for the specific final local loop
- fix the manual spawn race so the test positions are reproducible

This solution was chosen because it:

- preserved most of the useful learning already done
- addressed the real failure modes directly
- minimized risky last-minute changes
- produced a submission-quality package and demos

## 5. Final SARSA Deadlock Fix Explained Clearly

This deserves its own section because it is easy to misunderstand.

### What it was not

It was **not**:

- a change to the learned Q-table
- a retraining trick
- a generic hand-coded controller
- a fix shared with Q-learning

### What it actually was

It was a **test-only, SARSA-only, opt-in deadlock breaker**.

Trigger condition:

- test mode
- SARSA selected
- `test_deadlock_breaker_enabled = true`
- repeated visits to the same `clear|too_close|...` local state

Override:

- if heading is `parallel` or `toward_wall` -> use `turn_left_soft`
- if heading is `away_from_wall` -> use `straight`

Why it works:

- the bad SARSA loop was not “thinking” in a meaningful global sense
- it was trapped in a local hard-turn cycle with zero forward motion
- the override injects just enough forward progress to escape that cycle
- once out of the trap, the learned policy can continue

Why this was acceptable for the final project:

- it is explicitly documented
- it is narrow, not a hidden controller
- it only exists as a recording/test fallback
- Q-learning remains the unassisted baseline

## 6. Report Angles You Can Use In Deliverable 3

### Compare Q-learning vs SARSA

Useful framing:

- Q-learning is off-policy and tends to chase optimistic action values more
  aggressively
- SARSA is on-policy and can be more conservative, but also more sensitive to
  local action rankings under exploration

Observed practical difference here:

- Q-learning produced a stronger pure greedy submission policy
- SARSA was more vulnerable to local rotational failure modes in late testing

### Reward design matters as much as the algorithm

This project is a great example that:

- algorithm choice alone is not enough
- a slightly wrong reward can dominate behavior more than the choice between
  Q-learning and SARSA

Examples:

- rewarding `straight` too much when the wall is lost
- rewarding any right turn equally in U-turn states
- overpaying hard corrective turns in otherwise good states

### Evaluation design matters too

A major report point is:

- poor evaluation can reward the wrong policy
- “did not crash” is not the same as “solved the scenario”

Our final evaluation got closer to the task by requiring:

- reacquisition
- stable right-wall-following for consecutive steps

### Infrastructure affected ML conclusions

Another strong report point:

- startup races
- stale launch defaults
- reset instability
- and checkpoint bookkeeping

all changed the apparent learning outcome.

This is a good robotics lesson:

- debugging RL in simulation is never just about the RL update equation

## 7. YouTube Video Angles

If you turn this into a YouTube video, a good structure would be:

1. The task and why simple wall following becomes tricky at U-turns and dead ends
2. The first failure: too-coarse state representation
3. The second failure: reward hacking through open space
4. The third failure: the SARSA 270-degree right-turn pathology
5. The “how can it be stuck with no stop action?” moment
6. The final narrow fixes:
   - better reward
   - better evaluation
   - deterministic spawn
   - SARSA-only deadlock breaker
7. Final demos and takeaway lessons

Good sound bites:

- “The agent did not need a stop action to get stuck. Zero-forward hard turns
  created an effective stop.”
- “We were not just tuning reward; we were tuning what the agent believed the
  task actually was.”
- “The biggest mistake early on was treating survival as success.”
- “A robotics RL pipeline can fail because of the learning rule, the reward, the
  state representation, the evaluation metric, or the simulator plumbing.”

## 8. Carry-Forward Ideas

If this project continues later, the best next steps are probably:

- compare the final D2 tabular approach against a continuous-state controller
- test whether tiny forward motion on hard turns reduces local deadlocks without
  destabilizing learning
- design a cleaner evaluation suite over the five official scenarios
- separate “submission demo policy” from “research policy” more explicitly
- log per-state visitation and action histograms to make reward debugging faster
