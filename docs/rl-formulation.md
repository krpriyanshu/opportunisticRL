# RL Formulation

This document is the formal specification for the RL layer: action space,
observation space, reward, episode structure, opportunity representation,
and the baseline ladder. It is written before any RL code exists
(Milestone M0) so that later implementation is checked against a fixed
spec rather than the spec being inferred from whatever code happens to get
written. When implementation at M5+ forces a change to this spec, the
change is made here first, with a note on why, rather than left
undocumented in code.

See [research-question.md](research-question.md) for why this formulation
exists, and [architecture.md](architecture.md#confound-register) for the
confounds this formulation is specifically designed to avoid.

## Framework decision

**PyTorch** is selected as the RL framework, per Section 12's preference
for PyTorch "if it simplifies integration and experimentation." Rationale
recorded per Section 51.6:

- No JAX-specific performance requirement exists yet (Section 51: training
  is simulation-based and correctness, not throughput, is primary).
- The environment this repo is built in already has a working
  CUDA-enabled PyTorch install, avoiding an extra dependency-resolution
  step (verified: `torch==2.12.1+cu130`).
- PyTorch's ecosystem (Gymnasium, Stable-Baselines3 or a small custom PPO)
  is the more direct path to B2 (standard RL baseline) without building
  RL infrastructure ourselves, per the explicit instruction not to build a
  custom RL framework (Section 12).

This is revisited only if a concrete performance bottleneck is measured
(not assumed) at M6+.

## Action space (Section 6)

Discrete, initially 4 actions, shared by every baseline (Confound #2/#3):

| Value | Name | Meaning |
|---|---|---|
| 0 | `WAIT` | Hold current pose; no approach or grasp motion commanded. |
| 1 | `APPROACH` | Command the manipulation manager toward the current grasp-candidate pose. |
| 2 | `ATTEMPT_GRASP` | Commit to a grasp execution via the conventional manipulation stack (Section 7). |
| 3 | `ABORT` | Cancel an in-progress approach/grasp and return to a safe standby pose. |

A fifth action, `REPOSITION`, and/or continuous parameters (e.g. a
target-relative offset) are explicitly deferred (Section 6) until the
4-action formulation is shown to be insufficient by an actual experiment,
not added speculatively.

The action is issued at a fixed **decision period** (a config value, not
per-baseline), decoupling the RL decision rate from the underlying
controller's rate. This value is part of the environment config so it is
identical across all baselines being compared (Confound #2).

## Observation space (Section 8)

The observation is a flat, normalized vector with a documented schema.
Exact dimensionality depends on the finalized robot/simulator (Section 8's
own caveat) and will be fixed in code at M5 as `ObservationSchema`, but
the *fields* are fixed now:

```text
relative_target_position       (3,)   meters, target frame relative to end-effector
relative_target_orientation    (4,)   quaternion, target relative to end-effector
relative_target_linear_vel     (3,)   m/s, in end-effector/base frame
relative_target_angular_vel    (3,)   rad/s
ee_pose                        (7,)   position (3) + quaternion (4), vessel-base frame
joint_state                    (2*N,) position + velocity for N manipulator joints
vessel_linear_vel              (3,)   m/s, world frame
vessel_angular_vel             (3,)   rad/s
distance_to_grasp_region       (1,)   meters
orientation_error              (1,)   radians
relative_approach_velocity     (1,)   m/s, closing speed along approach axis
task_state                     (K,)   one-hot or small integer encoding of current phase
```

Rules (binding on the implementation at M5, not aspirational):

- Every field is normalized to a documented, bounded range before being
  handed to a policy network. Raw units above are the *pre-normalization*
  definition; the normalization constants live in config, not hard-coded.
- The conversion from simulator/ROS2 state to this vector is a pure,
  deterministic function — no hidden simulator handle is available to
  policy code.
- `pose_uncertainty` / `velocity_uncertainty` fields are supported by the
  schema (as optional, zero-filled-by-default fields) but **disabled** in
  the first experiment, per Section 8. Enabling them is an explicit,
  documented config change, not a silent default.
- The **reactive** and **opportunity-aware** policies read the *same*
  `ObservationSchema` above. The opportunity-aware policy additionally
  receives the output of an `OpportunityEstimator` (next section) as an
  explicitly separate, named field appended to the vector — never mixed
  into the base fields — so Confound #1 (unequal unrelated information)
  is checkable by inspecting one field list rather than a diff of two
  parallel schemas.

## Future opportunity representation (Section 9)

Interface, not a fixed formula:

```text
OpportunityEstimator:
    input:  current state, optional history, optional predicted target trajectory
    output: opportunity representation
```

Three representation shapes are supported by the interface without
requiring an environment rewrite (Section 9):

1. **Scalar opportunity score** `O(t) ∈ [0, 1]` at the current instant.
2. **Future opportunity vector** `[O(t+Δ1), ..., O(t+ΔN)]` over a fixed
   horizon.
3. **Probability of successful grasp** at each of several future horizons.

**First implementation (M8): deterministic/analytic, representation (2).**
A closed-form function of the same measurable quantities used for the
Section 10 feasibility definition (position error, orientation error,
relative velocity, relative angular velocity), projected forward using the
target's *currently estimated* velocity — not the simulator's true future
trajectory (Confound #7 / Section 39). This gives a non-RL reference
opportunity signal before any learned opportunity model is attempted, and
it is deliberately kept legible so its behavior can be unit-tested
directly (good state → high opportunity, bad state → low opportunity, per
Section 34).

A learned opportunity model (e.g. a small trajectory predictor) is a later
extension, only pursued if the analytic version is shown to be a
bottleneck — not built speculatively alongside it.

**Future-information leakage rule (Section 39, binding):**

| Input | Status |
|---|---|
| Current observation | Valid |
| Learned/analytic prediction from current + past state | Valid |
| Short-horizon *estimated* trajectory (extrapolated, not ground truth) | Valid, with caution — must be clearly derived, never copied from simulator future state |
| True future simulator trajectory | **Invalid** for any policy input. Reserved exclusively for the oracle baseline (B-oracle, Section 27), which is never used as a policy, only as an evaluation reference. |

## Grasp opportunity / feasibility definition (Section 10)

Deterministic, and shared by every baseline that needs a feasibility
notion (B0, B1, and the analytic `OpportunityEstimator`) so that B0/B1's
behavior and the opportunity signal are computed from one definition, not
reimplemented three times:

```text
reachable        := target within manipulator workspace bound
graspable         := reachable AND grasp geometry valid (approach axis, clearance)
good_opportunity  := graspable AND orientation_error < θ_max
                                AND relative_velocity < v_max
                                AND relative_angular_velocity < ω_max
successful_execution := realized outcome from the conventional grasp controller,
                         reported after ATTEMPT_GRASP — not assumed from
                         good_opportunity
```

`good_opportunity` being true does not imply `successful_execution` will
be true — the manipulation stack can still fail. This distinction is what
lets the project separate "did the policy pick a good moment" from "did
the controller execute correctly," which matters for failure analysis
(Section 41).

## Reward (Section 16)

```text
R_t = R_success · 1[success]
      - λ_f · C_failure
      - λ_c · C_collision
      - λ_t · C_time
      - λ_m · C_motion
```

- One implementation, parameterized by config (`R_success`, `λ_f`, `λ_c`,
  `λ_t`, `λ_m`), shared across every baseline that is RL-trained
  (Confound #6). Heuristic baselines (B0/B1) do not use the reward at
  decision time (they're not learned) but episodes are still scored with
  it for comparable metrics.
- `C_time` is applied every step regardless of action, including `WAIT`,
  so that indefinite waiting has an explicit, nonzero opportunity cost
  (Section 16's binding requirement: waiting forever must not be optimal).
  This is checked directly by a unit test at M5 (an all-`WAIT` episode's
  return must be strictly less than a successful episode's return under
  default config).
- Reward shaping beyond the five terms above is deliberately avoided
  ("do not over-shape the reward," Section 16). If shaping is later found
  necessary, it is added as a named, documented, separately-ablatable term
  — not folded into an existing coefficient.

## Episode structure (Section 17)

Terminates on exactly one of: `success` (grasp executed and verified),
`target_lost` (target leaves workspace bounds), `collision` (unsafe
contact), `timeout` (configurable max duration reached), or
`irrecoverable_state` (robot state outside recoverable bounds, e.g. a
controller fault). The termination reason is recorded on every episode
(feeds Section 41 failure analysis) — episodes never terminate silently
with an unclassified reason.

## Baseline ladder (Section 11)

Implemented strictly in this order; each is validated before the next is
attempted (Section 51.4). All share: action space, observation schema
(minus the opportunity field, where applicable), reward, controller stack,
and — for RL baselines — training step budget and disturbance
distribution (per the confound register).

| ID | Name | Decision rule | Learned? |
|---|---|---|---|
| B0 | Immediate/heuristic | `GRASP` iff `good_opportunity`, else `WAIT` | No |
| B1 | Reactive heuristic | Explicit configurable thresholds on distance/orientation/velocity (superset of B0's logic, tunable independently) | No |
| B2 | Standard RL (reactive) | PPO or SAC over the base observation schema only | Yes |
| B3 | Temporal RL | B2 + observation history (stacked frames or a recurrent (GRU/LSTM) policy) — only implemented if M5/M6 data shows meaningful temporal dependence, per Section 11 | Yes |
| B4 | Opportunity-aware RL (proposed) | B2/B3 + `OpportunityEstimator` output appended to observation | Yes |
| Oracle | Timing reference (not a baseline for H1) | Full future trajectory → best achievable grasp time, computed offline | No (not a policy) |

PPO is the tentative choice for B2 over SAC, since the action space is
discrete (Section 6) and PPO has simpler, more standard discrete-action
support; this is confirmed (not just assumed) once B2 is actually
implemented at M6, and documented there if SAC (or a discrete SAC variant)
turns out to be preferable.

## Training / evaluation separation (Section 13)

Never share a seed or disturbance draw between a training run and the
evaluation used to report that run's performance. Training uses randomized
disturbance draws across multiple seeds; evaluation uses fixed seeds and
explicitly named scenarios (E0–E5, defined in `docs/evaluation.md`,
written at M11 when evaluation scenarios are implemented). Training-time
episode statistics may be logged for monitoring but are never the number
reported as a result.

## Curriculum (Section 15)

Configurable staged difficulty, not hard-coded into the environment:
static → low target motion → low vessel disturbance → combined motion →
higher disturbance → observation noise → distribution shift. Implemented
as a list of named disturbance configs in `configs/experiments/`, selected
by training script argument, so the curriculum itself is an experiment
artifact rather than logic embedded in `DynamicGraspEnv`.

## Interfaces (Sections 21, 22, 12)

Fixed method signatures (implemented starting M5; recorded here first):

```python
class SimulationInterface(Protocol):
    def reset(self, seed: int | None = None) -> None: ...
    def step(self, dt: float) -> None: ...
    def get_robot_state(self) -> RobotState: ...
    def get_target_state(self) -> TargetState: ...
    def get_environment_state(self) -> EnvironmentState: ...
    def apply_action(self, action: int) -> None: ...


class DynamicGraspEnv(gymnasium.Env):
    def reset(self, *, seed: int | None = None, options: dict | None = None): ...
    def step(self, action: int): ...

    observation_space: gymnasium.spaces.Box
    action_space: gymnasium.spaces.Discrete


class RLAlgorithm(Protocol):
    def train(self, total_steps: int) -> None: ...
    def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult: ...
    def save_checkpoint(self, path: Path) -> None: ...
    def load_checkpoint(self, path: Path) -> None: ...
```

`RLAlgorithm` configuration surfaces (Section 12, all config-driven, not
hard-coded): seed, environment config, algorithm config, network config,
training steps, evaluation frequency, checkpoint frequency.

## Status of this document

This is a specification, written before implementation, per Section 51's
instruction to keep the research layer's interfaces fixed and testable
before building against them. It will be updated in place (not silently
diverged from) if implementation at M5+ reveals a problem with it — any
such change should be a deliberate, noted edit here, cross-referenced from
the commit that made it necessary.
