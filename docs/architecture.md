# Architecture

This document describes the layered system architecture, the boundary
between languages/frameworks, and — critically — the confound register that
must be checked before any comparison between baselines is reported as
research evidence. See [research-question.md](research-question.md) for the
question this architecture exists to let us answer, and
[project-plan.md](project-plan.md) for how it will be built up milestone by
milestone.

## Layered view

```text
┌───────────────────────────────────────────────┐
│              RESEARCH / RL LAYER               │
│  reactive RL · temporal RL · opportunity-aware │
│  src/opportunity_rl/{envs,policies,algorithms, │
│  opportunity,rewards,evaluation}               │
└───────────────────────┬───────────────────────┘
                         │  DynamicGraspEnv (Gymnasium API)
┌───────────────────────▼───────────────────────┐
│             TASK / ENVIRONMENT LAYER           │
│  dynamic target · vessel motion · disturbance  │
│  grasp opportunity · reward · termination      │
│  src/opportunity_rl/{envs,disturbances}        │
└───────────────────────┬───────────────────────┘
                         │  SimulationInterface (Section 21)
┌───────────────────────▼───────────────────────┐
│              ROBOTICS / ROS2 LAYER             │
│  state estimation · TF2 · manipulation ·       │
│  controller · policy node                      │
│  ros2_ws/src/{marine_state_estimator,          │
│  marine_rl_policy,marine_manipulation}         │
└───────────────────────┬───────────────────────┘
                         │
┌───────────────────────▼───────────────────────┐
│              SIMULATION / PHYSICS LAYER        │
│  Gazebo / marine environment · vessel dynamics │
│  robot · target · sensors                      │
│  ros2_ws/src/marine_sim_bringup                │
└───────────────────────────────────────────────┘
```

Each layer only depends on the layer directly below it through the
narrow interface named on the connecting arrow. In particular, the RL
layer never imports ROS2 or simulator-specific types — it depends only on
`DynamicGraspEnv`, a plain [Gymnasium](https://gymnasium.farama.org/)
environment. This is what Section 23 calls the "offline RL environment"
requirement, and it is the single most important structural decision in
this codebase: **training must be able to run without ROS2 or Gazebo in
the loop.** ROS2/Gazebo are one *implementation* of `SimulationInterface`,
used for integration testing and evaluation under the real state-estimation
and control stack, not for every training step.

```text
                     ┌────────────────────┐
                     │   DynamicGraspEnv   │
                     │  (Gymnasium API)     │
                     └──────────┬──────────┘
                                │  SimulationInterface
                 ┌──────────────┴──────────────┐
                 │                              │
        ┌────────▼────────┐          ┌──────────▼──────────┐
        │  Fast kinematic   │          │  ROS2 + Gazebo       │
        │  simulator         │          │  execution           │
        │  (training path,   │          │  (integration /      │
        │  pure Python)      │          │  evaluation path)    │
        └────────────────────┘          └──────────────────────┘
```

Both implementations satisfy the exact same `SimulationInterface`
(`reset`, `step`, `get_robot_state`, `get_target_state`,
`get_environment_state`, `apply_action`), so `DynamicGraspEnv` code is
identical regardless of which backend is plugged in. This lets us validate
the RL research question quickly on the fast backend, then confirm
consistency on the ROS2/Gazebo backend, without duplicating environment,
reward, or opportunity logic. Reward and opportunity computation live in
the task layer, never inside either `SimulationInterface` implementation —
per Section 22, the simulator must never contain RL-specific reward logic.

## Python / C++ and ROS2 / MoveIt2 boundary

Python is used for everything in the RL and task layers, and for the
ROS2 nodes in the robotics layer that do not need hard real-time
performance (state estimator, RL policy node, manipulation manager).
C++ is used only where ROS2/simulator integration genuinely benefits —
e.g. if a future controller loop needs tighter timing than `rclpy` can
reliably provide. No C++ exists yet, and none is planned before that
concrete need arises (Section 32).

MoveIt2 and existing ROS2 controller packages are used for trajectory
generation, IK, and low-level control (Section 7). No novel low-level
grasp controller, IK solver, or physics engine will be implemented in this
repository (Section 3, "explicitly out of scope"). The RL policy's action
space stops at the high-level decision (`WAIT` / `APPROACH` /
`ATTEMPT_GRASP` / `ABORT`, Section 6) and hands off to
`marine_manipulation`, which is conventional robotics code.

## Confound register

This is a living checklist, not prose. Section 38 and 39 of the project
ticket name specific ways an experiment comparing baselines can produce a
result that looks like evidence but isn't. Every item must be either
**resolved** (with a note on how) or **explicitly accepted and documented**
before a result from that comparison is used to update
[research-question.md](research-question.md). Do not silently work around
a confound by changing the comparison without recording it here.

| # | Confound | Status | Notes |
|---|----------|--------|-------|
| 1 | Opportunity-aware policy receives more *unrelated* state than reactive policy | Open — resolves at M8/M9 | The two policies must differ only in the opportunity signal itself, not in incidental extra observation fields. Enforced by construction: both read from the same `Observation` schema (Section 8); the opportunity-aware policy adds exactly the `OpportunityEstimator` output on top. |
| 2 | Different action frequencies between policies | Open — resolves at M5 | All policies compared under H1 must be stepped at the same decision rate. Will be fixed as a single `decision_period` env config value shared across baselines. |
| 3 | Different controller capabilities between methods | Open — resolves at M3/M7 | All methods hand off to the same `marine_manipulation` stack (Section 7). No baseline gets a different or enhanced controller. |
| 4 | Different training disturbance distributions | Open — resolves at M9/M10 | Enforced via shared `configs/disturbances/*.yaml` files referenced by name from each experiment config, not redefined per-experiment. |
| 5 | Different compute budgets per algorithm | Open — resolves at M9 | Total environment steps (not wall-clock) will be matched across compared runs and recorded in experiment metadata. |
| 6 | Reward differs between baselines | Open — resolves at M6/M9 | One shared reward function (Section 16) parameterized by config, not reimplemented per baseline. |
| 7 | Opportunity estimator leaks future ground truth into the online policy | Open — resolves at M8 | The deterministic opportunity estimator (Section 10) and any learned successor may only consume information available at decision time (current state + history + short-horizon *estimate*, never the simulator's true future trajectory). The oracle baseline (Section 27) is the only component permitted to read true future state, and it is never used as a policy input — only as an evaluation reference. See Section 39. |
| 8 | Simulator state exposed to one method but not another | Open — resolves at M5 | All policies read observations exclusively through the same `Observation` schema produced by `DynamicGraspEnv`; no baseline gets direct simulator/backend access. |

"Open — resolves at M*" means the milestone whose Definition of Done
should make the confound structurally impossible to reintroduce (e.g. by
having only one reward implementation rather than one-per-baseline). This
table will be revisited at the start of every milestone from M5 onward and
updated in place; entries are never deleted, only marked resolved with a
dated note, so the history of how each confound was actually addressed
stays visible.

## Simulator selection (Milestone M1 decision, recorded now per Section 51.7)

The ticket's assumption suggests ROS2-compatible Gazebo with a maritime
environment such as VRX. This has been checked against the actual
environment this repository is being built in, not assumed:

- ROS2 **Jazzy Jalisco** is installed (`/opt/ros/jazzy`), which targets
  Ubuntu 24.04 and pairs with **Gazebo Harmonic** (`gz-sim`), not Gazebo
  Classic (EOL, unsupported on Jazzy).
- **Verified 2026-08-10: Gazebo Harmonic is installed and working.**
  `sudo apt-get install ros-jazzy-ros-gz` (run by the user, since it
  requires elevated privileges this agent does not have interactive
  access to) pulled in ROS Jazzy's *vendor* packages
  (`ros-jazzy-gz-sim-vendor` et al.), which build/ship Gazebo Harmonic
  inside the ROS install prefix rather than via the separate OSRF apt
  repo. Confirmed with `gz sim --version` → `Gazebo Sim, version 8.11.0`
  (gz-sim8 = the Harmonic release train) once `/opt/ros/jazzy/setup.bash`
  is sourced (it sets `PATH`/`GZ_CONFIG_PATH`; the `gz` binary itself
  lives at `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` and is not on PATH
  otherwise). The separate OSRF `packages.osrfoundation.org` apt repo was
  also added during install but turned out to not be strictly necessary —
  left in place as it's harmless and may be needed for optional gz
  packages later.
- **VRX (Virtual RobotX)** is the leading candidate for the maritime/USV
  environment, as it is the standard ROS2 + Gazebo maritime simulation
  stack and provides a USV platform (WAM-V) plus wave/current disturbance
  models that match Section 14 directly. It has no apt package (confirmed
  via `apt-cache search vrx` — empty) and must be built from source. Its
  exact compatibility with Gazebo Harmonic + ROS2 Jazzy in this specific
  environment (build success, runtime behavior) is being verified now as
  the next concrete M1 step; result recorded in `docs/environment.md`.
- VRX does not ship a manipulator. Mounting a manipulator on the WAM-V
  hull and choosing that manipulator's URDF is a Milestone M1/M3 decision,
  currently leaning toward a small custom-authored URDF (a few-DOF arm
  sized to the WAM-V deck) rather than importing a full industrial-arm
  package, to avoid pulling in hardware-specific drivers and complexity
  that this project does not need (Section 3: no novel low-level
  controller, but also no reason to import one designed for unrelated
  hardware). This is not finalized; it will be finalized once M1's
  simulation environment exists and the deck geometry is known.

Milestone M0 (bootstrap) did not require a simulator — it validated the
Python/ROS2 tooling only. M1 is now attempting the VRX build; if it is
infeasible in this environment (network restrictions, package conflicts,
resource limits), that failure and the fallback chosen will be documented
here rather than silently substituted. Candidate fallbacks, in order, if
VRX is infeasible: (a) a hand-built minimal Gazebo Harmonic world with a
manually-authored buoyant vessel plugin and a simplified wave/current
disturbance model applied directly as forces (more implementation work,
full control, no dependency on VRX's release cadence), (b) an older
Gazebo/VRX combination pinned to whatever ROS2 distribution it actually
supports, run instead of Jazzy if necessary (would require reinstalling
ROS2, a larger and more disruptive change).

Because this is a system-level install (`apt`, elevated privileges,
multi-GB download, and — in the fallback case — a possible ROS2
distribution change), it will not be performed autonomously without
checking in first; see `docs/project-plan.md` for the M1 entry criteria.

**Wave-model literature review.** A rigorous, paper-by-paper survey of
how other marine-robotics simulators and papers actually model ocean
waves — including a precise, source-verified account of what this
project's own installed VRX wave model does (Pierson-Moskowitz, not
JONSWAP as some secondary sources claim) — is in
[docs/wave-simulation-literature-review.md](wave-simulation-literature-review.md).
Directly informs the disturbance-severity knobs available for M4.

## Coordinate and unit conventions (finalized at M2)

Full TF2 frame tree, coordinate conventions, units, and the
`marine_state_estimator` topic contract are documented in
[docs/ros2.md](ros2.md). Headline finding worth surfacing here since it
affects every layer above the ROS2 one: **this simulation has no
world-fixed TF2 frame at all** — `wamv/wamv/base_link` is the TF root,
and the vessel's world-frame pose is only obtainable via a Gazebo
Transport workaround (see `docs/ros2.md`), not standard ROS2 TF2. The
vessel-*relative* frame (not world) is therefore the practically load-bearing
reference frame for this project — matches `rl-formulation.md`'s
`ee_pose (7) ... vessel-base frame` observation field, which was
specified before this was confirmed and turned out to be the right call.
