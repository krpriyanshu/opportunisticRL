# Project Plan

This is the working plan for building the repository described in the
master ticket. It exists so that milestone sequencing, entry criteria, and
deferred decisions are written down once rather than re-derived. See
[research-question.md](research-question.md) for *why*,
[architecture.md](architecture.md) for the system design, and
[rl-formulation.md](rl-formulation.md) for the RL spec. This document
tracks *what has been decided, what is deferred, and in what order things
get built*.

## Ground rules (Section 51, restated as commitments)

1. Built incrementally, one milestone at a time.
2. Every interface stays testable in isolation.
3. The RL research layer stays runnable without ROS2 (Section 23).
4. Each milestone is validated (its Definition of Done actually
   demonstrated) before the next one starts.
5. Smallest implementation that satisfies the milestone — no speculative
   generality, no empty placeholder modules created "to match a directory
   tree."
6. Technical decisions are recorded in this file or in `architecture.md`,
   not left implicit in code.
7. Physically invalid or research-invalidating assumptions are flagged
   here rather than silently implemented.
8. No simulator/library capability is assumed working until it has
   actually been exercised in this environment.
9. No experiment is described as having run, and no baseline as
   "working," until it has actually been executed and its output
   inspected.
10. No performance numbers are invented.

## Milestone sequencing and status

| Milestone | Deliverable | Status |
|---|---|---|
| M0 | Project bootstrap | **Done.** |
| M1 | Simulation (marine env, vessel, manipulator, target) | **Essentially done.** Gazebo Harmonic + VRX installed, building, and launching cleanly (`scripts/launch_vrx_sim.sh`) — world, WAM-V vessel, full ROS2 state/sensor/thruster topics, wave/wind disturbance topics all verified live. Manipulator (Franka Panda) selected, mounted, spawning correctly, TF verified, vessel→end-effector motion coupling measured. `ros2_control`/`gz_ros2_control` wired up and verified: controllers active, commanded joint-space motion converges to target exactly. Only remaining piece: adapting the MoveIt2 SRDF/planning config for the vessel-mounted (non-world-fixed) base. No dynamic target yet — that's M4. See `docs/environment.md`, `docs/implementation-status.md`. |
| M2 | ROS2 state pipeline (TF2, robot/target/vessel/relative state) | **Done.** `marine_state_estimator` package publishes vessel odometry, filtered arm joint state, and vessel-relative end-effector pose ("relative state" — no target exists yet, so target-relative state is deferred to M3/M4 as planned). TF2 tree fully mapped, `docs/ros2.md` written. See `docs/ros2.md`, `docs/implementation-status.md`. |
| M3 | Conventional grasping (static target) | **Done.** MoveIt2 SRDF fixed (closes the M1 leftover), `move_group` launches, IK+plan+execute+scripted-attach pipeline validated end-to-end against the live sim — grasp confirmed rigid via a second independent motion, not just proximity. Target placement is deliberately world-fixed for this milestone; vessel-relative placement is M4's job. See `docs/implementation-status.md`, `docs/environment.md` gotcha #9. |
| M4 | Dynamic grasping environment (motion + disturbance + episodes) | **Done (fast backend).** `src/opportunity_rl/sim/` implements the frozen `SimulationInterface` (as a real `typing.Protocol`) via `FastSimulator`: pure-Python Gerstner/Pierson-Moskowitz vessel disturbance ported from VRX's own `Wavefield.cc`, an independently-moving dynamic target, a simplified reachable-workspace manipulator proxy, and the Section 17 episode/termination taxonomy + Section 10 feasibility predicate as one shared implementation. Named disturbance configs live in `configs/disturbances/*.yaml`. 39 unit tests (incl. analytic-vs-finite-difference velocity checks that caught two real derivative bugs). The ROS2/Gazebo backend implementing the *same* interface is deferred to when integration/eval needs it (M7+), per architecture.md's two-backend design. See `docs/implementation-status.md`. |
| M5 | RL environment (Gymnasium interface, offline-runnable) | **Done.** `src/opportunity_rl/env/` — `DynamicGraspEnv(gym.Env)` wrapping the M4 backend, fixed normalized observation schema, config-driven five-term reward, and the opportunity-field append slot. Confound-register items #1/#2/#6/#7 realized *structurally* (byte-identical base observations across baselines, one shared reward, shared decision rate, current-state-only estimator), each unit-tested. Env logic tested with numpy alone; Gymnasium wiring tested with the `rl` extra. See `docs/implementation-status.md`. |
| M6 | Standard RL baseline (B2) | **Done.** `src/opportunity_rl/algorithms/` — a small self-contained clipped-PPO+GAE (torch-only, no SB3, per the spec's "small custom PPO" option). Validated learning against the harder `moving_target` config across 3 seeds: post-train return +0.66/+0.68/+0.71 vs random floor −2.6/−2.9, pre-train ~−1.2. Two real problems found by inspecting the trained policy (default config too easy; wave-startup-ramp step-1 exploit) and fixed with a new named config + episode-phase randomization + regression test. Reproducibility metadata written per run. See `docs/implementation-status.md`. |
| M7 | Reactive benchmark (B0, B1, evaluation matrix) | **Done.** B0/B1 heuristics (sharing the feasibility predicate via `env.current_feasibility()`) + `experiments/evaluation_matrix.py`. Matrix on `moving_target`: random −2.66, B0/B1 +0.74, B2 +0.70. **Key finding:** learned reactive (B2) nearly matches perfect reactive (B0), so this task has ~no headroom for opportunity-awareness — an anticipation-required task variant is added at M8/M9 to give H1 a fair test on both. See `docs/implementation-status.md`. |
| M8 | Opportunity mechanism (deterministic estimator) | **Done.** `OpportunityEstimator` (analytic future-opportunity vector, 2nd-order/leakage-safe) + an anticipation-required committed-grasp task mode + a mild/medium/severe severity ladder. See `docs/implementation-status.md`. |
| M9 | Opportunity-aware RL (B4) | **Done.** B4 = B2 + appended opportunity block (confounds structurally enforced + tested). H1 result: at 100k steps B4≈B2 (delta ~+0.015, not consistent across seeds — H1 not clearly supported on final performance); at 20k steps B4 beats B2 by +0.094 on all 3 seeds (**sample-efficiency** benefit — H1 supported in that sense). Honest, nuanced result. See `docs/implementation-status.md`. |
| M10 | Robustness sweep (disturbance, distribution shift, seeds) | **Done (severity sweep).** mild/medium/severe configs + `experiments/learning_curve_h1.py`; H2 (benefit grows with severity) **rejected** — sample-efficiency benefit present at all severities but doesn't scale with difficulty. Distribution-shift split (H3) not done — flagged open. See `docs/evaluation.md`. |
| M11 | Research evaluation (tables, stats, plots, ablations, report) | **Done.** `docs/evaluation.md` + learning-curve figure + paired sign-test stats. H1 **partially supported**: sample efficiency yes (17/18 low-budget wins, p=0.0001), final performance no (8/18, p=0.81). `research-question.md` hypothesis statuses resolved honestly. Oracle/H4 and B3 remain open, marked so. |

Each row will be updated in place as work progresses; this table is the
first thing to check for current project status, and it is the canonical
answer to "what milestone are we on" — not any other doc.

## M0 scope (this milestone)

Per Section 52, M0 delivers: repository structure, Python environment,
ROS2 workspace, dependency configuration, linting, formatting, testing,
and basic CI — validated by "clean checkout → install deps → build ROS2
workspace → run tests → all pass."

**Decision: the ROS2 workspace at M0 is structurally present
(`ros2_ws/`) but contains no packages yet.** Section 51.5 says not to
create empty placeholder modules just to satisfy a directory tree; the
five ROS2 packages named in Section 31
(`marine_sim_bringup`, `marine_state_estimator`, `marine_rl_policy`,
`marine_manipulation`, `marine_interfaces`) each have a real purpose tied
to a specific later milestone (M1–M3) and no content worth shipping yet.
Creating them empty now would be exactly the placeholder-module anti-
pattern the ticket warns against. `colcon build` on an empty workspace
succeeds trivially (0 packages), which is enough to validate the ROS2
toolchain is installed and working; each package is created, with real
content, at the milestone that needs it.

**Decision: same reasoning applies to `configs/`, `experiments/`,
`scripts/`, and `results/`.** These appear empty in git regardless (git
does not track empty directories), so they are created starting at the
milestone that first writes a real file into them: `configs/` at M4
(first disturbance config), `experiments/`/`scripts/` at M6 (first
training run), `results/` after the first run actually produces output.

**Decision: `docs/environment.md`, `docs/ros2.md`, `docs/experiments.md`,
`docs/evaluation.md` are written at the milestone whose content they
document**, not stubbed now: `environment.md` at M1 (once the simulator
choice is actually exercised), `ros2.md` at M2 (once the TF2 tree and
topics exist), `experiments.md` at M6 (once there's a training pipeline to
describe), `evaluation.md` at M11 (once evaluation scenarios E0–E5 are
implemented). `docs/research-log.md` is created at M6 when the first
trainable baseline produces the first loggable experiment.

**Decision: Python dependency baseline stays minimal at M0.** No RL or
robotics libraries (`torch`, `gymnasium`, `numpy`) are added as required
dependencies yet — there is no code that needs them. They are declared as
an optional `rl` extra now (so the milestone that needs them just
activates the extra) but not installed as part of M0's validation. M0's
Python validation is: package installs, imports, lints, and its one smoke
test passes.

**Decision: license is MIT.** No stated constraint from the ticket or the
user; MIT is the common default for research/academic prototype code and
is trivially changeable later (single `LICENSE` file, one `pyproject.toml`
field) — treated as a low-cost, reversible bootstrap choice rather than
something requiring sign-off.

## M1 progress log

M1 requires installing a Gazebo version and a maritime simulation stack.
This environment started with **no Gazebo of any kind installed** and no
VRX. Per `architecture.md`'s simulator-selection section, the leading
candidate was Gazebo Harmonic + VRX (matching the installed ROS2 Jazzy).

Installing it meant `apt`-level package installation (elevated
privileges) and a nontrivial download — a system-modifying action that
was checked in on with the user rather than performed autonomously, per
this project's operating instructions (this agent's shell also has no
interactive TTY, so `sudo` isn't runnable from here regardless — every
privileged step was relayed to the user as an exact command).

**Result: Gazebo Harmonic (8.11.0) and VRX (`jazzy` branch) are both
installed and building successfully.** Full details, exact commands, and
every gotcha hit along the way (GPG key issues, a conda/CMake Python
interpreter conflict, a `pkill -f` self-matching footgun, missing/
undeclared VRX and Gazebo Python-binding dependencies) are in
`docs/environment.md` — not duplicated here.

Launch-verifying the `sydney_regatta` world initially looked blocked:
its 123 MiB terrain model, fetched from S3 via Gazebo's Fuel asset
service, appeared to stall indefinitely at ~7% downloaded. A longer,
user-requested observation window showed this was actually a slow,
bursty network transfer (long stalls, then fast catch-up bursts), not a
hard proxy block — the download completed and the world initialized
successfully. Gazebo caches the asset locally afterward, so this cost
was one-time. Separately, a real bug was found and fixed: VRX's
`wamv_description`/`wamv_gazebo` packages register an incomplete
`GZ_SIM_RESOURCE_PATH`, causing the vessel's GPS/engine/propeller meshes
to fail to load; fixed by appending the correct paths, now baked into
`scripts/launch_vrx_sim.sh`. Full details, including two captured visuals
of the running simulation, in `docs/environment.md`.

Remaining open item for M1: no manipulator is mounted on the WAM-V yet
(the manipulator URDF decision below is still unresolved). M2 (ROS2
state pipeline) is already substantially covered by what VRX/`ros_gz`
publish out of the box (`/tf`, `/wamv/pose`, per-sensor topics) —
worth re-checking against M2's actual requirements before assuming new
work is needed there.

## M2 progress log

The M1-era note above ("already substantially covered by what VRX
publishes out of the box") turned out to be **half right**: joint states
and per-sensor topics were indeed already usable directly. The other
half was wrong in an important way, discovered only by actually checking
rather than assuming — **`/wamv/pose` does not contain the vessel's
world-frame pose** (it carries sensor-mount statics only), and **neither
TF2 nor any other ROS2-native mechanism exposes it either**. VRX ships a
correctly-configured `OdometryPublisher` Gazebo plugin for exactly this,
but it silently fails to publish in this environment for reasons not
fully diagnosed. This forced genuine new work: a `gz topic`-polling
workaround in the new `marine_state_estimator` package, documented in
full in `docs/ros2.md` and as gotcha #8 in `docs/environment.md`. Lesson
for future milestones: "X is probably already published somewhere" is a
hypothesis to check via `ros2 topic echo`, not a planning assumption to
build on directly, even when it turns out to be half true.

`marine_state_estimator` also settles the "relative state" part of M2's
name: it's the end effector's pose expressed in the vessel's own
(non-inertial) frame, not anything target-relative — there is no target
yet (M3/M4), so target-relative state is correctly deferred, not
prematurely stubbed.

## Open technical decisions carried forward

These are named in `architecture.md`/`rl-formulation.md` and repeated
here as a single checklist of things not yet finalized:

- [ ] Gazebo version + VRX compatibility with ROS2 Jazzy, in this
      environment specifically (M1).
- [ ] Manipulator URDF (custom minimal arm vs. adapting an existing
      open-source arm description) (M1/M3).
- [ ] Exact observation vector dimensionality (depends on the above) (M5).
- [ ] PPO vs. SAC for B2, confirmed once actually implemented (M6).
- [ ] Whether B3 (temporal RL) is justified by evidence of temporal
      dependence in the M6/M7 data — not built speculatively (M7/M8).
- [ ] Number of seeds for the final statistical evaluation, chosen based
      on measured variance and training cost once training is running,
      not fixed in advance (M10/M11).

## Non-goals reminder

Restated from Section 3 so it isn't lost in implementation detail: no
novel low-level controller, no novel IK, no custom physics engine, no
end-to-end camera-to-torque RL, no multi-agent RL, no sim-to-real,
no hardware integration, no custom visualization app. These may become
future directions but are not part of this plan.
