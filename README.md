# Opportunity-Aware RL for Dynamic Robotic Grasping in a Disturbed Marine Environment

**Status: Milestones M0–M11 built and validated on the fast backend —
the research question has an executed, honestly-reported answer.** The
full stack runs end-to-end: a Gerstner/Pierson-Moskowitz marine
disturbance ported from VRX, a dynamic target, a Gymnasium
`DynamicGraspEnv`, the full baseline ladder (B0/B1 heuristics, B2 reactive
PPO, B4 opportunity-aware PPO), a leakage-safe analytic opportunity
estimator, and a matched controlled H1 comparison across seeds and
disturbance severities. Earlier milestones (M1–M3) validated the real
ROS2 + Gazebo + VRX + MoveIt2 stack and a rigid scripted grasp.

**The H1 result is nuanced, not a clean win** (and reported as such):
opportunity-awareness makes the timing policy **learn faster** (B4 beats
reactive B2 in 17/18 low-budget paired comparisons, p=0.0001) but **not
end up better** (they converge by 40k steps, p=0.81). Full evidence,
figure, statistics, and threats to validity:
[docs/evaluation.md](docs/evaluation.md). Not done (marked open, not
skipped silently): the oracle timing-regret reference (H4), a
distribution-shift split (H3), temporal RL (B3), and training on the
ROS2/Gazebo backend. See [Current research status](#current-research-status)
below.

**[docs/dashboard.html](docs/dashboard.html)** is a single self-contained
file — open it directly in a browser, no server or build step — with the
milestone tracker, confound register, RL spec, environment telemetry, and
a live in-browser toy simulator of the opportunity-window mechanism, all
on one page. It's a manually-regenerated snapshot, not a live view; open
it fresh after material project changes rather than trusting an old tab.

## 1. Research problem

A robotic manipulator mounted on a wave/current-disturbed vessel must
grasp an independently moving target. Grasp *quality* varies over time —
a target can be approaching a favorable configuration, briefly pass
through one, or be about to leave it. This project asks whether an RL
policy that is given some representation of that temporal structure
(future interaction *opportunity*) makes better grasp-timing decisions
than a policy that only reacts to the current instant.

Full statement, hypotheses, and what would count as support or rejection:
[docs/research-question.md](docs/research-question.md). The novelty
claim has been stress-tested against the closest prior work found:
[docs/related-work-review.md](docs/related-work-review.md).

## 2. Motivation

Most dynamic-grasping RL work either treats timing implicitly (react to
the current state and hope the policy learns to wait) or assumes a static
target. This project isolates the timing question directly by making
"when" a first-class, high-level decision (`WAIT` / `APPROACH` /
`ATTEMPT_GRASP` / `ABORT`) separate from low-level manipulation, and by
building an explicit, swappable representation of future interaction
opportunity to compare against a reactive baseline under matched
conditions. Marine/USV simulation is used because wave- and
current-driven disturbance is a physically natural, configurable source
of exactly this kind of temporal structure — it is supporting
infrastructure, not the research contribution.

## 3. Architecture

Four layers — RL/research, task/environment, robotics/ROS2, and
simulation/physics — each depending only on a narrow interface exposed by
the layer below, so the RL research code can run independently of ROS2
wherever possible. Full description, confound register, and the current
simulator-selection decision: [docs/architecture.md](docs/architecture.md).
RL-specific formalization (observation/action/reward/opportunity spec):
[docs/rl-formulation.md](docs/rl-formulation.md).

```text
RL / research layer        (src/opportunity_rl)
Task / environment layer   (src/opportunity_rl — DynamicGraspEnv)
Robotics / ROS2 layer      (ros2_ws/src — state estimation, TF2, MoveIt2)
Simulation / physics layer (ros2_ws/src/marine_sim_bringup — Gazebo)
```

## 4. Installation

Python research layer (no ROS2 required):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

The `rl` extra (`torch`, `gymnasium`, `numpy`) is not required until
Milestone M5 and is installed separately when needed:

```bash
pip install -e ".[rl]"
```

ROS2 workspace (Jazzy; see [ros2_ws/README.md](ros2_ws/README.md)). The
third-party packages are fetched via vcstool from the pinned refs in
`ros2_ws/*.repos`, not vendored:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install
```

Use **system Python** (not a conda environment) when building/running the
ROS2 workspace — see the note in [ros2_ws/README.md](ros2_ws/README.md)
on why the RL and ROS2 layers intentionally use separate Python
environments.

## 5. Simulation launch

Gazebo Harmonic + VRX (Sydney Regatta harbor world, WAM-V vessel, with a
Franka Panda arm mounted on deck and driven by `ros2_control`) are
installed and verified working (Milestone M1, essentially done). One
command launches it:

```bash
cd ros2_ws && colcon build --symlink-install  # first time only
scripts/launch_vrx_sim.sh headless:=True world:=sydney_regatta \
    urdf:=$(pwd)/install/wamv_manipulator_description/share/wamv_manipulator_description/urdf/wamv_with_arm.urdf.xacro
```

First launch downloads the world's assets (~159 MiB via Gazebo's Fuel
service) and can take several minutes depending on network conditions;
subsequent launches use the local cache. Full setup details, every
environment gotcha hit along the way, and captured visuals of the
running simulation are in
[docs/environment.md](docs/environment.md) — including
[a third-person shot](docs/images/m1_wamv_sydney_regatta.png) of the
WAM-V, [an onboard camera view](docs/images/m1_wamv_onboard_camera.png),
[a short video of the vessel underway](docs/images/m1_wamv_underway.mp4),
and [a video of the mounted arm executing a commanded trajectory](docs/images/m1_panda_commanded_motion.mp4).
Simulator selection rationale: [docs/architecture.md](docs/architecture.md#simulator-selection-milestone-m1-decision-recorded-now-per-section-517).
A rigorous, paper-by-paper review of how this and other marine-robotics
simulators model ocean waves — including exactly what wave spectrum this
project's own VRX install uses, verified from source — is in
[docs/wave-simulation-literature-review.md](docs/wave-simulation-literature-review.md).

## 6. Baseline execution

The full ladder is implemented: B0/B1 (reactive heuristics), B2
(reactive PPO), and B4 (opportunity-aware PPO). B3 (temporal RL) is not
built. Baseline ladder:
[docs/rl-formulation.md](docs/rl-formulation.md#baseline-ladder-section-11).

```bash
python3 experiments/evaluation_matrix.py --disturbance moving_target \
    --grasp-duration-steps 3
```

Pass the same `--grasp-duration-steps` the policies were trained at — the
H1 comparison uses 3, and a matrix run at the default 1 measures a
different task and is not comparable to it.

## 7. RL training

```bash
pip install -e ".[rl]"
python3 experiments/train_b2.py --disturbance moving_target --seed 0
```

Trains B2 against the fast `DynamicGraspEnv`, evaluates against a random
floor on held-out seeds, and writes reproducibility metadata + a
checkpoint under `results/<run>/`. Note: the default
`sydney_regatta_default` disturbance is intentionally easy (target always
graspable, so timing is irrelevant); use `--disturbance moving_target`
for the condition where the timing decision actually matters. Details:
[docs/implementation-status.md](docs/implementation-status.md).

## 8. Evaluation

The H1 controlled evaluation is complete and written up in
[docs/evaluation.md](docs/evaluation.md) (tables, the learning-curve
figure, paired sign-test statistics, and threats to validity). Headline:
opportunity-awareness (B4) beats reactive RL (B2) on sample efficiency
(17/18 low-budget paired wins, p=0.0001) but not on final performance
(8/18, p=0.81). Reproduce the core sweep with (needs both extras — the figure is
matplotlib, which lives in `viz`, not `rl`):

```bash
pip install -e ".[rl,viz]"
python3 experiments/learning_curve_h1.py \
    --disturbances moving_target_mild moving_target moving_target_severe \
    --grasp-duration-steps 3 --seeds 0 1 2
```

## 9. Experiment reproduction

Every training run is reproducible from its recorded metadata: `python3
experiments/train_b2.py` writes `results/<run>/metadata.json` capturing
seed, git commit, the resolved config, total env steps, and all
evaluation results, alongside the model checkpoint. `results/` is
gitignored (artifacts, not source). Training and evaluation never share a
seed (eval uses `seed + 10_000`).

## 10. Current research status

- Milestones **M0–M11 done** on the fast backend. See
  [docs/project-plan.md](docs/project-plan.md) for the full milestone table.
- **H1 is tested, with a nuanced answer** (Section 8): opportunity-awareness
  improves *sample efficiency* (17/18 low-budget paired wins, p=0.0001) but
  not *final performance* (8/18, p=0.81). H2 (does the benefit grow with
  disturbance severity?) has severity-ladder data but no confirmatory
  result. H3 (distribution shift) and H4 (oracle timing regret) are
  **untested** — open, not silently skipped.
- Repository currently contains: the research/architecture/RL docs; a
  working Gazebo Harmonic + VRX simulation with a Panda arm and a
  validated conventional grasp (M1–M3, real ROS2/Gazebo stack); a fast
  pure-Python `SimulationInterface` backend with a source-faithful marine
  wave disturbance and dynamic target (M4); a Gymnasium `DynamicGraspEnv`
  with the frozen observation/reward spec (M5); the full baseline ladder
  (B0/B1 heuristics, B2 reactive PPO, B4 opportunity-aware PPO); a
  leakage-safe analytic `OpportunityEstimator` (M8); and the matched H1
  comparison across seeds and severities (M10/M11). Not built: B3
  (temporal RL), and training against the ROS2/Gazebo backend.
- A useful finding surfaced early at M6: because episodes are long enough
  that even a random policy eventually grasps, **success rate saturates
  and is the wrong discriminating metric** — return/efficiency is what
  separates a good policy from a bad one here. This shaped what the M11
  evaluation measures.
- An engineering audit of long-term experiment-readiness is in
  [docs/engineering-review.md](docs/engineering-review.md).

### Results predating the frame/reproducibility fixes

Everything currently under `results/` was produced before four defects
were fixed (see the repository history): a vessel/world frame mismatch in
the published target state, the same mismatch in the opportunity
estimator, unseeded training-episode resets that made runs
irreproducible, and stale observations carried across an interleaved
train/evaluate boundary. The frame errors were latent (measured to flip
no grasp decision under the shipped configs) and the reported direction
of the H1 effect is not expected to change, **but those numbers were not
reproducible from their recorded metadata and their `git_commit` field
reads `"HEAD"` rather than a revision.** They should be regenerated
before being cited.

## 11. Known limitations

- MoveIt2's planning pipeline (`move_group`) runs for the mounted arm via
  `wamv_manipulator_moveit_config` (whose SRDF replaces the stock
  `world`-fixed virtual joint with a fixed mount on the vessel's base
  link), but the RL policies never use it — they train against the fast
  backend, and the MoveIt2 path is exercised only by M3's scripted grasp.
- DART (Gazebo's physics engine) cannot build collision geometry from
  the Panda's stock mesh-based collision shapes, and does not enforce
  the gripper's mimic joint — both pre-existing DART limitations, not
  introduced by this project, but relevant once real grasp contact is
  implemented (M3). See [docs/implementation-status.md](docs/implementation-status.md).
- Every quantitative claim in Sections 1–2 above is a hypothesis to be
  tested, not a result. This project makes no novelty or performance
  claims beyond what has actually been experimentally demonstrated (see
  [docs/research-question.md](docs/research-question.md)).

## Exploratory visualization (not M4/M5/M8 — see caveat below)

`scripts/visualize_opportunity_concept.py` is a standalone,
numpy+matplotlib-only illustration of the core research question (a
disturbed vessel, an independently drifting target, a "good opportunity"
window, and a reactive-vs-lookahead decision comparison), for exploring
and communicating the problem without standing up Gazebo. It is
explicitly **not** the M4 dynamic-grasping environment, the M5
`DynamicGraspEnv`, or the M8 `OpportunityEstimator` — those will be built
against the frozen interfaces in
[docs/rl-formulation.md](docs/rl-formulation.md) when their milestones
are reached; nothing in this script should be cited as progress on them.

```bash
pip install -e ".[viz]"
python3 scripts/visualize_opportunity_concept.py            # static PNG
python3 scripts/visualize_opportunity_concept.py --video     # + mp4 (needs ffmpeg)
```

## Repository layout

```text
docs/            research question, architecture, RL formulation, project plan
src/opportunity_rl/   RL research layer (Python, ROS2-independent)
ros2_ws/         ROS2 workspace (robotics/simulation layers)
tests/           unit tests for src/opportunity_rl
scripts/         diagnostics + the exploratory visualizer above
configs/         disturbance configs (one YAML per condition, loaded by name)
experiments/     training / comparison / evaluation-matrix scripts
results/         run artifacts (metadata + checkpoints), gitignored
```

## Development

```bash
ruff check .            # lint
ruff format .           # format
mypy src/opportunity_rl # type check
pytest                  # tests
```

**Known environment gotcha:** if your shell sources ROS2's
`setup.bash` on startup (common when `ros2_ws` work is also happening in
the same shell), `PYTHONPATH` will contain ROS2's `site-packages`. That
leaks ROS2's `launch_testing` package into this venv's pytest plugin
autodiscovery, which then fails importing `yaml` (a ROS2-side dependency,
not one of ours) instead of running our tests. Run `unset PYTHONPATH`
before invoking `pytest`/`ruff`/`mypy` in a shell that has ROS2 sourced,
or use a separate shell/tab for `src/opportunity_rl` work that never
sources `/opt/ros/jazzy/setup.bash`. This does not affect CI (the CI
runner has no ROS2 installed) — it is a local dev-shell issue only.

## License

MIT — see [LICENSE](LICENSE).
