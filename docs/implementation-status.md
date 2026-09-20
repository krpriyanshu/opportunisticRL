# Validation Record

This document records what was built at each milestone, what was verified, how it was verified, and what remains unverified. It is organised by milestone and is the appropriate reference for assessing the evidential basis of any claim about the system's behaviour. Its governing convention is that an item is recorded as verified only when it was *observed* working, not when the corresponding code was written.

Claims about research outcomes are not made here; see [evaluation.md](evaluation.md). Divergences between the RL specification and the implementation are collected in [rl-formulation.md §9](rl-formulation.md#9-divergences-from-specification-collected).

## 1. Summary of unverified and unbuilt items

Collected here so that the limitations are legible without reading the whole record:

1. **No ROS2-level automated integration tests.** Validation of the ROS2 packages was empirical against a live simulation. Their ROS2-free components are unit-tested, but no `launch_testing` suite exists.
2. **The two backends were never cross-validated.** Only the fast backend was implemented, so no quantitative agreement between it and the ROS2/Gazebo path has been established.
3. **Collision termination never fires.** The fast backend has no collision model; the pathway is wired and inert.
4. **No policy was trained or evaluated through ROS2/Gazebo.**
5. **The reported results predate four correctness fixes** and are not reproducible from their metadata; see [evaluation.md §2](evaluation.md#2-provenance-and-status-of-the-evidence).

## 2. Manipulator selection

**Selected: Franka Emika Panda**, via `moveit_resources_panda_description` — the description package MoveIt2 maintains for its own tutorials and continuous integration.

The criteria and how the choice met them:

1. **Degrees of freedom.** 7 DOF, satisfying the 6+ requirement.
2. **Version compatibility.** Released alongside MoveIt2 itself, so it matches the current MoveIt2 release by construction rather than by community best-effort porting.
3. **Ready configuration.** URDF, SRDF and a MoveIt2 configuration covering kinematics, controllers and planning ship with it, so only remounting was required.
4. **Mountability on a non-inertial base.** Its base link is not welded to a world frame inside the URDF; the fixed-to-world virtual joint lives in the MoveIt2 SRDF instead. The raw description therefore mounts onto a vessel frame without editing upstream files, simply by not reusing that virtual joint.
5. **No hardware-driver dependencies**, unlike the rejected alternative.

**Rejected: Universal Robots** (`ur_description`, UR5e/UR10e) — equally mature and Jazzy-compatible at 6 DOF, but oriented toward real-hardware driver integration that this project has no use for. Retained as a documented fallback.

The end effector is Panda's own two-finger gripper. Grasp success is determined by a deterministic distance, velocity and orientation check plus a scripted attach, not by physical finger-contact dynamics; the gripper geometry is retained for visual and collision plausibility but does not perform the grasp.

## 3. M1 — Simulation environment

**Added:** `ros2_ws/moveit_resources.repos` (vcstool pin) and `ros2_ws/src/wamv_manipulator_description`, a package whose top-level xacro includes both the WAM-V and Panda descriptions unmodified and adds exactly one fixed joint mounting the arm on the vessel deck, together with the `gz_ros2_control` wiring.

**Reused unmodified:** VRX's WAM-V description and Gazebo integration, and MoveIt2's Panda description, both mounted via a new package rather than by editing upstream sources.

Verified:

- The Panda description builds standalone and has no dependency on MoveIt2 itself.
- The combined xacro parses to 44 links and 43 joints, with all 11 Panda links present and **zero name collisions** against WAM-V's links.
- The model spawns through VRX's existing, unmodified launch pipeline; `/wamv/joint_states` publishes all 9 Panda joints alongside the vessel's own.
- The TF tree resolves and updates continuously from the vessel base link to the end effector, confirmed by `tf2_echo` rather than by inspecting the xacro.
- Vessel motion propagates to the end effector, measured over 5 samples across 45 wall-clock seconds: vessel world-frame z span 0.16 m, end-effector z span 0.17 m and x span 0.32 m. **This is a weak check and is recorded as such** — the mount is a zero-DOF fixed joint and no controller was holding the arm, so the arm's gravity-driven settling and the vessel's bobbing are not separated by this measurement.
- `ros2_control` and `gz_ros2_control` load, configure and activate `joint_state_broadcaster` and `panda_arm_controller`.
- Commanded motion converges numerically, not merely visually: a single-point trajectory to a 7-joint target produced smooth interpolation and convergence to **within 1e-9 rad on every joint**.

Two pre-existing DART limitations were observed and are not introduced by this work: mesh-based collision geometry is not built, and mimic joints are not enforced. Neither blocked M1; both matter once real grasp contact is required.

**Frame-naming note.** `robot_state_publisher` runs in the `/wamv` namespace, prefixing every frame with `wamv/`, while WAM-V's own xacro already bakes `wamv` into its link names. WAM-V links are therefore double-prefixed (`wamv/wamv/base_link`) and Panda links singly prefixed. This is a pre-existing VRX quirk, confirmed present before this integration.

## 4. M2 — ROS2 state pipeline

**Added:** `ros2_ws/src/marine_state_estimator`. The design and the empirical findings behind it are in [ros2.md](ros2.md).

Verified:

- The TF2 tree was mapped by direct inspection of `/tf_static`, confirming `wamv/wamv/base_link` is the root with no world-connected parent.
- Three ROS2-side candidates for vessel world pose were checked and ruled out empirically before a workaround was adopted: TF2, `/wamv/pose`, and VRX's own `OdometryPublisher` plugin, which is correctly configured but silently publishes nothing in this environment.
- The three pure-Python components are covered by 11 unit tests requiring no ROS2.
- All three topics publish live, plausible data. Vessel position and end-effector position were cross-checked against independently obtained `tf2_echo` values; vessel linear velocity was confirmed to oscillate smoothly through positive and negative values, consistent with wave bobbing, rather than being stuck at zero or diverging.

**A real defect was found during this validation and fixed.** The first implementation republished the last-known vessel pose at the full timer rate regardless of whether the background poller had produced fresh data. Because the poller refreshes only every 130–350 ms, most ticks computed a near-zero finite-difference velocity from a duplicate sample. The fix detects and skips stale samples, and was confirmed by resampling across 8 consecutive intervals and observing genuine variation.

**Not done:** no automated ROS2-level integration test.

## 5. M3 — Conventional grasping against a static target

**Added:** `wamv_manipulator_moveit_config` and `marine_manipulation`.

Verified:

- The SRDF virtual joint was corrected from a free-floating world base to a fixed joint parented to the vessel base link, matching the URDF's already-specified kinematics.
- `move_group` launches against the live simulation with `use_sim_time` set.
- `/compute_ik` returns a physically plausible joint solution for a vessel-relative Cartesian target, confirmed by direct service call before being relied upon.
- The grasp pipeline reads the target's *current* vessel-relative position rather than a spawn-time value, solves IK, plans and executes through MoveIt2, then commands a scripted attach.
- **Grasp rigidity was confirmed rather than inferred.** After attachment, a second independently commanded arm motion was executed and the target's world position moved by a comparable distance in the same direction, ruling out coincidental proximity. This is the strongest validation standard applied at any milestone.
- The vessel-relative geometry computation is unit-tested independently of the live simulation.

**Deliberate scope simplification, recorded not hidden:** the target is world-fixed in position rather than resting on or attached to the disturbed vessel. Realistic target placement under vessel motion is M4's concern.

## 6. M4 — Dynamic grasping environment, fast backend

**Added:** `src/opportunity_rl/sim/` and `configs/disturbances/`.

Verified:

- `SimulationInterface` was promoted from a documentation-only specification to a real `typing.Protocol`, so conformance is checked structurally by mypy rather than asserted.
- The wave disturbance is a from-scratch pure-Python port of VRX's trochoidal (Gerstner) multi-component model, using the same model selection and parameter names as VRX's SDF `<wave>` element, cross-checked against the source-grounded account in [wave-simulation-literature-review.md](wave-simulation-literature-review.md). Because it solves only the forward problem for a single floating body, it yields exact analytic velocity — an advantage over the ROS2 path's finite-difference workaround. **Stated caveat:** the height term is verified exactly against VRX's own routine; the steepness-scaled horizontal term is the standard Gerstner formula and is not claimed bit-identical to VRX's internal Newton solver.
- The dynamic target is built from the same tested primitive as the vessel and is never conditioned on the vessel's motion, as the research question requires.
- The manipulator proxy is explicitly a reachable-workspace bound with a velocity-limited approach, not a 7-DOF joint simulation; its joint values are documented as fixed proxy quantities.
- Reward and success judgement are kept out of the simulator, which tracks only the physical fact of holding the target. Verified by construction: the simulator imports no reward code.
- An unknown disturbance configuration name fails loudly rather than falling back to defaults, so a run's configuration claim remains trustworthy.
- 39 unit tests cover the module. **Two of them caught genuine derivative defects** during development — a missing product-rule term from the time-varying wave startup ramp, and a missing Doppler term from the drifting target's rest position moving through the wave field — both fixed and re-verified to 1e-6 tolerance.

## 7. M5 — RL environment

**Added:** `src/opportunity_rl/env/`.

Verified:

- `DynamicGraspEnv` exposes `Discrete(4)` actions and a `Box` observation whose dimension is computed from the schema rather than hard-coded.
- **Confound 1 is structural**: the opportunity signal is a separately appended block, and a reactive and an opportunity-aware environment produce byte-identical base observations, asserted directly by unit test. The question "what does the aware policy see that the reactive one does not" is answered exactly by `obs[BASE_DIM:]`.
- **Confound 6**: one shared, config-driven reward returning each term separately for failure analysis.
- **Confound 2**: a single decision-period value by which the environment advances the simulator each step.
- **Confound 7**: the estimator is called with current state only; the environment holds no future state, so leakage is impossible rather than merely prohibited.
- The binding requirement that indefinite waiting must not be optimal is unit-tested: an all-`WAIT` 100-step return is strictly less than a single successful grasp's return.
- Gymnasium API conformance is tested, and skipped cleanly when the `rl` extra is absent so the numpy-only logic tests still run.

**Known inert pathway:** the collision flag is always false, because the fast backend has no collision model.

## 8. M6 — Standard RL baseline (B2)

**Added:** `src/opportunity_rl/algorithms/` and `experiments/train_b2.py`. B2 is a self-contained clipped PPO with GAE depending only on torch. PPO was confirmed over SAC on the grounds that the action space is discrete.

Validation result on the `moving_target` configuration at 100k steps, evaluated on held-out seeds:

| Seed | Random return | PPO pre-training | PPO post-training |
|---|---|---|---|
| 0 | −2.85 | −1.21 | **+0.71** |
| 1 | −2.56 | −1.23 | **+0.68** |
| 2 | −2.57 | −1.23 | **+0.66** |

Two substantive problems were identified by inspecting the trained policy's behaviour rather than by unit tests:

1. **The default disturbance configuration was too easy.** The target was measured graspable 100% of the time, making timing irrelevant, so a random policy tied PPO. A harder named configuration (approximately 34% graspable) was added, which also became the first rung of the severity ladder.
2. **A startup-ramp exploit.** A trained policy achieved a mean episode length of 1.0 — grasping on step 1 of every episode, because the wave startup ramp made t = 0 a guaranteed zero-velocity opportunity regardless of configuration. Fixed by randomising each episode's start phase past the ramp, with a regression test. Afterwards, episode length rose to approximately 27 and the random policy's return fell to approximately −2.7.

**Metric finding, which shaped the eventual evaluation design:** success rate saturates at 1.0 for both random and trained policies, because episodes are long enough that even random action eventually grasps. Return is the discriminating metric. This was recorded at M6 rather than discovered during final analysis.

## 9. M7 — Reactive benchmarks and evaluation matrix

**Added:** B0 and B1 heuristics acting on the single shared feasibility predicate rather than reimplementing it, plus `experiments/evaluation_matrix.py`. B1 with default thresholds provably reduces to B0, which is unit-tested.

Matrix result on `moving_target`, 3 seeds, 200 evaluation episodes:

| Baseline | Return (mean ± std) | Success | Episode length |
|---|---|---|---|
| Random | −2.66 ± 0.17 | 1.000 | 31.5 |
| B0 | **+0.74 ± 0.00** | 1.000 | 25.9 |
| B1 | +0.74 ± 0.00 | 1.000 | 25.9 |
| B2 (PPO) | +0.70 ± 0.00 | 1.000 | 27.6 |

**This was the load-bearing finding for the experimental design.** Learned reactive B2 at +0.70 had essentially closed the gap to perfect reactive B0 at +0.74, meaning the task left almost no headroom in which opportunity-awareness could demonstrate value. The diagnosed cause is that the grasp succeeds whenever the target is within a large workspace radius and moving slowly, and the end effector at standby is already within that radius, so `APPROACH` is effectively a no-op and there is no approach latency for anticipation to conceal.

**Consequence, acted upon:** a fair test of H1 requires a task in which reaction is fundamentally insufficient — where grasping requires the end effector to have approached, and approaching takes time. This anticipation-required variant was built at M8. H1 was then to be tested on both the original task, where little benefit was expected, and the anticipation task.

## 10. M8 — Opportunity mechanism and anticipation-required task

**Added:** `OpportunityEstimator`, a committed multi-step grasp mode, and the severity ladder.

Verified:

- The estimator emits a future-opportunity vector over four horizons, each element a smooth analogue of the feasibility predicate at an extrapolated state.
- **Leakage safety is structural**: it reads only current relative position and velocity plus one step of past velocity for a finite-difference acceleration, never the simulator's true future. It is stateful and is reset each episode, verified by unit test.
- **Second-order extrapolation is necessary, not decorative.** Good windows arise from the target decelerating at wave turning points, to which a constant-velocity predictor is structurally blind. A unit test confirms the second-order estimator's peak future opportunity exceeds the first-order's by more than 0.2 on a decelerating approach.
- The committed-grasp mode makes a grasp a multi-step manoeuvre that fails if the opportunity lapses before completion. The measured window-length distribution on `moving_target` is highly variable — median 2 steps, maximum 36, with **31% of windows shorter than a 3-step manoeuvre** — so committing to a grasp the window cannot sustain is a real and avoidable error.
- Severity ladder measured good-opportunity fractions: 0.55 (mild), 0.33 (medium), 0.21 (severe).

## 11. M9–M11 — The H1 comparison, severity sweep, and evaluation

B4 is B2's PPO trained against an opportunity-aware environment, differing only by the appended block. No new algorithm was introduced. Confound 1 is enforced by constructing both environments with identical arguments differing only in the estimator, and is unit-tested by asserting byte-identical base observations across steps.

The severity sweep ran 3 severities × 3 seeds × 2 policies × 4 budgets, with a second run extending to 8 seeds. The statistical analysis, findings, and threats to validity are reported in [evaluation.md](evaluation.md) and are not duplicated here. Two points belong in this record because they concern validation rather than results:

1. **The scientific-integrity guards held.** Confounds 1, 2, 6 and 7 are structural and tested; training and evaluation seeds never overlap; every run wrote reproducibility metadata.
2. **The metadata was nonetheless insufficient.** The provenance helper recorded `"git_commit": "HEAD"` because it did not check git's return code, so the guard that was supposed to make runs reproducible did not do so. This is the clearest lesson in this record: a validation mechanism that is never itself tested can fail silently while appearing to function.

**Not built**, and recorded rather than silently skipped: the oracle timing-regret reference (H4), the distribution-shift split (H3), temporal RL (B3), and any training against the ROS2/Gazebo backend.
