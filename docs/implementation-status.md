# Implementation Status

Snapshot taken at the start of implementing the remaining system
end-to-end, per the implementation directive. This file is updated at
each milestone gate, not rewritten from scratch — treat it as a running
log, most recent entry last within each section.

## Discovered existing components (verified by inspection, not assumed)

- **Python layer**: `src/opportunity_rl` — package skeleton only
  (`__version__`, `py.typed`). No environment, algorithm, opportunity, or
  reward code exists yet.
- **ROS2 workspace**: `ros2_ws/src/vrx` (vcs-fetched, gitignored, `jazzy`
  branch) — builds cleanly (5/5 packages: `vrx_gazebo`, `wamv_description`,
  `wamv_gazebo`, `vrx_ros`, `vrx_gz`). No custom packages exist yet
  besides VRX itself.
- **Simulator**: Gazebo Harmonic 8.11.0 (ROS-vendored), VRX `sydney_regatta`
  world verified loading with the WAM-V spawning correctly (mesh-path bug
  found and fixed, documented in `docs/environment.md`). Full ROS2 state
  pipeline confirmed live: `/tf`, `/tf_static`, `/wamv/pose`,
  `/wamv/joint_states`, per-sensor topics, thruster topics, VRX
  wave/wind disturbance topics.
- **Launch**: `scripts/launch_vrx_sim.sh` — one-command sim launch with
  all environment fixes baked in.
- **Base link / TF root for mounting**: `wamv_gazebo/urdf/wamv_gazebo.urdf.xacro`
  includes `wamv_gazebo.xacro`, which defines `${namespace}/base_link`
  (default namespace `wamv`) as the vessel's root link — this is the
  correct, existing attachment point for a manipulator; no reason to
  invent a new one.
- **NOT installed**: MoveIt2, `ros2_control`, `ros2_controllers`,
  `gz_ros2_control` — none of these were needed for M0/M1-sim-only work,
  so they were never requested. Confirmed via `apt-cache policy` (all
  show `Installed: (none)`, `Candidate:` available) — they exist in the
  configured apt repos, just not yet installed. This blocks the
  controller/MoveIt2 portions of M1 (see below).

## Manipulator decision

**Chosen: Franka Emika Panda, via `moveit_resources_panda_description`**
(the description package MoveIt2 itself maintains and uses for its own
tutorials/CI, currently versioned for ROS2 Jazzy in the Debian repos as
`ros-jazzy-moveit-resources-panda-description` /
`ros-jazzy-moveit-resources-panda-moveit-config`, and available at
[github.com/moveit/moveit_resources](https://github.com/moveit/moveit_resources)
on the `ros2` branch).

Rationale, against the directive's criteria:
- 7 DOF (satisfies "6+ DOF").
- ROS2 Jazzy + MoveIt2 2.12.4 compatible by construction — this package
  is versioned and released *alongside* MoveIt2 itself specifically so
  it always matches the current MoveIt2 release, not a community
  best-effort port.
- URDF/SRDF and a ready-made MoveIt2 config (kinematics, controllers,
  planning config) ship out of the box — no MoveIt2 config authoring
  needed from scratch, only remounting.
- Its base link (`panda_link0`) is not welded to a `world`/fixed frame
  inside the URDF itself — the fixed-to-world virtual joint lives in the
  MoveIt2 config's SRDF, separate from the URDF. That means the raw
  description mounts cleanly onto a non-inertial (vessel) frame without
  editing upstream files, by simply not reusing that SRDF virtual joint.
- No vendor hardware driver baggage (unlike `ur_description`/
  `ur_moveit_config`, which pull in real-robot-driver-oriented tooling
  not relevant here and explicitly out of scope per "no hardware
  integration"). Panda's `moveit_resources` package is simulation/
  tutorial-only by design, which matches this project's needs exactly.

Rejected alternative: Universal Robots (`ur_description`/
`ur_moveit_config`, UR5e/UR10e) — also mature and MoveIt2-Jazzy
compatible, 6 DOF, but oriented toward real-hardware driver integration
(`ur_robot_driver`) that this project has no use for and that would add
unnecessary surface area. Kept as a documented fallback if Panda turns
out to be unsuitable for some reason not yet discovered.

End effector: Panda's own two-finger gripper (`panda_hand`,
`panda_leftfinger`/`panda_rightfinger`) ships with the description. Per
directive Section 7, actual grasp success will be a deterministic
distance/velocity/orientation check plus a scripted attach (e.g. a
Gazebo `DetachableJoint`), not physical finger-contact dynamics — the
gripper geometry stays for visual/collision plausibility, but is not the
thing that "does" the grasp. This is an M3-scope decision, not M1.

## Components being reused

- VRX's WAM-V description and Gazebo integration, entirely unmodified.
- MoveIt2's own Panda description/config, entirely unmodified at the
  source level — mounted via a new fixed joint in a new package, not by
  editing upstream files.

## Components being added (this milestone: M1)

- `ros2_ws/moveit_resources.repos` — vcstool pin for
  `moveit_resources` (`ros2` branch), fetched the same way as VRX
  (gitignored, not vendored into our git history).
- `ros2_ws/src/wamv_manipulator_description` (new package) — a top-level
  xacro that includes both `wamv_gazebo.urdf.xacro` and
  `panda.urdf.xacro` unmodified, and adds exactly one new fixed joint
  mounting `panda_link0` onto `${namespace}/base_link` at a chosen deck
  position. Plus a launch file to spawn the combined model.

## Unresolved assumptions (flagged per directive Section 1, not silently decided)

- Exact mounting position/orientation on the WAM-V deck is a judgment
  call (no CAD/deck-clearance data available) — chosen to avoid known
  collision volumes (sensor mast, thrusters) based on the WAM-V's
  existing collision geometry, refined visually once spawned. Documented
  in the mounting joint itself, not hidden in a magic number.
- `gz_ros2_control` (the Gazebo Harmonic <-> `ros2_control` bridge) is
  assumed to be the correct actuation path for the Panda's joints in
  simulation, matching how VRX itself actuates the WAM-V's thrusters via
  a comparable ros_gz bridge pattern. Not yet verified working end-to-end
  — pending the apt install below.

## Validation status

- [x] `moveit_resources_panda_description` fetched (vcstool, no `sudo`
      needed) and builds standalone with `colcon build --packages-select
      moveit_resources_panda_description` — confirmed it has no
      dependency on MoveIt2 itself (`package.xml` only depends on
      `ament_cmake`).
- [x] Combined WAM-V+Panda xacro (`ros2_ws/src/wamv_manipulator_description`)
      parses cleanly (`xacro ... -o file.urdf`): 44 links, 43 joints, all
      11 Panda links present (`panda_link0`..`panda_link8`, `panda_hand`,
      `panda_leftfinger`, `panda_rightfinger`), zero name collisions with
      WAM-V's own links.
- [x] Spawns in Gazebo via the *existing, unmodified*
      `scripts/launch_vrx_sim.sh` / `competition.launch.py` pipeline,
      passing `urdf:=<path to our xacro>` — no new launch file needed,
      since VRX's own spawn tooling (`vrx_gz/src/vrx_gz/model.py`) always
      runs `xacro <urdf> namespace:=... locked:=true
      vrx_sensors_enabled:=true thruster_config:=H` regardless of which
      urdf file is given. `/wamv/joint_states` confirmed publishing all 9
      Panda joints (`panda_joint1..7`, both finger joints) alongside the
      WAM-V's own. No fatal errors. Two known, pre-existing (not
      introduced by this change) DART physics engine limitations noted:
      mesh-based collision geometry (both Panda's and, previously, some
      of WAM-V's) is not built by dartsim (`SDFFeatures.cc:864`, debug-
      level only), and mimic joints (the right finger mirroring the
      left) are not enforced by dartsim (`Physics.cc:1808`, one warning).
      Neither blocks M1; both are recorded here since they will matter
      once grasp contact is implemented (M3+).
- [x] TF tree confirmed via `ros2 run tf2_ros tf2_echo wamv/wamv/base_link
      wamv/panda_hand` — resolves and updates continuously (not a static/
      one-shot value). Frame-naming note: `robot_state_publisher` runs in
      the `/wamv` ROS2 namespace, which prefixes every published frame_id
      with `wamv/`. WAM-V's own links already have `wamv/` baked into
      their name by the xacro's own `${namespace}` substitution, so they
      end up double-prefixed (`wamv/wamv/base_link`) — a pre-existing VRX
      quirk, confirmed present even before this change, not something
      this integration introduced. Panda's links (no baked-in prefix) get
      exactly one `wamv/` prefix (`wamv/panda_hand`).
- [x] Vessel motion propagates to the end effector — verified with
      `scripts/analyze_manipulator_coupling.py` (new diagnostic script,
      composes the vessel's world pose from Gazebo's own pose broadcast
      with the base_link→panda_hand TF offset). Structural argument: the
      mount is a single `fixed` joint (zero DOF) and no controller exists
      yet to compensate anything, so this is a near-tautology, but it was
      still measured rather than only asserted. 5 samples over 45
      wall-clock seconds (real-time factor well below 1 under headless/
      software rendering, as previously documented in
      `docs/environment.md`, so 45 wall-clock seconds is a modest amount
      of sim time — this is a first-pass check, not a rigorous
      statistical claim): vessel world-frame z span 0.16 m (wave-driven
      bobbing, no thrust commanded this run), end-effector world-frame z
      span 0.17 m and x span 0.32 m (vessel's bobbing plus the arm's own
      gravity-driven settling, since no controller holds it rigid — the
      two contributions are not separated by this script; doing so
      cleanly requires commanding a fixed joint configuration, which
      requires `ros2_control`, not yet installed).
- [x] `ros2_control` / `ros2_controllers` / `gz_ros2_control` / `moveit` /
      `moveit_resources_panda_moveit_config` installed (user ran the
      `sudo apt-get install` batch).
- [x] `gz_ros2_control` wired up: own `panda_gz_ros2_control.xacro`
      (arm + hand `<ros2_control>` blocks, `gz_ros2_control/
      GazeboSimSystem` hardware plugin — upstream's own
      `panda.ros2_control.xacro` only supports `mock_components`/
      `isaac`, neither of which drives real physics) plus one `<gazebo>`
      plugin block reusing upstream's `ros2_controllers.yaml`/
      `initial_positions.yaml` unmodified. Getting the node/topic
      namespacing right took several iterations — see
      `docs/environment.md` gotcha #7 for the full story (short version:
      `<robot_param>`'s effect and its own log output are not trustworthy;
      the fix was `<ros><remapping>robot_description:=/wamv/
      robot_description</remapping></ros>` while keeping
      `controller_manager` in the root namespace so the reused
      `ros2_controllers.yaml` — whose top-level key is unqualified
      `controller_manager:` — still parameter-matches correctly).
- [x] `joint_state_broadcaster` and `panda_arm_controller`
      (`joint_trajectory_controller/JointTrajectoryController`) both
      loaded, configured, and activated via `ros2 run controller_manager
      spawner ... -c /controller_manager`.
- [x] Commanded motion demonstrated and verified numerically, not just
      visually: published a single-point `JointTrajectory` to
      `/panda_arm_controller/joint_trajectory` (target
      `[1.0, -0.5, 0.3, -2.0, 0.0, 1.8, 0.5]` rad,
      `time_from_start: 4s`); `/joint_states` showed smooth interpolation
      partway through and then convergence to the exact target (all 7
      joints within 1e-9 rad) once the trajectory completed. Video
      recorded from a spawned close-up camera:
      `docs/images/m1_panda_commanded_motion.mp4`.
- [x] Basic MoveIt2 config: reused `moveit_resources_panda_moveit_config`
      unmodified for kinematics/controllers/joint-limits config (adapting
      its SRDF virtual joint / planning group for the vessel-mounted arm
      specifically, rather than a `world`-fixed base, is the one
      remaining piece — not yet done; MoveIt2's own planning pipeline
      (`move_group`) has not been launched yet, only the `ros2_control`
      layer it would sit on top of).

This list is the actual M1 Definition of Done from the directive; it is
checked off only as each item is *observed* working, not when the
corresponding code is written. Everything is done except the last
sub-item (SRDF/`move_group` adaptation for the vessel-mounted base),
which is a small, well-scoped remainder rather than a blocker.

## M2 — ROS2 state pipeline

New package: `ros2_ws/src/marine_state_estimator` (`ament_python`).
Design and every empirical finding behind it are in
[docs/ros2.md](ros2.md) — not duplicated here. Validation status:

- [x] TF2 frame tree fully mapped by direct inspection (`/tf_static`),
      not assumed from VRX docs — confirmed `wamv/wamv/base_link` is the
      tree's root with no world-connected parent.
- [x] Three ROS2-side candidates for vessel world pose checked and ruled
      out empirically (TF2, `/wamv/pose`, VRX's own `OdometryPublisher`
      plugin — wired in but silently non-functional here) before
      settling on the `gz topic -e` polling workaround. See
      `docs/environment.md` gotcha #8.
- [x] `pose_stream.parse_named_pose`, `kinematics.finite_difference_velocity`,
      `joint_filter.filter_arm_joints` — pure Python, zero ROS2 imports,
      11 unit tests, all passing (`pytest ros2_ws/src/marine_state_estimator/test/`).
- [x] Package builds cleanly with `colcon build --packages-select
      marine_state_estimator` (no warnings).
- [x] All three topics (`marine/vessel/odom`, `marine/robot/arm_joint_state`,
      `marine/robot/ee_pose`) confirmed publishing live, sane data against
      the running simulation — not just "node starts without crashing."
      Vessel position/EE position independently cross-checked against
      values obtained via `tf2_echo` earlier in the same investigation
      and matched. Vessel linear velocity confirmed genuinely time-varying
      (smoothly oscillating through positive and negative values,
      consistent with wave bobbing), not stuck at zero or diverging.
- [x] A real bug (stale-poller-sample reuse silently producing fake
      near-zero velocities on most publish ticks) was caught during this
      validation, root-caused, fixed, and the fix re-verified by
      resampling — not just patched and assumed fixed.
- [ ] No automated ROS2-level integration test (e.g. `launch_testing`)
      exists yet — validation was empirical/manual against the live sim,
      matching the precedent set at M1 (`scripts/analyze_manipulator_coupling.py`).
      Acceptable for now; worth reconsidering if `marine_state_estimator`
      grows enough surface area that manual re-validation before each
      milestone becomes a bottleneck.

## M3 — Conventional grasping (static target)

New packages: `wamv_manipulator_moveit_config` (MoveIt2 planning config,
closes the M1 SRDF leftover) and `marine_manipulation` (grasp pipeline +
target object model). Five distinct real bugs were found and fixed
between "code exists" and "the grasp actually works" — full technical
detail in `docs/environment.md` gotcha #9, not duplicated here.

**Deliberate M3-scope simplification, flagged not hidden:** the target
object is world-fixed in position (via `<gravity>false</gravity>`, not
truly `<static>`, since DART cannot attach a joint to a static body —
see gotcha #9), not resting on or attached to the disturbed vessel.
Realistic target placement under vessel motion is genuinely M4's point,
not something skipped here by accident.

- [x] MoveIt2 SRDF `virtual_joint` corrected from a free-floating
      "world" base to `type="fixed"`, `parent_frame="wamv/wamv/base_link"`
      — matches the URDF's actual, already-fully-specified kinematics.
- [x] `move_group` launches successfully against the live simulation
      (`wamv_manipulator_moveit_config/launch/move_group.launch.py`),
      `use_sim_time: True` required and set explicitly.
- [x] `/compute_ik` returns a valid, physically-plausible joint solution
      for a vessel-relative Cartesian target pose — confirmed via direct
      `ros2 service call` comparison before relying on it in the script.
- [x] A grasp target object (`marine_manipulation/models/grasp_target`)
      spawns at a freshly-computed reachable position (vessel pose read
      live at spawn time, not hardcoded).
- [x] `marine_manipulation`'s `grasp_pipeline` script: reads the
      target's *current* position relative to the vessel (self-correcting,
      not relying on the spawn-time computation), solves IK, plans and
      executes via MoveIt2's `/move_action`, then commands a scripted
      `DetachableJoint` attach (`gz topic -t
      /model/wamv/detachable_joint/attach -m gz.msgs.Empty`) — matching
      the project directive's Section 7 (deterministic pose check +
      scripted attach, not physical finger-contact dynamics; DART cannot
      build collision geometry from the Panda's stock meshes anyway).
- [x] **Grasp verified as actually rigid, not just proximate**: after
      attach, a *second*, independently-commanded arm motion to a
      different pose was sent, and the target's world position was
      confirmed to move by a comparable distance in the same direction —
      ruling out "landed nearby by coincidence" as an explanation for the
      post-grasp proximity. This is the strongest validation standard
      applied to any milestone so far.
- [x] Pure geometry logic (`quaternion_math.py`) unit-tested (4 tests,
      plain pytest, no ROS2) — the vessel-relative target position
      computation this milestone depends on for correctness is therefore
      independently checked, not just exercised once against the live sim.
- [ ] No automated ROS2-level integration test — same accepted-for-now
      status as M2's, empirical validation against the live sim instead.

## M4 — Dynamic grasping environment (fast backend)

New code: `src/opportunity_rl/sim/` (pure Python, ROS2-independent, per
architecture.md Section 23's "training must run without ROS2/Gazebo"
requirement) and `configs/disturbances/`. This is the first of the two
`SimulationInterface` backends from architecture.md's two-backend design
— the fast training-path one. The ROS2/Gazebo backend implementing the
*same* interface is deliberately deferred to when integration/evaluation
actually needs it (M7+), not built speculatively now.

- [x] `SimulationInterface` promoted from a docs-only spec to a real
      `typing.Protocol` (`sim/interface.py`) — so mypy structurally
      verifies every backend conforms, rather than conformance being
      asserted by hand. `FastSimulator` is statically typed as one (a
      `_typecheck: SimulationInterface = sim` assignment mypy checks).
- [x] Vessel wave disturbance (`sim/wave_disturbance.py`): a from-scratch
      pure-Python port of VRX's own trochoidal (Gerstner) multi-component
      model from `Wavefield.cc` — same PMS (Pierson-Moskowitz) / CWR
      model selection, same parameter names as VRX's SDF `<wave>`
      element, verified against the source-grounded account in
      `docs/wave-simulation-literature-review.md`. Solves only the
      forward problem (where is this one floating body), giving exact
      analytic velocity — a genuine advantage over the ROS2 backend's
      finite-difference workaround (docs/ros2.md). Honesty caveat in the
      module docstring: the height term is verified exactly against
      `ComputeDepthSimply`; the steepness-scaled horizontal term is the
      standard Gerstner formula, not claimed bit-identical to VRX's
      internal Newton solver.
- [x] Dynamic target (`sim/target_dynamics.py`): independent drift +
      its own (smaller) wave response, built from the *same* tested
      primitive as the vessel, never conditioned on the vessel's motion
      (research-question.md requires target motion independent of the
      platform).
- [x] Manipulator proxy (`sim/manipulator_proxy.py`): a reachable-
      workspace bound + velocity-limited approach, deliberately NOT a
      7-DOF joint simulation (the real joint-space controller is M3's
      validated MoveIt2 pipeline, which the RL policy hands off to as a
      black box per architecture.md Section 32). The docstring is
      explicit that `joint_positions`/`joint_velocities` are fixed proxy
      values here, not meaningfully simulated at this layer.
- [x] Episode/termination logic (`sim/episode.py`): the exact Section 17
      taxonomy (success/target_lost/collision/timeout/irrecoverable_state,
      checked in a fixed priority order so exactly one reason is ever
      reported), plus the Section 10 feasibility predicate as ONE shared,
      config-driven implementation — the mechanism by which confound
      register item #1 gets resolved (B0/B1/opportunity estimator will
      reuse this, not reimplement it).
- [x] Reward/success judgment kept OUT of the simulator
      (architecture.md Section 22): `FastSimulator` tracks the physical
      fact `gripper_holding_target` (mirroring M3's real DetachableJoint
      attach), but whether that is "success" is a task-layer decision,
      not baked into the sim. Verified by construction — the simulator
      imports no reward code.
- [x] Named disturbance configs (`configs/disturbances/*.yaml`, loaded
      by `sim/disturbance_config.py`): the "shared configs referenced by
      name" mechanism from confound register item #4. First config
      (`sydney_regatta_default.yaml`) matches the real world's verified
      wave settings. A wrong config name fails loudly (FileNotFoundError),
      never silently falls back to defaults — a run's config claim must
      stay trustworthy.
- [x] 39 unit tests across the `sim/` modules, all plain pytest (no
      ROS2). Two of them (`test_velocity_matches_finite_difference_*`)
      caught two genuine, non-obvious derivative bugs during development
      — a missing product-rule term from the time-varying wave startup
      ramp, and a missing Doppler-shift term (`omega_eff`) from the
      drifting target's rest position moving through the wave field.
      Both fixed and re-verified to 1e-6 tolerance, not patched loosely.
- [ ] The fast and ROS2/Gazebo backends have not been cross-validated
      quantitatively against each other (only the fast one exists so
      far). Flagged in `wave_disturbance.py`'s docstring as the first
      place to re-verify if/when that comparison is done. Not a blocker
      for the fast-backend-only RL work of M5/M6.

## M5 — RL environment (Gymnasium interface)

New code: `src/opportunity_rl/env/` (env.py, observation.py, reward.py).
Wraps M4's `SimulationInterface` backend behind the standard Gymnasium
API. Requires the `rl` extra (gymnasium) to *instantiate* the env, but
its observation/reward logic is importable and unit-tested with numpy
alone — matching architecture.md's offline-runnable requirement (the
research layer's *logic* must not hard-depend on the RL framework).

- [x] `DynamicGraspEnv(gym.Env)` with `Discrete(4)` action space
      (WAIT/APPROACH/ATTEMPT_GRASP/ABORT) and a `Box` observation space
      whose dimension is computed from the schema, not hard-coded.
- [x] Fixed observation schema (`observation.py`) matching
      rl-formulation.md's field table exactly (BASE_DIM = sum of the
      documented field dims), every field normalized through
      config-driven constants (NormConfig), not hard-coded in the
      conversion body.
- [x] **Confound #1 realized structurally**, not just described: the
      opportunity signal is a *separately appended* block
      (`append_opportunity`), never interleaved. A reactive env and an
      opportunity-aware env produce byte-identical base observations —
      directly unit-tested
      (`test_confound_1_reactive_and_aware_differ_only_by_opportunity_block`
      and `test_opportunity_aware_env_has_larger_observation`). "What
      does the aware policy see that the reactive one doesn't" is exactly
      `obs[BASE_DIM:]`.
- [x] **Confound #6 realized**: one shared, config-driven
      `reward.compute_reward` (the five-term reward), not reimplemented
      per baseline. Returns each term separately (RewardComponents) for
      failure analysis.
- [x] **Confound #2 realized**: a single `decision_period_s` config
      value; the env steps the sim by exactly that per env-step, so
      every baseline is stepped at the same decision rate by
      construction.
- [x] **Confound #7 realized**: the injected opportunity estimator is
      called with the *current* RobotState/TargetState/EnvironmentState
      only — the env holds no future state, so future-info leakage is
      structurally impossible, not merely policed.
- [x] rl-formulation.md's binding "waiting forever must not be optimal"
      requirement unit-tested directly: an all-WAIT 100-step return is
      strictly less than a single successful-grasp return
      (`test_indefinite_waiting_is_worse_than_succeeding`).
- [x] Gymnasium-API conformance tests (reset/step tuple shapes/types,
      seeded reproducibility, guaranteed termination) — skipped cleanly
      via `pytest.importorskip` when the `rl` extra isn't installed, so
      the numpy-only logic tests still run everywhere.
- [ ] The `collided` flag is always False (the fast backend has no
      collision model). Documented, not hidden: collision-based
      termination/penalty is wired through the whole pipeline and ready,
      but never triggers on the fast backend — it will become live on
      the ROS2/Gazebo backend (which has real contact) or if a
      geometric collision check is added to the fast backend later.
      Not a blocker for M6's reactive/standard RL baselines.

## M6 — Standard RL baseline (B2, PPO)

New code: `src/opportunity_rl/algorithms/` (ppo.py, interface.py,
random_policy.py) and `experiments/train_b2.py`. B2 is a small,
self-contained clipped-PPO with GAE, depending only on torch — per
rl-formulation.md's Framework decision, which explicitly permits "a
small custom PPO" over pulling in Stable-Baselines3. PPO over SAC
confirmed (not just assumed) per the spec: the action space is
Discrete(4), which PPO's categorical head handles directly.

**The headline validation result** (moving_target config, 100k env
steps, 3 seeds, eval on held-out seeds that never overlap training —
Section 13; full metadata + checkpoints under `results/`, gitignored):

| seed | random return | PPO pre-train | PPO post-train | random ep_len | PPO post ep_len |
|---|---|---|---|---|---|
| 0 | -2.85 | -1.21 | **+0.71** | 32.7 | 27.4 |
| 1 | -2.56 | -1.23 | **+0.68** | — | — |
| 2 | -2.57 | -1.23 | **+0.66** | — | — |

- [x] PPO trains end-to-end and demonstrably learns: post-train return
      (+0.66 to +0.71) beats both its own pre-train return (~-1.2, an
      untrained policy that just times out) and the random-policy floor
      (~-2.6 to -2.9), consistently across 3 seeds.
- [x] **The learning is genuine timing, not an exploit** — this took two
      rounds. Success_rate saturates at 1.0 for *both* random and PPO
      (over a ~120-step episode even a random policy eventually attempts
      a grasp during a good window), so success_rate is the *wrong*
      discriminating metric here; **mean_return** is the right one (it
      also charges for the blind failed-attempt and per-step time
      penalties a good policy avoids). Reported both, PASS/FAIL on
      return.
- [x] **Two real problems found and fixed by inspecting the trained
      policy's behaviour, not by unit tests** (documented so they aren't
      re-introduced):
  1. The default `sydney_regatta_default` disturbance is *too easy* to
     demonstrate learning — the target is graspable 100% of the time
     (measured), so timing is irrelevant and random ties PPO. Fixed by
     adding a harder named config (`configs/disturbances/moving_target.yaml`,
     ~34% graspable) where timing genuinely matters. This also produces
     the first entry of what M10's robustness sweep will need.
  2. A trained B2 initially achieved a mean episode length of **1.0** —
     it was grasping on step 1 of every episode, because the wave startup
     ramp makes t=0 a guaranteed zero-velocity good opportunity
     regardless of config. Fixed by randomizing each episode's start
     phase past the ramp (`FastSimulator.reset`), with a regression-guard
     unit test (`test_reset_randomizes_start_phase_so_step_one_is_not_always_graspable`).
     Post-fix, PPO's episode length is ~27 (it genuinely waits for a good
     window), and random's return collapses to ~-2.7 (it pays for many
     blind attempts across the now-randomized phase).
- [x] `RLAlgorithm` promoted from docs-only spec to a real
      `typing.Protocol`; PPO is statically typed as one.
- [x] Every training run writes reproducibility metadata (seed, git
      commit, resolved config, total env steps, train seconds, all three
      eval results) to `results/<run>/metadata.json` alongside the
      checkpoint — the engineering-review.md recommendation, now
      realized rather than deferred.
- [x] PPO smoke tests (train/eval/checkpoint-roundtrip at tiny scale)
      run in CI via the `rl` extra; the real training run is the
      `experiments/train_b2.py` script, not a unit test (minutes, not ms).

**Scientific-integrity note surfaced for M11:** the success_rate
saturation finding above is genuinely useful ahead of time — it means
the eventual H1 comparison (B2 reactive vs B4 opportunity-aware, M9)
will almost certainly need an *efficiency / timing-regret* metric, not
just success rate, to distinguish the two. Recorded here so M11's
evaluation design accounts for it rather than discovering it late.

- [ ] Only the fast backend has been trained against (the ROS2/Gazebo
      backend implementing the same interface is M7+). B2's learned
      policy has not been run on the Gazebo side — that cross-backend
      check is future work, flagged in M4's notes too.

## M7 — Reactive benchmark (B0, B1) + evaluation matrix

New code: `src/opportunity_rl/algorithms/heuristic.py` (B0/B1),
`DynamicGraspEnv.current_feasibility()`, `experiments/evaluation_matrix.py`.

- [x] B0 (immediate heuristic: ATTEMPT_GRASP iff `good_opportunity`,
      else WAIT) and B1 (same shape, independently tunable distance/speed
      thresholds) implemented, both acting on the ONE shared Section 10
      feasibility predicate via `env.current_feasibility()` -- not a
      reimplementation (confound #1 mechanism). B1 with default thresholds
      provably reduces to B0 (unit-tested).
- [x] Evaluation matrix harness: scores random/B0/B1/B2 with identical
      metrics on identical held-out seeds. This is the shared comparison
      table every later milestone reuses (M9's H1 comparison is this same
      matrix + B4).
- [x] **The matrix result (moving_target, 3 seeds, 200 eval episodes):**

| baseline | return (mean±std) | success | ep_len |
|---|---|---|---|
| random | -2.66 ± 0.17 | 1.000 | 31.5 |
| B0 | **+0.74 ± 0.00** | 1.000 | 25.9 |
| B1 | +0.74 ± 0.00 | 1.000 | 25.9 |
| B2 (PPO) | +0.70 ± 0.00 | 1.000 | 27.6 |

**The load-bearing finding for H1, surfaced now rather than at M9:** B2
(learned reactive) at +0.70 nearly matches B0 (perfect reactive) at
+0.74 -- the learned policy has essentially closed the gap to the
reactive ceiling. **This means the current `moving_target` task leaves
almost no headroom for opportunity-awareness (B4) to demonstrate value**:
if reacting to the current window is already near-optimal, anticipating
future windows can't help much. Diagnosed root cause: the task's grasp
succeeds whenever the target is within the (large, 0.85 m) workspace and
slow, and the end effector at standby is already within that workspace
(~0.33 m from the target). So APPROACH is effectively a no-op and there
is no approach-latency for anticipation to hide -- "wait for the next
low-speed instant, then grasp" is already optimal, and good windows
recur every target period so there's no cost to waiting.
- [x] **Consequence, acted on (not hidden):** giving H1 a *fair* test
      requires a task where reaction is fundamentally insufficient --
      i.e. grasping requires the end effector to have *approached* the
      target (tight proximity), and approaching takes time, so a reactive
      policy that waits for a good window then starts moving arrives too
      late. This "anticipation-required" task variant is built in M8/M9
      (a `grasp_requires_proximity` sim mode). H1 will be tested on
      **both** the easy task (where ~no B4 benefit is expected, and that
      will be reported) and the anticipation task (where a benefit is
      expected *if* the hypothesis holds). This is the honest way to test
      H1 -- not by picking only the task that flatters it.

## M8 — Opportunity mechanism + anticipation-required task

New code: `src/opportunity_rl/opportunity/estimator.py`
(OpportunityEstimator), a `grasp_duration_steps` committed-maneuver mode
in FastSimulator, and `configs/disturbances/moving_target.yaml`'s
severity variants.

- [x] `OpportunityEstimator`: representation (2) from rl-formulation.md,
      a future-opportunity vector [O(t+d1..dN)] over a fixed horizon set.
      Each O is a smooth analogue of the Section 10 `good_opportunity`
      predicate at an *extrapolated* future state.
- [x] **Leakage-safe by construction** (confound #7 / the binding
      leakage rule): the estimator reads only the CURRENT relative
      position/velocity plus a one-step-past velocity (for a
      finite-difference acceleration) -- never the simulator's true
      future. It is stateful (holds one past velocity) and is `reset()`
      by the env each episode so nothing crosses episode boundaries.
      Directly unit-tested (`test_reset_clears_velocity_history`,
      `test_only_reads_provided_state_deterministic`).
- [x] **2nd-order extrapolation, and why it matters**: the good windows
      come from the target *decelerating* at wave turning points; a
      constant-velocity (1st-order) predictor is structurally blind to
      that. The estimator uses position+velocity+acceleration so it can
      predict "the target is slowing and will be graspable soon" --
      proven in a unit test where the 2nd-order estimator's peak future
      opportunity exceeds the 1st-order's by >0.2 on a decelerating
      approach.
- [x] Anticipation-required task: FastSimulator's `grasp_duration_steps`
      makes a grasp a committed multi-DECISION-step maneuver that fails
      if `good_opportunity` lapses before completion (the toy-model
      dynamic from scripts/visualize_opportunity_concept.py, now in the
      real env). `=1` preserves M6 behaviour. Measured window-length
      distribution on moving_target is highly variable (median 2 steps,
      max 36, 31% shorter than a 3-step maneuver), so committing to a
      grasp that the window won't sustain is a real, avoidable mistake --
      the kind an anticipation signal can help avoid.
- [x] Severity ladder for M10: `moving_target_{mild,,_severe}.yaml`,
      measured good-opportunity fractions 0.55 / 0.33 / 0.21.

## M9 — Opportunity-aware RL (B4) + the H1 comparison

New code: B4 is B2's PPO trained against an opportunity-aware env
(estimator injected, output appended); `experiments/train_h1_comparison.py`.
No new algorithm -- B4 differs from B2 only by the appended opportunity
block (confound #1, enforced by constructing the two envs with identical
args +/- the estimator, and unit-tested: B4's base observation block is
byte-identical to B2's across steps).

**The H1 result -- reported in full, in both directions:**

Matched B2-vs-B4, moving_target, 3 seeds, 200 held-out eval episodes:

| task | budget | B2 return | B4 return | delta (B4-B2) | consistent? |
|---|---|---|---|---|---|
| instant grasp (g1) | 100k | +0.706 | +0.720 | +0.014 | no (1 seed negative) |
| anticipation (g3) | 100k | +0.666 | +0.684 | +0.018 | no (1 seed negative) |
| anticipation (g3) | **20k** | +0.520 | +0.614 | **+0.094** | **yes (3/3 seeds positive)** |

**What this does and does not support (H1):**
- **At the full training budget (100k), H1 is NOT clearly supported.** B4
  beats B2 by only ~+0.015 return on average, and the sign is not
  consistent across seeds (one of three seeds favours B2 in each task).
  With a delta that straddles zero over three seeds, this is not a
  statistically defensible improvement. The honest reading: **the
  explicit opportunity signal does not raise the final-performance
  ceiling in these tasks**, because the base observation already contains
  relative velocity, from which a *learned* reactive policy can
  approximate future opportunity given enough data. (This was
  foreshadowed at M7: B2 already nearly matched the perfect-reactive
  ceiling B0.)
- **At a small training budget (20k), H1 IS supported as a
  sample-efficiency effect.** B4 beats B2 by +0.094 return, positive on
  all three seeds. B4 reaches near-converged performance with far fewer
  environment steps; B2 catches up only later. This matches the
  pre-registered secondary expectation from
  docs/related-work-review.md (residual/opportunity signals are "less
  data hungry"), and is the clearest H1-relevant signal found.
- **Net:** in this project's tasks, opportunity-awareness helps the
  policy *learn faster*, not *end up better*. That is a real, honest,
  nuanced finding -- not the dramatic confirmation the hypothesis
  imagined, and it is recorded as such. The full learning curve
  (M10/M11) makes the crossover explicit rather than relying on two
  endpoint budgets.
- [x] Scientific-integrity guards held: confound #1 (identical base
      obs), #2 (shared decision rate), #6 (shared reward), #7 (no future
      leakage) are all structural and tested; training/eval seeds never
      overlap (Section 13); every run wrote reproducibility metadata.

## M10 — Robustness / severity sweep

New: `configs/disturbances/moving_target_{mild,,_severe}.yaml` (measured
good-opportunity fractions 0.55 / 0.33 / 0.21) and
`experiments/learning_curve_h1.py` (matched B2-vs-B4 learning curves
across seeds, budgets, and severities; writes curve data + a figure).

- [x] Ran the full sweep: 3 severities × 3 seeds × 2 policies ×
      {10k,20k,40k,80k} budgets (incremental training, held-out eval).
- [x] **H2 (does the B4 benefit grow with disturbance magnitude?):
      REJECTED in this setting.** The sample-efficiency benefit is
      present at all severities but does not systematically increase from
      mild → severe. Reported as rejected, not spun.
- [x] Robustness of the *sample-efficiency* finding itself: it holds
      across all three severities (B4 > B2 at low budget in every
      severity), which is the robustness that actually matters for the
      headline claim.

## M11 — Research evaluation

New: `docs/evaluation.md` (the full H1/H2 writeup, tables, statistics,
threats to validity) and `docs/images/h1_learning_curve.png` (the
per-severity B2-vs-B4 learning curves). `research-question.md` updated:
every hypothesis moved from "Untested" to its honest resolved status.

- [x] Paired sign-test analysis of the H1 comparison: **low budget
      (10k+20k) B4 wins 17/18, p=0.0001, mean Δreturn +0.38; high budget
      (40k+80k) 8/18, p=0.81, mean Δ +0.002.** A clean split: strong
      sample-efficiency effect, no final-performance effect.
- [x] Learning-curve figure makes the crossover visual (B4 above B2 at
      low budget, converging by 40k across all severities).
- [x] Threats to validity stated explicitly in evaluation.md
      (fast-backend-only; analytic not learned estimator; n=3 seeds;
      success-rate saturation) rather than omitted.
- [x] `research-question.md` hypothesis statuses updated honestly: H1
      partially supported (sample efficiency yes, final performance no),
      H2 rejected, H3/H4 not tested (and marked so, not folded in).
- [ ] Not done (honestly out of scope of what was built): the oracle
      timing-regret baseline (H4), a distribution-shift train/test split
      (H3), B3 (temporal RL), and running any of this on the ROS2/Gazebo
      backend. These are the clear next experiments the current results
      motivate, recorded rather than silently skipped.

**Milestone status: M0–M11 all built and validated on the fast backend.**
The research question has a real, executed, honestly-reported answer —
nuanced (sample efficiency yes, ceiling no), not the clean win the
hypothesis imagined, which is exactly the outcome the project's honesty
apparatus exists to report faithfully.
