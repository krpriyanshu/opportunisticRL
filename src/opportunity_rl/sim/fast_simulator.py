"""The fast, pure-Python `SimulationInterface` implementation --
architecture.md's "training path" backend. Ties together
wave_disturbance (vessel), target_dynamics (target), and
manipulator_proxy (simplified end effector) behind the exact frozen
interface from docs/rl-formulation.md's Interfaces section:

    class SimulationInterface(Protocol):
        def reset(self, seed: int | None = None) -> None: ...
        def step(self, dt: float) -> None: ...
        def get_robot_state(self) -> RobotState: ...
        def get_target_state(self) -> TargetState: ...
        def get_environment_state(self) -> EnvironmentState: ...
        def apply_action(self, action: int) -> None: ...

Action values match rl-formulation.md's action space table exactly:
0=WAIT, 1=APPROACH, 2=ATTEMPT_GRASP, 3=ABORT.

What this class does and does not decide: it updates physical state
(positions, velocities, whether the gripper is currently latched onto
the target) in response to actions. It does **not** decide episode
termination or reward -- per architecture.md Section 22, that is the
task layer's job (episode.py + the M5 DynamicGraspEnv), computed from
the state this class exposes, not embedded in it. This mirrors the real
boundary already validated on the ROS2/Gazebo side at M3: the simulator
performs the scripted attach (a physical action), but whether that
constitutes "success" for RL purposes is a separate judgment.
"""

from __future__ import annotations

import random

from opportunity_rl.sim.disturbance_config import DisturbanceConfig
from opportunity_rl.sim.episode import FeasibilityConfig, evaluate_feasibility
from opportunity_rl.sim.interface import ABORT, APPROACH, ATTEMPT_GRASP, WAIT
from opportunity_rl.sim.manipulator_proxy import (
    RESTING_JOINT_POSITIONS,
    ZERO_JOINT_VELOCITIES,
    ManipulatorConfig,
    clamp_to_reach,
    step_toward,
)
from opportunity_rl.sim.state import (
    IDENTITY_QUAT,
    EnvironmentState,
    RobotState,
    TargetState,
    Vec3,
)
from opportunity_rl.sim.target_dynamics import TargetTrajectory
from opportunity_rl.sim.wave_disturbance import (
    VesselKinematics,
    build_components,
    vessel_kinematics,
)

# Action constants are re-exported here (they now live on the frozen
# SimulationInterface contract in interface.py) so existing
# `from ...fast_simulator import ATTEMPT_GRASP` call sites keep working.
__all__ = [
    "ABORT",
    "APPROACH",
    "ATTEMPT_GRASP",
    "DEFAULT_MOUNT_OFFSET",
    "DEFAULT_STANDBY_POSITION",
    "WAIT",
    "FastSimulator",
]

# Reused from M1's actual empirically-validated resting pose
# (moveit_resources_panda_moveit_config's initial_positions.yaml, and
# confirmed live via /wamv/joint_states) so the fast backend's default
# standby pose is grounded in a real measurement, not invented.
DEFAULT_MOUNT_OFFSET: Vec3 = (0.85, 0.35, 1.3)  # vessel-relative, manipulator_mount_joint
DEFAULT_STANDBY_POSITION: Vec3 = (1.157, 0.35, 1.89)  # vessel-relative, measured M1 resting EE pose


class FastSimulator:
    """Implements SimulationInterface. See module docstring."""

    def __init__(
        self,
        disturbance: DisturbanceConfig,
        mount_offset: Vec3 = DEFAULT_MOUNT_OFFSET,
        standby_position: Vec3 = DEFAULT_STANDBY_POSITION,
        manipulator_cfg: ManipulatorConfig | None = None,
        feasibility_cfg: FeasibilityConfig | None = None,
        vessel_start_position: Vec3 = (0.0, 0.0, 0.0),
        grasp_duration_steps: int = 1,
    ) -> None:
        self._disturbance = disturbance
        self._mount_offset = mount_offset
        self._standby_position = standby_position
        self._manipulator_cfg = manipulator_cfg or ManipulatorConfig()
        self._feasibility_cfg = feasibility_cfg or FeasibilityConfig()
        self._vessel_start_position = vessel_start_position
        # A multi-DECISION-step grasp maneuver: once ATTEMPT_GRASP is
        # committed, good_opportunity must hold for this many consecutive
        # decision steps for the grasp to succeed; if the window closes
        # mid-maneuver, the grasp fails. grasp_duration_steps=1 is the
        # instant grasp used through M6 (a single-step check), preserved
        # as the default. Values >1 make *commitment timing* matter: you
        # must commit when the window will PERSIST, not merely when it is
        # open right now -- which is exactly the case where anticipating
        # the opportunity's future trajectory beats reacting to its
        # current value (the M8/M9 anticipation-required task; see
        # docs/implementation-status.md's M7 finding on why M6's task had
        # no such headroom).
        self._grasp_duration_steps = grasp_duration_steps
        self._vessel_components = build_components(disturbance.vessel_wave)
        self._rng = random.Random()
        self.reset()

    def _vessel_kinematics(self, t: float) -> VesselKinematics:
        """Vessel wave kinematics at time t. Small helper so the (verbose)
        rest-xy tuple construction isn't repeated at every call site."""
        return vessel_kinematics(
            self._vessel_components,
            (self._vessel_start_position[0], self._vessel_start_position[1]),
            t,
        )

    def reset(self, seed: int | None = None) -> None:
        self._rng.seed(seed)
        # Start each episode at a RANDOM wave phase, past the startup
        # ramp, not at t=0. Why this matters (found by inspecting a
        # trained B2's behaviour, not by design): the wave startup ramp
        # (1 - exp(-t/tau)) makes t=0 a zero-displacement, zero-velocity
        # instant -- i.e. ALWAYS a good grasp opportunity, regardless of
        # the disturbance config. Starting every episode at t=0 therefore
        # lets a policy trivially "grasp on step 1" and bypass the timing
        # problem entirely (a first version did exactly this: post-train
        # mean episode length was 1.0). Offsetting the clock past the ramp
        # settle time (waves at full amplitude) by a random phase within
        # the longest wave period makes step 1 a random point in the
        # opportunity cycle, so the agent must actually wait for good
        # moments. t0 stays 0 so the ramp is complete at the offset time.
        settle = 5.0 * self._disturbance.vessel_wave.tau  # ramp ~1.0 by here
        max_period = max(self._disturbance.vessel_wave.period, self._disturbance.target.wave.period)
        # Sample the phase over MANY periods (20x), not just one: the
        # relative speed is a beat between the vessel and target wave
        # frequencies, and a single-period window can land entirely in a
        # high-speed part of that beat (found empirically -- an early
        # 1-period window put every reset just above the speed threshold,
        # so no reset was ever a good opportunity). A wide window makes
        # the reset-phase distribution match the steady-state one.
        self._t = settle + self._rng.uniform(0.0, 20.0 * max_period)
        self._ee_position = self._standby_position
        self._approach_target: Vec3 | None = None
        self._gripper_holding_target = False
        # Grasp maneuver state (only meaningful when grasp_duration_steps > 1):
        # steps remaining in an in-progress committed grasp, and whether
        # the most recent maneuver just failed (window closed mid-grasp).
        self._grasp_steps_remaining = 0
        self._grasp_just_failed = False
        self._target_trajectory = TargetTrajectory(self._disturbance.target, t0=0.0)

    def step(self, dt: float) -> None:
        self._t += dt
        if self._approach_target is not None:
            clamped = clamp_to_reach(
                self._approach_target, self._mount_offset, self._manipulator_cfg.reach_radius_m
            )
            self._ee_position, _ = step_toward(
                self._ee_position, clamped, self._manipulator_cfg.max_speed_mps, dt
            )

    def _good_opportunity_now(self) -> bool:
        feasibility = evaluate_feasibility(
            self.get_robot_state(),
            self._target_relative_position(self._t),
            self._target_relative_velocity(self._t),
            self._feasibility_cfg,
        )
        return feasibility.good_opportunity

    def apply_action(self, action: int) -> None:
        self._grasp_just_failed = False

        # If a multi-step grasp maneuver is in progress, it is committed:
        # WAIT/APPROACH/ATTEMPT_GRASP are ignored, and what matters is
        # whether good_opportunity still holds this step. ABORT is the one
        # action that can still change the outcome (rl-formulation.md
        # Section 2: ABORT "cancels an in-progress approach or grasp").
        if self._grasp_steps_remaining > 0:
            # The window check comes BEFORE the abort check on purpose. If
            # the window has already closed, the grasp has failed, and
            # aborting now cannot undo that. Otherwise a policy could
            # watch the current state, see the window close, and abort
            # every time to dodge the failure penalty. Checking the window
            # first means ABORT only helps when issued while the window
            # is still open, i.e. when the policy *anticipates* that the
            # window will close before the maneuver completes. That is
            # exactly the decision H1 is about.
            if not self._good_opportunity_now():
                # Window closed mid-maneuver -> the committed grasp fails.
                self._grasp_steps_remaining = 0
                self._grasp_just_failed = True
                self._approach_target = self._standby_position
                return
            if action == ABORT:
                # Deliberate cancel while the window is still open: no
                # grasp and no failure flag (so no failure penalty); the
                # arm returns to standby. The policy still pays the
                # per-step time cost for the steps it spent committed.
                self._grasp_steps_remaining = 0
                self._approach_target = self._standby_position
                return
            self._grasp_steps_remaining -= 1
            if self._grasp_steps_remaining == 0:
                self._gripper_holding_target = True
            return

        if action == WAIT:
            self._approach_target = None
        elif action == APPROACH:
            self._approach_target = self._target_relative_position(self._t)
        elif action == ATTEMPT_GRASP:
            self._approach_target = None
            if not self._good_opportunity_now():
                self._grasp_just_failed = True
                return
            if self._grasp_duration_steps <= 1:
                # Instant grasp (M6 behaviour): one-step check, succeed now.
                self._gripper_holding_target = True
            else:
                # Begin a committed multi-step maneuver. This step counts
                # as the first (good) step; the rest are consumed on the
                # following apply_action calls regardless of their action.
                self._grasp_steps_remaining = self._grasp_duration_steps - 1
        elif action == ABORT:
            self._approach_target = self._standby_position
            self._gripper_holding_target = False
        else:
            raise ValueError(f"unknown action {action!r}, expected one of 0,1,2,3")

    def get_robot_state(self) -> RobotState:
        return RobotState(
            ee_position=self._ee_position,
            # orientation control not modeled -- see manipulator_proxy
            ee_orientation=IDENTITY_QUAT,
            joint_positions=RESTING_JOINT_POSITIONS,
            joint_velocities=ZERO_JOINT_VELOCITIES,
            gripper_holding_target=self._gripper_holding_target,
            grasp_just_failed=self._grasp_just_failed,
        )

    def get_target_state(self) -> TargetState:
        # TargetTrajectory is a WORLD-frame trajectory: the target is an
        # independent floating body in the same sea, whose motion is never
        # conditioned on the vessel's (target_dynamics.py's module
        # docstring -- the research question hinges on that independence).
        # So its position is published as-is, NOT offset by the vessel.
        #
        # A previous version added the vessel's wave displacement here,
        # which silently disagreed with _target_relative_position() below
        # (that one subtracts the vessel, treating the trajectory as
        # world). The net effect was that the grasp gate in apply_action
        # and the observation/termination the env computes from this
        # method used relative positions differing by the vessel's heave
        # (~0.11 m peak under the shipped configs) -- i.e. two different
        # feasibility answers, defeating episode.py's "one implementation,
        # not reimplemented per consumer" invariant. Regression-guarded by
        # test_published_state_agrees_with_internal_relative_state.
        position, velocity = self._target_trajectory.state_at(self._t)
        return TargetState(
            position=position,
            orientation=IDENTITY_QUAT,
            linear_velocity=velocity,
            angular_velocity=(0.0, 0.0, 0.0),
        )

    def get_environment_state(self) -> EnvironmentState:
        kin = self._vessel_kinematics(self._t)
        return EnvironmentState(
            vessel_position=(
                kin.position[0],
                kin.position[1],
                self._vessel_start_position[2] + kin.position[2],
            ),
            # roll/pitch available on `kin`, not yet converted to quat
            vessel_orientation=IDENTITY_QUAT,
            vessel_linear_velocity=kin.linear_velocity,
            vessel_angular_velocity=(0.0, 0.0, 0.0),
            sim_time=self._t,
        )

    def _target_relative_position(self, t: float) -> Vec3:
        """Target position relative to the vessel -- note this is *not*
        the same as relative to the EE; callers needing distance to the
        EE combine this with get_robot_state().ee_position themselves,
        matching episode.evaluate_feasibility's own signature."""
        target_world, _ = self._target_trajectory.state_at(t)
        vessel_kin = self._vessel_kinematics(t)
        return (
            target_world[0] - vessel_kin.position[0],
            target_world[1] - vessel_kin.position[1],
            target_world[2] - vessel_kin.position[2] - self._vessel_start_position[2],
        )

    def _target_relative_velocity(self, t: float) -> Vec3:
        _, target_vel = self._target_trajectory.state_at(t)
        vessel_kin = self._vessel_kinematics(t)
        return (
            target_vel[0] - vessel_kin.linear_velocity[0],
            target_vel[1] - vessel_kin.linear_velocity[1],
            target_vel[2] - vessel_kin.linear_velocity[2],
        )
