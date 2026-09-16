"""State types shared by both `SimulationInterface` implementations (the
fast backend here, and the future ROS2/Gazebo backend), per
docs/rl-formulation.md's frozen interface spec and
docs/architecture.md's layered view. Plain dataclasses, no ROS2 or
Gazebo types anywhere -- this is what keeps the RL research layer
independent of the robotics layer (architecture.md Section 23).

Vector/quaternion convention matches the rest of this project:
tuple[float, float, float] for R^3, tuple[float, float, float, float]
as (x, y, z, w) for quaternions (REP-103 / geometry_msgs order).
"""

from __future__ import annotations

from dataclasses import dataclass, field

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)
ZERO_VEC3: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RobotState:
    """Matches rl-formulation.md's ee_pose (vessel-base frame) and
    joint_state observation fields."""

    ee_position: Vec3  # vessel-base frame, meters
    ee_orientation: Quat  # vessel-base frame
    joint_positions: tuple[float, ...]  # radians, ARM_JOINT_ORDER order
    joint_velocities: tuple[float, ...]  # rad/s
    # A genuine physical fact (mirrors the real DetachableJoint attach
    # state from M3), not a reward/success judgment -- whether an
    # episode counts as "success" is a task-layer decision (episode.py,
    # used by the M5 environment), never decided inside the simulator
    # itself (architecture.md Section 22: "the simulator must never
    # contain RL-specific reward logic").
    gripper_holding_target: bool = False
    # True on exactly the step a committed grasp maneuver failed (window
    # closed before completion) or a one-step grasp attempt missed. Also
    # a physical fact, not a reward -- the env turns it into the failure
    # penalty. Lets a multi-step grasp report its outcome without the env
    # having to track the maneuver's internal state.
    grasp_just_failed: bool = False


@dataclass(frozen=True)
class TargetState:
    """World-frame target state. `graspable` is not part of the raw
    physical state -- it is left out deliberately; feasibility (Section
    10) is computed from this state by the task layer, not stored on it,
    so there is exactly one implementation of that logic (confound
    register item #1)."""

    position: Vec3  # world frame, meters
    orientation: Quat  # world frame
    linear_velocity: Vec3  # world frame, m/s
    angular_velocity: Vec3  # world frame, rad/s


@dataclass(frozen=True)
class EnvironmentState:
    """Vessel state, world frame. Matches rl-formulation.md's
    vessel_linear_vel / vessel_angular_vel observation fields plus the
    pose needed to convert between world and vessel-relative frames."""

    vessel_position: Vec3
    vessel_orientation: Quat
    vessel_linear_velocity: Vec3
    vessel_angular_velocity: Vec3
    sim_time: float


TerminationReason = str  # one of TERMINATION_REASONS, see episode.py


@dataclass(frozen=True)
class StepResult:
    """What one `DynamicGraspEnv.step()` needs from the simulation layer
    about how the step concluded -- separate from RobotState/TargetState/
    EnvironmentState (which describe *what things are*, not *what
    happened*). `terminated`/`reason` follow episode.py's Section 17
    termination taxonomy: an episode never ends with an unclassified
    reason (rl-formulation.md's binding requirement)."""

    terminated: bool
    reason: TerminationReason | None = None
    info: dict[str, float] = field(default_factory=dict)
