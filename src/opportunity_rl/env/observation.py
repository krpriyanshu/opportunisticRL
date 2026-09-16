"""The fixed observation schema from docs/rl-formulation.md's Observation
space section, plus the opportunity-field append mechanism that makes
confound register item #1 checkable by inspecting one field list.

Two hard rules from rl-formulation.md, enforced structurally here:

1. The reactive and opportunity-aware policies read the *same* base
   schema. The opportunity-aware policy additionally receives an
   OpportunityEstimator's output appended as an explicitly separate,
   named block -- never mixed into the base fields. So this module has a
   clean split: `base_observation()` (identical for every baseline) and
   `append_opportunity()` (the only difference between reactive and
   opportunity-aware). Confound #1 is then a one-line check: do the two
   policies differ *only* by whether append_opportunity was called.

2. Every field is normalized to a documented, bounded range before a
   policy sees it, using constants that live in config (NormConfig),
   not hard-coded in the middle of the conversion.

No torch here (numpy only): observation construction is part of the
environment, which architecture.md requires to be runnable without the
RL framework.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from opportunity_rl.sim.state import EnvironmentState, RobotState, TargetState

# Field layout, in order, matching rl-formulation.md's table. Each entry
# is (name, dimension). The pose_uncertainty/velocity_uncertainty
# optional fields (Section 8) are represented as zero-filled blocks that
# are DISABLED by default -- present in the schema so enabling them later
# is a documented config change, not a silent default, exactly as the
# spec requires.
BASE_FIELDS: tuple[tuple[str, int], ...] = (
    ("relative_target_position", 3),
    ("relative_target_orientation", 4),
    ("relative_target_linear_vel", 3),
    ("relative_target_angular_vel", 3),
    ("ee_pose", 7),
    ("joint_state", 14),  # 2 * 7 arm joints (position + velocity)
    ("vessel_linear_vel", 3),
    ("vessel_angular_vel", 3),
    ("distance_to_grasp_region", 1),
    ("orientation_error", 1),
    ("relative_approach_velocity", 1),
    ("task_state", 4),  # one-hot over WAIT/APPROACH/ATTEMPT_GRASP/ABORT
)

BASE_DIM: int = sum(dim for _, dim in BASE_FIELDS)


@dataclass(frozen=True)
class NormConfig:
    """Normalization constants. Everything divided through by a
    characteristic scale so the network sees roughly unit-range inputs.
    Config-driven per rl-formulation.md's binding rule, not hard-coded
    in the conversion body."""

    position_scale_m: float = 3.0
    linear_velocity_scale_mps: float = 1.0
    angular_velocity_scale_rps: float = 3.14159
    joint_position_scale_rad: float = 3.14159
    joint_velocity_scale_rps: float = 2.175  # Panda's max joint velocity (M1 joint_limits.yaml)
    distance_scale_m: float = 3.0
    orientation_error_scale_rad: float = 3.14159


def _norm3(v: tuple[float, float, float], scale: float) -> list[float]:
    return [v[0] / scale, v[1] / scale, v[2] / scale]


def base_observation(
    *,
    robot: RobotState,
    target: TargetState,
    environment: EnvironmentState,
    relative_target_position: tuple[float, float, float],
    relative_target_velocity: tuple[float, float, float],
    distance_to_grasp_region: float,
    orientation_error: float,
    relative_approach_velocity: float,
    task_state: int,
    norm: NormConfig,
) -> np.ndarray:
    """Build the normalized base observation vector (BASE_DIM,). This is
    identical for every baseline -- the reactive and opportunity-aware
    policies both start here; only the opportunity-aware one then calls
    append_opportunity() on top."""
    values: list[float] = []

    values += _norm3(relative_target_position, norm.position_scale_m)
    values += list(target.orientation)  # quaternion, already unit-ish
    values += _norm3(relative_target_velocity, norm.linear_velocity_scale_mps)
    values += _norm3(target.angular_velocity, norm.angular_velocity_scale_rps)

    values += _norm3(robot.ee_position, norm.position_scale_m)
    values += list(robot.ee_orientation)

    for p in robot.joint_positions:
        values.append(p / norm.joint_position_scale_rad)
    for v in robot.joint_velocities:
        values.append(v / norm.joint_velocity_scale_rps)

    values += _norm3(environment.vessel_linear_velocity, norm.linear_velocity_scale_mps)
    values += _norm3(environment.vessel_angular_velocity, norm.angular_velocity_scale_rps)

    values.append(distance_to_grasp_region / norm.distance_scale_m)
    values.append(orientation_error / norm.orientation_error_scale_rad)
    values.append(relative_approach_velocity / norm.linear_velocity_scale_mps)

    one_hot = [0.0, 0.0, 0.0, 0.0]
    if 0 <= task_state < 4:
        one_hot[task_state] = 1.0
    values += one_hot

    arr = np.asarray(values, dtype=np.float32)
    assert arr.shape == (BASE_DIM,), f"base obs dim {arr.shape} != expected ({BASE_DIM},)"
    return arr


def append_opportunity(base: np.ndarray, opportunity: np.ndarray) -> np.ndarray:
    """Concatenate an OpportunityEstimator's output onto the base vector
    as a separate, clearly-delimited block. THE only difference between
    the reactive and opportunity-aware observations (confound #1). The
    opportunity block is appended, never interleaved, so 'what does the
    opportunity-aware policy see that the reactive one doesn't' is
    answered by exactly `base[BASE_DIM:]`."""
    return np.concatenate([base, np.asarray(opportunity, dtype=np.float32)])


@dataclass(frozen=True)
class ObservationSpec:
    """Describes the actual observation dimensionality of a configured
    environment (base only, or base + opportunity block)."""

    include_opportunity: bool
    opportunity_dim: int = 0
    fields: tuple[tuple[str, int], ...] = field(default=BASE_FIELDS)

    @property
    def total_dim(self) -> int:
        return BASE_DIM + (self.opportunity_dim if self.include_opportunity else 0)
