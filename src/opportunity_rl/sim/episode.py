"""Episode termination logic, per docs/rl-formulation.md Section 17:
"Terminates on exactly one of: success, target_lost, collision, timeout,
irrecoverable_state ... The termination reason is recorded on every
episode ... episodes never terminate silently with an unclassified
reason."

Also implements the *shared* feasibility predicate from Section 10
(reachable/graspable/good_opportunity), used identically by episode
termination here and (unmodified) by B0/B1 and the analytic
OpportunityEstimator once those exist (M6+/M8) -- one implementation,
per confound register item #1's resolution mechanism, not
reimplemented per consumer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from opportunity_rl.sim.state import RobotState

TERMINATION_REASONS = (
    "success",
    "target_lost",
    "collision",
    "timeout",
    "irrecoverable_state",
)


@dataclass(frozen=True)
class FeasibilityConfig:
    """Thresholds for the Section 10 feasibility predicate. Deliberately
    config-driven, not hard-coded, so B0/B1/the opportunity estimator
    (M6-M8) can share this exact logic with different threshold configs
    rather than different implementations.

    Only the two thresholds the predicate actually reads are declared.
    Orientation-error and angular-speed thresholds were previously
    declared here but read nowhere -- the fast backend models neither
    (ee_orientation is IDENTITY_QUAT, angular velocities are zero), so a
    declared-but-ignored threshold advertised a gate that did not exist.
    They belong here again only when a backend genuinely enforces them."""

    workspace_radius_m: float = 0.85
    max_relative_speed_mps: float = 0.3


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _speed(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


@dataclass(frozen=True)
class Feasibility:
    distance_m: float
    relative_speed_mps: float
    reachable: bool
    good_opportunity: bool


def evaluate_feasibility(
    robot: RobotState,
    target_relative_position: tuple[float, float, float],
    target_relative_velocity: tuple[float, float, float],
    cfg: FeasibilityConfig,
) -> Feasibility:
    """`target_relative_position`/`_velocity` must already be in the
    vessel-base frame (the same frame RobotState.ee_position uses) --
    computing that frame conversion is the caller's job (it needs the
    vessel pose, which this function deliberately does not take, to keep
    the feasibility predicate itself simple and directly testable)."""
    distance = _distance(robot.ee_position, target_relative_position)
    ee_velocity_proxy = (0.0, 0.0, 0.0)  # see RobotState note: EE velocity not tracked separately
    rel_velocity = tuple(target_relative_velocity[i] - ee_velocity_proxy[i] for i in range(3))
    relative_speed = _speed(rel_velocity)  # type: ignore[arg-type]

    reachable = distance < cfg.workspace_radius_m
    good_opportunity = reachable and relative_speed < cfg.max_relative_speed_mps
    return Feasibility(
        distance_m=distance,
        relative_speed_mps=relative_speed,
        reachable=reachable,
        good_opportunity=good_opportunity,
    )


@dataclass(frozen=True)
class EpisodeConfig:
    max_duration_s: float = 60.0
    target_lost_radius_m: float = 3.0  # beyond this, the target is unreachable this episode
    # No collision_margin_m: the fast backend has no collision model, so a
    # declared margin would advertise a check nothing performs. It comes
    # back when a backend actually tests contact.


def check_termination(
    *,
    sim_time: float,
    episode_start_time: float,
    feasibility: Feasibility,
    episode_cfg: EpisodeConfig,
    grasp_succeeded: bool,
    collided: bool,
    irrecoverable: bool,
) -> str | None:
    """Returns a value from TERMINATION_REASONS, or None if the episode
    continues. Checked in a fixed priority order so exactly one reason
    is ever reported even if multiple conditions are simultaneously
    true (e.g. a collision on the final timeout step)."""
    if grasp_succeeded:
        return "success"
    if collided:
        return "collision"
    if irrecoverable:
        return "irrecoverable_state"
    if feasibility.distance_m > episode_cfg.target_lost_radius_m:
        return "target_lost"
    if sim_time - episode_start_time >= episode_cfg.max_duration_s:
        return "timeout"
    return None
