"""Simplified end-effector kinematics for the fast simulator backend.

Deliberately NOT a 7-DOF joint-space arm simulation. Per
docs/research-question.md and architecture.md's non-goals ("no novel
low-level controller, no novel IK"), this project's contribution is the
timing decision (WAIT/APPROACH/ATTEMPT_GRASP/ABORT), not low-level
manipulation -- the real joint-space controller is
`marine_manipulation`'s conventional MoveIt2 pipeline (M3, validated
against Gazebo), which the RL policy hands off to and treats as a black
box (architecture.md Section 32 boundary). The fast backend only needs
enough of the manipulator to make the *timing* problem realistic: a
reachable-workspace bound and a velocity-limited approach to a
commanded point. `joint_positions`/`joint_velocities` on RobotState are
populated with a fixed, non-moving proxy value here (the resting
configuration) precisely to make clear they are not meaningfully
simulated at this layer -- a consumer that needs real joint angles
belongs on the ROS2/Gazebo backend, not this one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from opportunity_rl.sim.state import Vec3

# Matches moveit_resources_panda_moveit_config's initial_positions.yaml
# (M1), reused here only as a fixed placeholder value -- see module
# docstring for why this is not actually simulated.
RESTING_JOINT_POSITIONS: tuple[float, ...] = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)
ZERO_JOINT_VELOCITIES: tuple[float, ...] = (0.0,) * 7


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


@dataclass(frozen=True)
class ManipulatorConfig:
    reach_radius_m: float = 0.85  # matches the Panda's real ~0.855 m reach (M1)
    max_speed_mps: float = 0.3  # bounded approach speed


def clamp_to_reach(point: Vec3, mount_position: Vec3, reach_radius_m: float) -> Vec3:
    """If `point` (vessel-relative) is outside the reachable sphere
    around the arm's mount point, clamp it to the sphere's surface along
    the same direction -- used so a commanded approach target can never
    imply an unreachable pose."""
    offset = _sub(point, mount_position)
    dist = _norm(offset)
    if dist <= reach_radius_m or dist == 0.0:
        return point
    return _add(mount_position, _scale(offset, reach_radius_m / dist))


def step_toward(current: Vec3, target: Vec3, max_speed_mps: float, dt: float) -> tuple[Vec3, Vec3]:
    """One velocity-limited step of `current` toward `target`. Returns
    (new_position, velocity_used). A simple, deliberately non-smooth
    (no acceleration limit) kinematic proxy -- see module docstring for
    why this is sufficient for this project's purposes."""
    offset = _sub(target, current)
    dist = _norm(offset)
    max_step = max_speed_mps * dt
    if dist <= max_step or dist == 0.0:
        return target, _scale(offset, 1.0 / dt) if dt > 0 else (0.0, 0.0, 0.0)
    direction = _scale(offset, 1.0 / dist)
    velocity = _scale(direction, max_speed_mps)
    new_position = _add(current, _scale(velocity, dt))
    return new_position, velocity
