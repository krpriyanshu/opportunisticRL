"""The reward from docs/rl-formulation.md's Reward section:

    R_t = R_success * 1[success]
          - lambda_f * C_failure
          - lambda_c * C_collision
          - lambda_t * C_time
          - lambda_m * C_motion

ONE implementation, parameterized by config, shared across every
RL-trained baseline -- this is the mechanism resolving confound register
item #6 (reward differs between baselines). Not reimplemented per
baseline; a baseline that wants a different reward changes the config,
which is recorded in the run metadata, not the code.

Two binding requirements from the spec, enforced here:
- C_time is charged EVERY step regardless of action (including WAIT), so
  indefinite waiting has a nonzero opportunity cost -- "waiting forever
  must not be optimal". Unit-tested directly (an all-WAIT episode's
  return must be strictly less than a successful episode's).
- No shaping beyond these five named terms. If shaping is later needed,
  it is added as a separate named, ablatable term, not folded into an
  existing coefficient.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RewardConfig:
    r_success: float = 1.0
    lambda_failure: float = 0.5
    lambda_collision: float = 1.0
    lambda_time: float = 0.01  # per-step; small so it accrues over an episode
    lambda_motion: float = 0.001


@dataclass(frozen=True)
class RewardComponents:
    """Every term of one step's reward, kept separate (not just the
    scalar sum) so failure analysis and ablations can inspect which term
    drove a decision -- rl-formulation.md Section 41."""

    success: float
    failure: float
    collision: float
    time: float
    motion: float

    @property
    def total(self) -> float:
        return self.success - self.failure - self.collision - self.time - self.motion


def compute_reward(
    *,
    cfg: RewardConfig,
    grasp_succeeded: bool,
    grasp_failed: bool,
    collided: bool,
    ee_motion_magnitude: float,
) -> RewardComponents:
    """`grasp_failed` is a *failed* ATTEMPT_GRASP (committed and missed),
    distinct from merely not attempting -- the penalty is for a bad
    commitment, not for waiting. `ee_motion_magnitude` is the end
    effector's displacement this step (meters), penalized lightly so the
    policy doesn't thrash, per C_motion."""
    return RewardComponents(
        success=cfg.r_success if grasp_succeeded else 0.0,
        failure=cfg.lambda_failure if grasp_failed else 0.0,
        collision=cfg.lambda_collision if collided else 0.0,
        time=cfg.lambda_time,  # charged every step, unconditionally
        motion=cfg.lambda_motion * ee_motion_magnitude,
    )
