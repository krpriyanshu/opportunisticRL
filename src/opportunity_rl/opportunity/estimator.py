"""The analytic `OpportunityEstimator` (M8).

Output: representation (2) from docs/rl-formulation.md -- a future
opportunity vector [O(t+d1), ..., O(t+dN)] over a fixed set of horizons,
each O in [0, 1], a smooth analogue of the Section 10 `good_opportunity`
predicate (high when the target is predicted to be reachable AND slow at
that future time).

How the future is predicted, and the binding leakage rule
(rl-formulation.md's "future-information leakage rule", confound #7):
this estimator NEVER reads the simulator's true future state. It sees
only the CURRENT relative position/velocity (and, for 2nd order, an
acceleration finite-differenced from the immediately preceding step) and
extrapolates:

    order 1 (constant velocity):
        rel_pos(t+d) = rel_pos + rel_vel * d
        rel_speed(t+d) = |rel_vel|                (constant)

    order 2 (constant acceleration):
        rel_pos(t+d) = rel_pos + rel_vel*d + 0.5*rel_acc*d^2
        rel_vel(t+d) = rel_vel + rel_acc*d
        rel_speed(t+d) = |rel_vel(t+d)|

Order 2 matters here: the good-opportunity windows in this project come
from the target *slowing at the turning points* of its wave-driven
oscillation, and a constant-velocity model is structurally blind to
"the target is decelerating and about to be slow" -- the exact thing an
anticipation signal needs to see. Acceleration is estimated by
finite-differencing the last two observed relative velocities (current +
one past step), which is current/past information only -- leakage-safe.

Statefulness: the estimator holds one past relative-velocity sample to
form the acceleration estimate, so it must be `reset()` at the start of
each episode. `DynamicGraspEnv` calls `reset()` on any estimator that
has one (see env.reset). On the first step of an episode there is no
past sample, so order-2 falls back to order-1 for that single step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from opportunity_rl.sim.episode import FeasibilityConfig
from opportunity_rl.sim.state import EnvironmentState, RobotState, TargetState

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class OpportunityConfig:
    horizons_s: tuple[float, ...] = (0.5, 1.0, 1.5, 2.0)
    order: int = 2  # 1 = constant velocity, 2 = constant acceleration
    # Smoothness of the sigmoid opportunity score around the feasibility
    # thresholds (fraction of the threshold used as the sigmoid width).
    smoothness: float = 0.25

    @property
    def dim(self) -> int:
        return len(self.horizons_s)


def _sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


class OpportunityEstimator:
    """Callable(robot, target, environment) -> ndarray of shape (dim,).
    Injected into DynamicGraspEnv as the opportunity signal (M9's B4)."""

    def __init__(
        self,
        config: OpportunityConfig | None = None,
        feasibility: FeasibilityConfig | None = None,
    ) -> None:
        self._cfg = config or OpportunityConfig()
        self._feas = feasibility or FeasibilityConfig()
        self._prev_rel_vel: Vec3 | None = None

    @property
    def dim(self) -> int:
        return self._cfg.dim

    def reset(self) -> None:
        self._prev_rel_vel = None

    def __call__(
        self, robot: RobotState, target: TargetState, environment: EnvironmentState
    ) -> np.ndarray:
        # Relative to the END EFFECTOR (what actually matters for a
        # grasp). `target.position` is WORLD frame while
        # `robot.ee_position` is VESSEL-BASE frame, so the target must be
        # converted into the vessel-base frame first -- the two are not
        # the same frame and the difference does NOT cancel. Subtracting
        # them directly (as a previous version did) left the vessel's own
        # wave displacement in the distance term, so this estimator's
        # notion of "how far is the target" disagreed with the
        # evaluate_feasibility predicate it is meant to be a smooth
        # analogue of by up to ~0.09 m against an 0.85 m threshold.
        # Regression-guarded by
        # test_distance_term_matches_the_feasibility_predicate.
        target_in_vessel_frame: Vec3 = (
            target.position[0] - environment.vessel_position[0],
            target.position[1] - environment.vessel_position[1],
            target.position[2] - environment.vessel_position[2],
        )
        rel_pos: Vec3 = (
            target_in_vessel_frame[0] - robot.ee_position[0],
            target_in_vessel_frame[1] - robot.ee_position[1],
            target_in_vessel_frame[2] - robot.ee_position[2],
        )
        # Relative velocity: target vs vessel (the end effector rides the
        # vessel; its own commanded motion is small and not modelled as a
        # velocity here). Matches evaluate_feasibility's relative-speed
        # notion.
        rel_vel: Vec3 = (
            target.linear_velocity[0] - environment.vessel_linear_velocity[0],
            target.linear_velocity[1] - environment.vessel_linear_velocity[1],
            target.linear_velocity[2] - environment.vessel_linear_velocity[2],
        )

        rel_acc: Vec3 = (0.0, 0.0, 0.0)
        if self._cfg.order >= 2 and self._prev_rel_vel is not None:
            # Acceleration = (this step's rel_vel - last step's rel_vel) /
            # decision_dt. The env calls this exactly once per decision
            # step and sets _decision_dt to its own decision period (via
            # set_decision_dt), so the difference is correctly scaled to
            # m/s^2. This is a current+past finite difference -- no future
            # state is read (leakage rule).
            dt = self._decision_dt
            rel_acc = (
                (rel_vel[0] - self._prev_rel_vel[0]) / dt,
                (rel_vel[1] - self._prev_rel_vel[1]) / dt,
                (rel_vel[2] - self._prev_rel_vel[2]) / dt,
            )
        self._prev_rel_vel = rel_vel

        scores = [self._score_at(rel_pos, rel_vel, rel_acc, d) for d in self._cfg.horizons_s]
        return np.asarray(scores, dtype=np.float32)

    # The env sets this so the acceleration finite-difference is scaled by
    # the real decision period rather than assumed to be 1 second.
    _decision_dt: float = 0.5

    def set_decision_dt(self, dt: float) -> None:
        self._decision_dt = dt

    def _score_at(self, rel_pos: Vec3, rel_vel: Vec3, rel_acc: Vec3, d: float) -> float:
        future_pos = (
            rel_pos[0] + rel_vel[0] * d + 0.5 * rel_acc[0] * d * d,
            rel_pos[1] + rel_vel[1] * d + 0.5 * rel_acc[1] * d * d,
            rel_pos[2] + rel_vel[2] * d + 0.5 * rel_acc[2] * d * d,
        )
        future_vel = (
            rel_vel[0] + rel_acc[0] * d,
            rel_vel[1] + rel_acc[1] * d,
            rel_vel[2] + rel_acc[2] * d,
        )
        distance = _norm(future_pos)
        speed = _norm(future_vel)

        dist_term = _sigmoid(
            (self._feas.workspace_radius_m - distance)
            / (self._cfg.smoothness * self._feas.workspace_radius_m)
        )
        speed_term = _sigmoid(
            (self._feas.max_relative_speed_mps - speed)
            / (self._cfg.smoothness * self._feas.max_relative_speed_mps)
        )
        return dist_term * speed_term
