"""`DynamicGraspEnv` -- the Gymnasium environment the RL algorithms train
against (docs/rl-formulation.md's Interfaces section). Wraps any
`SimulationInterface` backend (M4's FastSimulator, or a future
ROS2/Gazebo one) so the RL layer is identical regardless of backend
(architecture.md's two-backend design).

Confound-register mechanisms realized here, not just described:
- #2 (decision frequency): a single `decision_period_s` config value,
  applied identically -- the env steps the simulation by exactly this
  much per env-step, so every baseline compared under H1 is stepped at
  the same decision rate by construction.
- #6 (reward): the one shared reward.compute_reward, config-driven.
- #1 (opportunity leakage / unequal info): the opportunity signal is an
  optional appended block (observation.append_opportunity). A reactive
  env has include_opportunity=False; an opportunity-aware env has it
  True with an injected estimator. Nothing else differs.
- #7 (future-info leakage): the estimator is called with the *current*
  RobotState/TargetState/EnvironmentState only. This env never passes an
  estimator any future simulator state -- structurally impossible here,
  since the env itself only holds the current step's state.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces

    _GYM_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only in no-rl-extra envs
    _GYM_AVAILABLE = False
    gym = None  # type: ignore[assignment]
    spaces = None  # type: ignore[assignment]

from opportunity_rl.env.observation import (
    NormConfig,
    ObservationSpec,
    append_opportunity,
    base_observation,
)
from opportunity_rl.env.reward import RewardConfig, compute_reward
from opportunity_rl.sim.episode import (
    EpisodeConfig,
    Feasibility,
    FeasibilityConfig,
    check_termination,
    evaluate_feasibility,
)
from opportunity_rl.sim.interface import SimulationInterface
from opportunity_rl.sim.state import EnvironmentState, RobotState, TargetState

# An OpportunityEstimator (M8) is any callable mapping current state to a
# fixed-length opportunity vector. Typed structurally so this env has no
# dependency on the estimator's eventual concrete class -- it is injected.
OpportunityEstimator = Callable[[RobotState, TargetState, EnvironmentState], np.ndarray]

# Of episode.TERMINATION_REASONS, the ones that are true MDP terminals.
# "timeout" is the sole truncation (an external time limit, not an
# absorbing state) -- see the note in step().
TERMINAL_REASONS = ("success", "collision", "irrecoverable_state", "target_lost")


@dataclass(frozen=True)
class EnvConfig:
    decision_period_s: float = 0.5  # confound #2: shared decision rate
    sim_substep_s: float = 0.05  # physics step; decision_period_s / this steps per env-step
    reward: RewardConfig = RewardConfig()
    feasibility: FeasibilityConfig = FeasibilityConfig()
    episode: EpisodeConfig = EpisodeConfig()
    norm: NormConfig = NormConfig()


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return float(np.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3))))


class DynamicGraspEnv(gym.Env if _GYM_AVAILABLE else object):  # type: ignore[misc]
    """The RL environment. Requires the `rl` extra (gymnasium) to
    instantiate; the observation/reward helper modules it uses are
    importable and testable without it."""

    # Explicit annotations: the conditional base class
    # (gym.Env-or-object) prevents mypy from inheriting gym.Env's own
    # attribute types, so these are annotated here directly.
    metadata: dict[str, list[str]] = {"render_modes": []}
    action_space: spaces.Discrete
    observation_space: spaces.Box

    def __init__(
        self,
        simulator: SimulationInterface,
        config: EnvConfig | None = None,
        opportunity_estimator: OpportunityEstimator | None = None,
        opportunity_dim: int = 0,
    ) -> None:
        if not _GYM_AVAILABLE:
            raise ImportError(
                "DynamicGraspEnv requires the 'rl' extra (gymnasium). "
                "Install with: pip install -e '.[rl]'"
            )
        super().__init__()
        self._sim = simulator
        self._cfg = config or EnvConfig()
        self._estimator = opportunity_estimator
        self._obs_spec = ObservationSpec(
            include_opportunity=opportunity_estimator is not None,
            opportunity_dim=opportunity_dim,
        )
        # A stateful estimator (e.g. one that finite-differences
        # acceleration across steps) needs the decision period to scale
        # that difference correctly; hand it over if it accepts one.
        if opportunity_estimator is not None and hasattr(opportunity_estimator, "set_decision_dt"):
            opportunity_estimator.set_decision_dt(self._cfg.decision_period_s)

        self.action_space = spaces.Discrete(4)  # WAIT/APPROACH/ATTEMPT_GRASP/ABORT
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._obs_spec.total_dim,), dtype=np.float32
        )

        self._episode_start_time = 0.0
        self._last_ee_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_action = 0

    # ---- Gymnasium API -------------------------------------------------
    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        self._sim.reset(seed=seed)
        # Reset any per-episode state the estimator holds (e.g. its
        # velocity history for the acceleration estimate) so no
        # information carries across episode boundaries.
        if self._estimator is not None and hasattr(self._estimator, "reset"):
            self._estimator.reset()
        env_state = self._sim.get_environment_state()
        self._episode_start_time = env_state.sim_time
        self._last_ee_position = self._sim.get_robot_state().ee_position
        self._last_action = 0
        return self._build_observation(), {}

    def step(self, action: int) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = int(action)
        prev_holding = self._sim.get_robot_state().gripper_holding_target
        self._sim.apply_action(action)

        n_substeps = max(1, round(self._cfg.decision_period_s / self._cfg.sim_substep_s))
        for _ in range(n_substeps):
            self._sim.step(self._cfg.sim_substep_s)

        robot = self._sim.get_robot_state()
        feasibility = self._current_feasibility(robot)

        grasp_succeeded = robot.gripper_holding_target and not prev_holding
        # Use the simulator's physical grasp_just_failed flag, not
        # "action was ATTEMPT_GRASP and not holding": with a multi-step
        # committed grasp maneuver (grasp_duration_steps > 1), a failure
        # can occur on a step whose action is WAIT (the maneuver is
        # committed and ignores the action), and an ATTEMPT_GRASP issued
        # *during* an in-progress maneuver is a no-op, not a failure. The
        # flag reports the real outcome in both the instant and multi-step
        # cases.
        grasp_failed = robot.grasp_just_failed
        collided = False  # the fast backend has no collision model yet (documented in gate report)

        reason = check_termination(
            sim_time=self._sim.get_environment_state().sim_time,
            episode_start_time=self._episode_start_time,
            feasibility=feasibility,
            episode_cfg=self._cfg.episode,
            grasp_succeeded=grasp_succeeded,
            collided=collided,
            irrecoverable=False,
        )
        # Gymnasium semantics, not "success vs everything else": `terminated`
        # means the MDP reached an absorbing state with no future reward,
        # `truncated` means an external cutoff stopped an episode that the
        # MDP itself would have continued. success/collision/
        # irrecoverable_state/target_lost are all genuine task terminals
        # (once the target has drifted out of range for good, no future
        # reward is reachable); only the max-duration timeout is a cutoff.
        # This distinction is load-bearing downstream: a value-based
        # learner must bootstrap through a truncation but not through a
        # termination (see ppo._collect_rollout's time-limit bootstrap).
        # Previously everything except success was reported as `truncated`.
        terminated = reason in TERMINAL_REASONS
        truncated = reason == "timeout"

        ee_motion = _distance(robot.ee_position, self._last_ee_position)
        reward_components = compute_reward(
            cfg=self._cfg.reward,
            grasp_succeeded=grasp_succeeded,
            grasp_failed=grasp_failed,
            collided=collided,
            ee_motion_magnitude=ee_motion,
        )
        self._last_ee_position = robot.ee_position
        self._last_action = action

        info: dict[str, Any] = {
            "termination_reason": reason,
            "reward_success": reward_components.success,
            "reward_failure": reward_components.failure,
            "reward_collision": reward_components.collision,
            "reward_time": reward_components.time,
            "reward_motion": reward_components.motion,
            "distance_to_target": feasibility.distance_m,
            "good_opportunity": feasibility.good_opportunity,
        }
        return (
            self._build_observation(),
            reward_components.total,
            terminated,
            truncated,
            info,
        )

    # ---- public: shared feasibility (for heuristic baselines B0/B1) ----
    def current_feasibility(self) -> Feasibility:
        """The Section 10 feasibility of the CURRENT state, via the one
        shared predicate. Exposed so heuristic baselines (B0/B1, M7) act
        on the same feasibility definition the environment's own
        termination uses, rather than reimplementing it -- confound
        register item #1's "one definition, not reimplemented three
        times". The learned policies (B2/B4) do NOT use this; they see
        only the flat observation vector. B0/B1 are simpler reference
        baselines, not the H1 comparison, so giving them the structured
        predicate output directly is legitimate and matches their
        rl-formulation.md definition."""
        return self._current_feasibility(self._sim.get_robot_state())

    # ---- internals -----------------------------------------------------
    def _current_feasibility(self, robot: RobotState) -> Feasibility:
        target_rel_pos = self._target_relative_to_vessel_position()
        target_rel_vel = self._target_relative_to_vessel_velocity()
        return evaluate_feasibility(robot, target_rel_pos, target_rel_vel, self._cfg.feasibility)

    def _target_relative_to_vessel_position(self) -> tuple[float, float, float]:
        target = self._sim.get_target_state()
        env = self._sim.get_environment_state()
        return tuple(target.position[i] - env.vessel_position[i] for i in range(3))  # type: ignore[return-value]

    def _target_relative_to_vessel_velocity(self) -> tuple[float, float, float]:
        target = self._sim.get_target_state()
        env = self._sim.get_environment_state()
        return tuple(target.linear_velocity[i] - env.vessel_linear_velocity[i] for i in range(3))  # type: ignore[return-value]

    def _build_observation(self) -> np.ndarray:
        robot = self._sim.get_robot_state()
        target = self._sim.get_target_state()
        env = self._sim.get_environment_state()
        rel_pos = self._target_relative_to_vessel_position()
        rel_vel = self._target_relative_to_vessel_velocity()
        feasibility = evaluate_feasibility(robot, rel_pos, rel_vel, self._cfg.feasibility)

        rel_to_ee = tuple(rel_pos[i] - robot.ee_position[i] for i in range(3))
        approach_speed = float(np.sqrt(sum(v**2 for v in rel_vel)))

        base = base_observation(
            robot=robot,
            target=target,
            environment=env,
            relative_target_position=rel_to_ee,  # type: ignore[arg-type]
            relative_target_velocity=rel_vel,
            distance_to_grasp_region=feasibility.distance_m,
            orientation_error=0.0,  # orientation control not modeled in the fast backend
            relative_approach_velocity=approach_speed,
            task_state=self._last_action,
            norm=self._cfg.norm,
        )
        if self._estimator is not None:
            # Confound #7: the estimator sees only current state -- this
            # env holds no future state to leak even if it wanted to.
            opportunity = self._estimator(robot, target, env)
            return append_opportunity(base, opportunity)
        return base
