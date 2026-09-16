"""B0 and B1: the non-learned reactive baselines from
docs/rl-formulation.md's baseline ladder (Section 11). Both decide from
the CURRENT feasibility only (no memory, no future reasoning) -- they are
the "reactive" reference points H1's learned comparison (B2 reactive RL
vs B4 opportunity-aware RL) is contextualized against.

- B0 (immediate/heuristic): ATTEMPT_GRASP iff `good_opportunity` (the
  shared Section 10 predicate) is true right now, else WAIT. The
  simplest possible reactive rule.
- B1 (reactive heuristic): same shape, but with its OWN independently
  configurable distance/speed thresholds (a superset of B0 -- with
  thresholds equal to the feasibility predicate's it reduces to B0).
  This exists so "a tuned reactive heuristic" is a distinct data point
  from "the exact feasibility predicate", per Section 11.

Neither uses the reward at decision time (they're not learned), but
episodes are still scored with the same reward for comparable metrics
(Section 11). Both expose the same `evaluate()` signature as the RL
algorithms so the M7 evaluation matrix can treat every baseline
uniformly.

No torch here -- these are pure control rules. They need gymnasium only
transitively (through the env they evaluate against).
"""

from __future__ import annotations

import numpy as np

from opportunity_rl.algorithms.interface import EvaluationResult
from opportunity_rl.sim.interface import ATTEMPT_GRASP, WAIT


def _score_episodes(env, choose_action, n_episodes: int, seed: int) -> EvaluationResult:
    """Shared episode loop for the heuristic baselines. `choose_action`
    maps the env's current feasibility to an action. Reports the same
    EvaluationResult as the RL algorithms (identical metrics, so the
    matrix compares like with like)."""
    successes = 0
    returns: list[float] = []
    lengths: list[int] = []
    reason_counts: dict[str, int] = {}

    for ep in range(n_episodes):
        env.reset(seed=seed + ep)
        done = False
        ep_return = 0.0
        ep_len = 0
        while not done:
            feasibility = env.current_feasibility()
            action = choose_action(feasibility)
            _, reward, terminated, truncated, info = env.step(action)
            ep_return += reward
            ep_len += 1
            done = terminated or truncated
            if done:
                reason = info.get("termination_reason") or "none"
                reason_counts[reason] = reason_counts.get(reason, 0) + 1
                if reason == "success":
                    successes += 1
        returns.append(ep_return)
        lengths.append(ep_len)

    return EvaluationResult(
        n_episodes=n_episodes,
        success_rate=successes / n_episodes,
        mean_return=float(np.mean(returns)),
        mean_episode_length=float(np.mean(lengths)),
        termination_reason_counts=reason_counts,
    )


class B0ImmediatePolicy:
    """ATTEMPT_GRASP iff good_opportunity now, else WAIT."""

    def __init__(self, env) -> None:
        self._env = env

    def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult:
        return _score_episodes(
            self._env,
            lambda f: ATTEMPT_GRASP if f.good_opportunity else WAIT,
            n_episodes,
            seed,
        )


class B1ThresholdPolicy:
    """ATTEMPT_GRASP iff the target is within B1's own distance and speed
    thresholds, else WAIT. Defaults equal the feasibility predicate's, so
    by default B1 == B0; the point is that these are independently
    tunable (Section 11)."""

    def __init__(
        self,
        env,
        max_distance_m: float = 0.85,
        max_relative_speed_mps: float = 0.3,
    ) -> None:
        self._env = env
        self._max_distance_m = max_distance_m
        self._max_relative_speed_mps = max_relative_speed_mps

    def _decide(self, feasibility) -> int:
        within = (
            feasibility.distance_m < self._max_distance_m
            and feasibility.relative_speed_mps < self._max_relative_speed_mps
        )
        return ATTEMPT_GRASP if within else WAIT

    def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult:
        return _score_episodes(self._env, self._decide, n_episodes, seed)
