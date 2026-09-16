"""A uniform-random policy, used only as the floor that B2 (PPO) must
beat for M6 to count as "the standard RL baseline actually learns". Not
a baseline in the H1 ladder (B0-B4) -- it is the trivial control that
distinguishes "PPO learned something" from "PPO does no better than
chance". Implements the same evaluate() shape as RLAlgorithm so the two
are directly comparable.
"""

from __future__ import annotations

import numpy as np

from opportunity_rl.algorithms.interface import EvaluationResult


class RandomPolicy:
    def __init__(self, env, seed: int = 0) -> None:
        self._env = env
        self._rng = np.random.default_rng(seed)
        self._n_actions = int(env.action_space.n)

    def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult:
        successes = 0
        returns = []
        lengths = []
        reason_counts: dict[str, int] = {}

        for ep in range(n_episodes):
            obs, _ = self._env.reset(seed=seed + ep)
            done = False
            ep_return = 0.0
            ep_len = 0
            while not done:
                action = int(self._rng.integers(self._n_actions))
                obs, reward, terminated, truncated, info = self._env.step(action)
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
