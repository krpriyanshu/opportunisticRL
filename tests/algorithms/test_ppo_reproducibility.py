"""Regression guards for PPO's determinism and train/eval isolation.

These cover three defects that all made a *reported* number untrustworthy
rather than making anything crash, which is why none of them were caught
by the existing smoke tests:

1. Training episodes were reset without a seed, so the simulator backend
   reseeded from OS entropy and two same-seed runs diverged.
2. `evaluate()` drives the same env the instance trains on and left
   `_obs` stale, so the next rollout recorded observations that did not
   match the states that produced their rewards.
3. Time-limit truncation was treated as termination, discarding the value
   of everything after the horizon.
"""

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("torch")
pytest.importorskip("gymnasium")

import torch  # noqa: E402

from opportunity_rl.algorithms.ppo import (  # noqa: E402
    PPO,
    TRAIN_SEED_BASE,
    PPOConfig,
    _compute_gae,
)
from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig  # noqa: E402
from opportunity_rl.sim.disturbance_config import load_disturbance_config  # noqa: E402
from opportunity_rl.sim.fast_simulator import FastSimulator  # noqa: E402

EVAL_SEED_OFFSET = 10_000


def _env():
    return DynamicGraspEnv(
        FastSimulator(load_disturbance_config("moving_target"), grasp_duration_steps=3),
        config=EnvConfig(),
    )


def _flat_weights(ppo: PPO) -> torch.Tensor:
    return torch.cat([p.detach().flatten() for p in ppo._net.parameters()])


def _train(seed: int, steps: int = 1024) -> PPO:
    ppo = PPO(_env(), config=PPOConfig(seed=seed, rollout_steps=256, epochs_per_update=2))
    ppo.train(steps)
    return ppo


def test_same_seed_training_is_bit_identical():
    """The headline reproducibility property. Before the fix these runs
    diverged, because every inter-episode env.reset() drew a fresh wave
    phase from OS entropy regardless of the configured seed."""
    assert torch.equal(_flat_weights(_train(0)), _flat_weights(_train(0)))


def test_different_seeds_still_diverge():
    """Guards against 'fixing' determinism by accidentally freezing the
    episode stream: different seeds must still explore differently."""
    assert not torch.equal(_flat_weights(_train(0)), _flat_weights(_train(1)))


def test_training_episode_seeds_never_collide_with_eval_seeds():
    """rl-formulation.md Section 13: training and evaluation never share a
    seed. Enforced by construction (disjoint bands), not by luck."""
    ppo = PPO(_env(), config=PPOConfig(seed=3, rollout_steps=64))
    train_seeds = {ppo._next_train_episode_seed() for _ in range(5_000)}
    eval_seeds = {3 + EVAL_SEED_OFFSET + ep for ep in range(1_000)}

    assert len(train_seeds) == 5_000, "training seeds must not repeat within a run"
    assert train_seeds.isdisjoint(eval_seeds)
    assert min(train_seeds) >= TRAIN_SEED_BASE


def test_evaluate_leaves_the_training_env_and_cached_obs_in_sync():
    """`evaluate()` drives the same env PPO trains on. Afterwards the
    cached observation must still describe the env's live state, or the
    next rollout records mismatched (obs, reward) pairs."""
    ppo = PPO(_env(), config=PPOConfig(seed=0, rollout_steps=64, epochs_per_update=1))
    ppo.train(64)
    ppo.evaluate(n_episodes=3, seed=EVAL_SEED_OFFSET)
    assert np.array_equal(ppo._obs, ppo._env._build_observation())


def test_interleaved_train_and_evaluate_is_reproducible():
    """The exact pattern learning_curve_h1.py uses at every budget
    boundary: train, evaluate, train again."""

    def run():
        ppo = PPO(_env(), config=PPOConfig(seed=0, rollout_steps=128, epochs_per_update=1))
        ppo.train(128)
        first = ppo.evaluate(n_episodes=3, seed=EVAL_SEED_OFFSET).mean_return
        ppo.train(128)
        second = ppo.evaluate(n_episodes=3, seed=EVAL_SEED_OFFSET).mean_return
        return first, second, _flat_weights(ppo)

    a_first, a_second, a_w = run()
    b_first, b_second, b_w = run()
    assert a_first == b_first
    assert a_second == b_second
    assert torch.equal(a_w, b_w)


def test_gae_bootstraps_through_a_time_limit_truncation():
    """A truncated step's reward carries gamma * V(final_obs) (folded in
    by _collect_rollout), so its advantage reflects the future the cutoff
    hid. Treating truncation as termination instead -- the old behaviour
    -- systematically depressed values at the horizon, and with these
    episodes almost all ending in timeout that was the common case."""
    gamma, lam = 0.99, 0.95
    values = np.array([0.5, 0.5], dtype=np.float32)
    dones = np.array([1.0, 0.0], dtype=np.float32)
    final_value = 2.0

    as_termination, _ = _compute_gae(
        np.array([1.0, 0.0], dtype=np.float32), values, dones, 0.0, gamma, lam
    )
    # What _collect_rollout now builds for a truncated step.
    as_truncation, _ = _compute_gae(
        np.array([1.0 + gamma * final_value, 0.0], dtype=np.float32),
        values,
        dones,
        0.0,
        gamma,
        lam,
    )
    assert as_truncation[0] == pytest.approx(as_termination[0] + gamma * final_value)


def test_checkpoint_roundtrip_still_works_after_evaluate(tmp_path: Path):
    ppo = PPO(_env(), config=PPOConfig(seed=0, rollout_steps=64, epochs_per_update=1))
    ppo.train(64)
    ppo.evaluate(n_episodes=2, seed=EVAL_SEED_OFFSET)
    ckpt = tmp_path / "ckpt.pt"
    ppo.save_checkpoint(ckpt)
    other = PPO(_env(), config=PPOConfig(seed=1, rollout_steps=64))
    other.load_checkpoint(ckpt)
    assert torch.equal(_flat_weights(ppo), _flat_weights(other))
