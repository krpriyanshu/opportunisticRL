"""PPO smoke tests -- require the `rl` extra (torch + gymnasium), skipped
cleanly otherwise. These validate the training/eval/checkpoint *wiring*
runs end-to-end at a tiny scale; the actual "does it learn" validation is
the experiments/train_b2.py run (minutes, not a unit test), whose numbers
go in the M6 gate report.
"""

from pathlib import Path

import pytest

pytest.importorskip("torch")
pytest.importorskip("gymnasium")

from opportunity_rl.algorithms.interface import RLAlgorithm  # noqa: E402
from opportunity_rl.algorithms.ppo import PPO, PPOConfig  # noqa: E402
from opportunity_rl.algorithms.random_policy import RandomPolicy  # noqa: E402
from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig  # noqa: E402
from opportunity_rl.sim.disturbance_config import load_disturbance_config  # noqa: E402
from opportunity_rl.sim.fast_simulator import FastSimulator  # noqa: E402


def _env():
    return DynamicGraspEnv(
        FastSimulator(load_disturbance_config("sydney_regatta_default")), config=EnvConfig()
    )


def _ppo():
    # Tiny config: just enough to exercise one rollout + update cheaply.
    return PPO(_env(), config=PPOConfig(rollout_steps=128, epochs_per_update=2, seed=0))


def test_ppo_conforms_to_rl_algorithm_protocol() -> None:
    algo: RLAlgorithm = _ppo()  # mypy-checked structural conformance
    assert algo is not None


def test_train_runs_without_error():
    ppo = _ppo()
    ppo.train(total_steps=128)  # one rollout


def test_evaluate_returns_well_formed_result():
    ppo = _ppo()
    result = ppo.evaluate(n_episodes=3, seed=999)
    assert 0.0 <= result.success_rate <= 1.0
    assert result.n_episodes == 3
    assert sum(result.termination_reason_counts.values()) == 3


def test_checkpoint_roundtrip(tmp_path: Path):
    ppo = _ppo()
    ppo.train(total_steps=128)
    ckpt = tmp_path / "ckpt.pt"
    ppo.save_checkpoint(ckpt)
    assert ckpt.exists()
    # A fresh PPO can load it without error.
    ppo2 = _ppo()
    ppo2.load_checkpoint(ckpt)


def test_random_policy_evaluate_is_well_formed():
    result = RandomPolicy(_env(), seed=0).evaluate(n_episodes=3, seed=123)
    assert 0.0 <= result.success_rate <= 1.0
    assert result.n_episodes == 3
