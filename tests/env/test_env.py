"""DynamicGraspEnv integration tests. Require the `rl` extra (gymnasium);
skipped cleanly if it isn't installed, so the rest of the suite still
runs in a no-rl-extra environment (matching the offline-runnable
requirement -- the env's *logic* is tested in test_observation/test_reward
without gymnasium; these tests cover the Gymnasium wiring itself)."""

import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")

from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig  # noqa: E402
from opportunity_rl.env.observation import BASE_DIM  # noqa: E402
from opportunity_rl.sim.disturbance_config import load_disturbance_config  # noqa: E402
from opportunity_rl.sim.fast_simulator import FastSimulator  # noqa: E402


def _env(**kwargs):
    sim = FastSimulator(load_disturbance_config("sydney_regatta_default"))
    return DynamicGraspEnv(sim, config=EnvConfig(), **kwargs)


def test_conforms_to_gymnasium_api():
    env = _env()
    assert env.action_space.n == 4
    assert env.observation_space.shape == (BASE_DIM,)


def test_reset_returns_observation_of_correct_shape():
    env = _env()
    obs, info = env.reset(seed=0)
    assert obs.shape == (BASE_DIM,)
    assert obs.dtype == np.float32
    assert isinstance(info, dict)


def test_step_returns_five_tuple_with_correct_types():
    env = _env()
    env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(0)  # WAIT
    assert obs.shape == (BASE_DIM,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert "termination_reason" in info


def test_seeded_reset_is_reproducible():
    env1 = _env()
    env2 = _env()
    obs1, _ = env1.reset(seed=42)
    obs2, _ = env2.reset(seed=42)
    assert np.array_equal(obs1, obs2)


def test_episode_eventually_terminates_or_truncates():
    # Under an all-WAIT policy the target eventually drifts/times out --
    # the episode must not run forever.
    env = _env()
    env.reset(seed=1)
    done = False
    for _ in range(1000):
        _, _, terminated, truncated, _ = env.step(0)
        if terminated or truncated:
            done = True
            break
    assert done


def test_opportunity_aware_env_has_larger_observation():
    sim = FastSimulator(load_disturbance_config("sydney_regatta_default"))

    def estimator(robot, target, environment):
        return np.array([0.5, 0.5, 0.5], dtype=np.float32)

    env = DynamicGraspEnv(
        sim, config=EnvConfig(), opportunity_estimator=estimator, opportunity_dim=3
    )
    obs, _ = env.reset(seed=0)
    assert obs.shape == (BASE_DIM + 3,)
    # The base block matches a reactive env's observation exactly (confound #1).
    reactive = _env()
    reactive_obs, _ = reactive.reset(seed=0)
    assert np.array_equal(obs[:BASE_DIM], reactive_obs)


def test_successful_grasp_gives_positive_reward_and_terminates():
    # Configure a target right at the standby EE position with no motion,
    # so ATTEMPT_GRASP on the first step succeeds deterministically.
    from opportunity_rl.sim.fast_simulator import DEFAULT_STANDBY_POSITION
    from opportunity_rl.sim.target_dynamics import TargetConfig
    from opportunity_rl.sim.wave_disturbance import WaveConfig

    base_cfg = load_disturbance_config("sydney_regatta_default")
    cfg = type(base_cfg)(
        name=base_cfg.name,
        vessel_wave=WaveConfig(gain=0.0, amplitude=0.0),
        target=TargetConfig(
            start_position=DEFAULT_STANDBY_POSITION,
            drift_velocity=(0.0, 0.0, 0.0),
            wave=WaveConfig(gain=0.0, amplitude=0.0),
        ),
    )
    env = DynamicGraspEnv(FastSimulator(cfg), config=EnvConfig())
    env.reset(seed=0)
    _, reward, terminated, _, info = env.step(2)  # ATTEMPT_GRASP
    assert info["termination_reason"] == "success"
    assert terminated
    assert reward > 0


def test_timeout_is_truncation_not_termination():
    # Gymnasium semantics: the max-duration cutoff is the only truncation
    # in this env. Previously every non-success reason was reported as
    # truncated, which tells a value-based learner to bootstrap through
    # genuine terminal states.
    env = _env()
    env.reset(seed=1)
    for _ in range(1000):
        _, _, terminated, truncated, info = env.step(0)  # WAIT until the clock runs out
        if terminated or truncated:
            assert info["termination_reason"] == "timeout"
            assert truncated and not terminated
            break
    else:
        raise AssertionError("episode never ended")


def test_success_is_termination_not_truncation():
    from opportunity_rl.sim.fast_simulator import DEFAULT_STANDBY_POSITION
    from opportunity_rl.sim.target_dynamics import TargetConfig
    from opportunity_rl.sim.wave_disturbance import WaveConfig

    base_cfg = load_disturbance_config("sydney_regatta_default")
    cfg = type(base_cfg)(
        name=base_cfg.name,
        vessel_wave=WaveConfig(gain=0.0, amplitude=0.0),
        target=TargetConfig(
            start_position=DEFAULT_STANDBY_POSITION,
            drift_velocity=(0.0, 0.0, 0.0),
            wave=WaveConfig(gain=0.0, amplitude=0.0),
        ),
    )
    env = DynamicGraspEnv(FastSimulator(cfg), config=EnvConfig())
    env.reset(seed=0)
    _, _, terminated, truncated, info = env.step(2)  # ATTEMPT_GRASP
    assert info["termination_reason"] == "success"
    assert terminated and not truncated


def test_info_reports_every_reward_component():
    # All five named reward terms are inspectable for failure analysis
    # (rl-formulation.md Section 41); reward_collision used to be missing.
    env = _env()
    env.reset(seed=0)
    _, _, _, _, info = env.step(0)
    for key in (
        "reward_success",
        "reward_failure",
        "reward_collision",
        "reward_time",
        "reward_motion",
    ):
        assert key in info, key
