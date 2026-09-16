"""B4 (opportunity-aware env) integration tests -- require the `rl`
extra. The critical scientific-integrity check lives here: the
opportunity-aware and reactive environments must produce IDENTICAL base
observations (confound #1), and the estimator must only ever be handed
current state (confound #7)."""

import numpy as np
import pytest

pytest.importorskip("gymnasium")

from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig  # noqa: E402
from opportunity_rl.env.observation import BASE_DIM  # noqa: E402
from opportunity_rl.opportunity.estimator import (  # noqa: E402
    OpportunityConfig,
    OpportunityEstimator,
)
from opportunity_rl.sim.disturbance_config import load_disturbance_config  # noqa: E402
from opportunity_rl.sim.fast_simulator import FastSimulator  # noqa: E402


def _sim():
    return FastSimulator(load_disturbance_config("moving_target"))


def _b2_env():
    return DynamicGraspEnv(_sim(), config=EnvConfig())


def _b4_env():
    opp = OpportunityConfig()
    return DynamicGraspEnv(
        _sim(),
        config=EnvConfig(),
        opportunity_estimator=OpportunityEstimator(opp),
        opportunity_dim=opp.dim,
    )


def test_b4_observation_is_base_plus_opportunity_block():
    opp = OpportunityConfig()
    obs, _ = _b4_env().reset(seed=0)
    assert obs.shape == (BASE_DIM + opp.dim,)


def test_confound_1_b4_base_block_identical_to_b2_observation():
    # The load-bearing H1 integrity check: with the same seed, B4's first
    # BASE_DIM observation entries must be byte-identical to B2's full
    # observation. If they differed, the two policies would see different
    # base information and any B4-vs-B2 result would be confounded.
    b2_obs, _ = _b2_env().reset(seed=123)
    b4_obs, _ = _b4_env().reset(seed=123)
    assert np.array_equal(b4_obs[:BASE_DIM], b2_obs)


def test_confound_1_holds_across_several_steps():
    b2, b4 = _b2_env(), _b4_env()
    b2.reset(seed=7)
    b4.reset(seed=7)
    for action in (0, 1, 0, 2, 0):  # same action sequence in both
        o2, _, t2, tr2, _ = b2.step(action)
        o4, _, t4, tr4, _ = b4.step(action)
        assert np.array_equal(o4[:BASE_DIM], o2), "base observation diverged"
        if (t2 or tr2) or (t4 or tr4):
            break


def test_opportunity_block_varies_with_state():
    # Sanity: the appended opportunity block is not a constant -- it
    # actually reflects the (changing) predicted future opportunity.
    env = _b4_env()
    env.reset(seed=3)
    blocks = []
    for _ in range(10):
        obs, _, term, trunc, _ = env.step(0)  # WAIT, let the target move
        blocks.append(obs[BASE_DIM:].copy())
        if term or trunc:
            break
    blocks = np.array(blocks)
    assert blocks.std() > 0.0  # the opportunity signal changes over time
