"""B0/B1 heuristic baseline tests. Require the `rl` extra (gymnasium, for
the env); the policy logic itself is trivial control, but it only runs
against a real env."""

import pytest

pytest.importorskip("gymnasium")

from opportunity_rl.algorithms.heuristic import (  # noqa: E402
    B0ImmediatePolicy,
    B1ThresholdPolicy,
)
from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig  # noqa: E402
from opportunity_rl.sim.disturbance_config import load_disturbance_config  # noqa: E402
from opportunity_rl.sim.fast_simulator import FastSimulator  # noqa: E402


def _env(disturbance="moving_target"):
    return DynamicGraspEnv(FastSimulator(load_disturbance_config(disturbance)), config=EnvConfig())


def test_b0_evaluate_is_well_formed():
    result = B0ImmediatePolicy(_env()).evaluate(n_episodes=10, seed=0)
    assert 0.0 <= result.success_rate <= 1.0
    assert result.n_episodes == 10
    assert sum(result.termination_reason_counts.values()) == 10


def test_b1_with_default_thresholds_matches_b0():
    # B1's defaults equal the feasibility predicate's thresholds, so with
    # the same seeds B1 should behave identically to B0.
    seed = 42
    b0 = B0ImmediatePolicy(_env()).evaluate(n_episodes=20, seed=seed)
    b1 = B1ThresholdPolicy(_env()).evaluate(n_episodes=20, seed=seed)
    assert b0.success_rate == b1.success_rate
    assert b0.mean_return == pytest.approx(b1.mean_return)


def test_b1_stricter_speed_threshold_changes_behavior():
    # A much stricter speed threshold makes B1 wait for calmer moments --
    # it should behave differently from the default (this is the whole
    # point of B1 being independently tunable).
    seed = 7
    default = B1ThresholdPolicy(_env()).evaluate(n_episodes=20, seed=seed)
    strict = B1ThresholdPolicy(_env(), max_relative_speed_mps=0.05).evaluate(
        n_episodes=20, seed=seed
    )
    # At minimum the returns should differ; a stricter grasp gate changes
    # when (or whether) the policy commits.
    assert default.mean_return != strict.mean_return


def test_b0_beats_random_on_the_hard_config():
    # On moving_target, B0 (grasp only when good_opportunity) should
    # decisively beat a blind random policy on return -- it never makes a
    # doomed grasp attempt, so it avoids the failed-attempt penalty.
    from opportunity_rl.algorithms.random_policy import RandomPolicy

    seed = 0
    b0 = B0ImmediatePolicy(_env()).evaluate(n_episodes=50, seed=seed)
    rand = RandomPolicy(_env(), seed=seed).evaluate(n_episodes=50, seed=seed)
    assert b0.mean_return > rand.mean_return
