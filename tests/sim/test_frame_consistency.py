"""Regression guards for the vessel/world frame conventions.

The simulator publishes target state in the WORLD frame and robot state in
the VESSEL-BASE frame. Two consumers derive a vessel-frame relative
position from that: the env (from the published TargetState /
EnvironmentState) and the simulator's own grasp gate (from its internal
trajectory). Those two must agree exactly -- if they drift apart, the
observation a policy sees and the criterion its grasp is judged by are
measuring different things, which is exactly the "one feasibility
definition, not reimplemented per consumer" invariant episode.py claims.

They did drift apart: get_target_state() offset the (already world-frame)
trajectory by the vessel's wave displacement, so the two disagreed by the
vessel's heave (~0.11 m peak under the shipped configs).
"""

import pytest

from opportunity_rl.sim.disturbance_config import load_disturbance_config
from opportunity_rl.sim.episode import FeasibilityConfig, evaluate_feasibility
from opportunity_rl.sim.fast_simulator import WAIT, FastSimulator

CONFIGS = ("sydney_regatta_default", "moving_target", "moving_target_severe")


def _env_derived_relative_position(sim: FastSimulator) -> tuple[float, float, float]:
    """Exactly what DynamicGraspEnv._target_relative_to_vessel_position
    computes, but written out here so this test depends only on the
    published SimulationInterface surface (no gymnasium import needed)."""
    target = sim.get_target_state()
    env = sim.get_environment_state()
    return tuple(target.position[i] - env.vessel_position[i] for i in range(3))


@pytest.mark.parametrize("config_name", CONFIGS)
def test_published_state_agrees_with_internal_relative_state(config_name):
    sim = FastSimulator(load_disturbance_config(config_name))
    for seed in range(5):
        sim.reset(seed=seed)
        for _ in range(40):
            from_published = _env_derived_relative_position(sim)
            from_internal = sim._target_relative_position(sim._t)
            assert from_published == pytest.approx(from_internal, abs=1e-9)
            sim.apply_action(WAIT)
            sim.step(0.5)


@pytest.mark.parametrize("config_name", CONFIGS)
def test_published_and_internal_relative_velocity_agree(config_name):
    sim = FastSimulator(load_disturbance_config(config_name))
    for seed in range(5):
        sim.reset(seed=seed)
        for _ in range(40):
            target = sim.get_target_state()
            env = sim.get_environment_state()
            from_published = tuple(
                target.linear_velocity[i] - env.vessel_linear_velocity[i] for i in range(3)
            )
            from_internal = sim._target_relative_velocity(sim._t)
            assert from_published == pytest.approx(from_internal, abs=1e-9)
            sim.apply_action(WAIT)
            sim.step(0.5)


@pytest.mark.parametrize("config_name", CONFIGS)
def test_grasp_gate_and_env_feasibility_never_disagree(config_name):
    """The consequence that actually matters: the good_opportunity answer
    derived from published state must match the one the simulator's own
    grasp gate uses, on every step."""
    sim = FastSimulator(load_disturbance_config(config_name))
    cfg = FeasibilityConfig()
    for seed in range(5):
        sim.reset(seed=seed)
        for _ in range(40):
            from_published = evaluate_feasibility(
                sim.get_robot_state(),
                _env_derived_relative_position(sim),
                sim._target_relative_velocity(sim._t),
                cfg,
            ).good_opportunity
            assert from_published == sim._good_opportunity_now()
            sim.apply_action(WAIT)
            sim.step(0.5)
