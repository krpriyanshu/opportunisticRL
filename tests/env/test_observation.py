"""Observation schema tests -- numpy only, no gymnasium/torch."""

import numpy as np

from opportunity_rl.env.observation import (
    BASE_DIM,
    NormConfig,
    ObservationSpec,
    append_opportunity,
    base_observation,
)
from opportunity_rl.sim.state import EnvironmentState, RobotState, TargetState


def _states():
    robot = RobotState(
        ee_position=(1.0, 0.5, 1.8),
        ee_orientation=(0.0, 0.0, 0.0, 1.0),
        joint_positions=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
        joint_velocities=(0.0,) * 7,
    )
    target = TargetState(
        position=(1.2, 0.5, 1.6),
        orientation=(0.0, 0.0, 0.0, 1.0),
        linear_velocity=(0.1, 0.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
    )
    env = EnvironmentState(
        vessel_position=(0.0, 0.0, 0.0),
        vessel_orientation=(0.0, 0.0, 0.0, 1.0),
        vessel_linear_velocity=(0.05, 0.0, 0.0),
        vessel_angular_velocity=(0.0, 0.0, 0.01),
        sim_time=5.0,
    )
    return robot, target, env


def _base():
    robot, target, env = _states()
    return base_observation(
        robot=robot,
        target=target,
        environment=env,
        relative_target_position=(0.2, 0.0, -0.2),
        relative_target_velocity=(0.05, 0.0, 0.0),
        distance_to_grasp_region=0.3,
        orientation_error=0.1,
        relative_approach_velocity=0.05,
        task_state=1,
        norm=NormConfig(),
    )


def test_base_observation_has_expected_dimension():
    obs = _base()
    assert obs.shape == (BASE_DIM,)
    assert obs.dtype == np.float32


def test_base_observation_is_finite():
    assert np.all(np.isfinite(_base()))


def test_task_state_one_hot_is_set_correctly():
    obs = _base()  # task_state=1
    one_hot = obs[-4:]
    assert list(one_hot) == [0.0, 1.0, 0.0, 0.0]


def test_append_opportunity_extends_by_opportunity_dim():
    base = _base()
    opportunity = np.array([0.7, 0.5, 0.3], dtype=np.float32)
    combined = append_opportunity(base, opportunity)
    assert combined.shape == (BASE_DIM + 3,)
    # The base block is unchanged -- opportunity is appended, not mixed.
    assert np.array_equal(combined[:BASE_DIM], base)
    assert np.array_equal(combined[BASE_DIM:], opportunity)


def test_observation_spec_total_dim():
    reactive = ObservationSpec(include_opportunity=False)
    aware = ObservationSpec(include_opportunity=True, opportunity_dim=5)
    assert reactive.total_dim == BASE_DIM
    assert aware.total_dim == BASE_DIM + 5


def test_confound_1_reactive_and_aware_differ_only_by_opportunity_block():
    # The structural check that confound #1 relies on: the opportunity-
    # aware observation's first BASE_DIM entries must be byte-identical
    # to the reactive observation. Anything else would mean the two
    # policies see different *base* information, invalidating H1.
    base = _base()
    aware = append_opportunity(base, np.array([0.9], dtype=np.float32))
    assert np.array_equal(aware[:BASE_DIM], base)
