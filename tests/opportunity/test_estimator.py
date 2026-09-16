"""OpportunityEstimator tests -- numpy only, no gymnasium/torch.

The two properties that matter most: (1) the output is a well-formed
opportunity vector, and (2) 2nd-order extrapolation actually predicts an
upcoming window (a decelerating, approaching target should score HIGHER
in the future than now) -- because a constant-velocity model can't, and
predicting the deceleration is the whole point (docstring)."""

import numpy as np
import pytest

from opportunity_rl.opportunity.estimator import OpportunityConfig, OpportunityEstimator
from opportunity_rl.sim.state import EnvironmentState, RobotState, TargetState


def _robot(ee=(0.0, 0.0, 0.0)):
    return RobotState(
        ee_position=ee,
        ee_orientation=(0.0, 0.0, 0.0, 1.0),
        joint_positions=(0.0,) * 7,
        joint_velocities=(0.0,) * 7,
    )


def _target(pos, vel):
    return TargetState(
        position=pos,
        orientation=(0.0, 0.0, 0.0, 1.0),
        linear_velocity=vel,
        angular_velocity=(0.0, 0.0, 0.0),
    )


def _env(vessel_vel=(0.0, 0.0, 0.0)):
    return EnvironmentState(
        vessel_position=(0.0, 0.0, 0.0),
        vessel_orientation=(0.0, 0.0, 0.0, 1.0),
        vessel_linear_velocity=vessel_vel,
        vessel_angular_velocity=(0.0, 0.0, 0.0),
        sim_time=1.0,
    )


def test_output_shape_matches_horizons():
    est = OpportunityEstimator(OpportunityConfig(horizons_s=(0.5, 1.0, 1.5)))
    out = est(_robot(), _target((0.3, 0.0, 0.0), (0.0, 0.0, 0.0)), _env())
    assert out.shape == (3,)
    assert est.dim == 3


def test_scores_are_in_unit_interval():
    est = OpportunityEstimator()
    out = est(_robot(), _target((0.5, 0.2, 0.0), (0.1, 0.0, 0.0)), _env())
    assert np.all(out >= 0.0) and np.all(out <= 1.0)


def test_close_and_slow_target_scores_high_now_and_future():
    # Target right next to the EE, barely moving -> good opportunity that
    # persists; all horizons should score high.
    est = OpportunityEstimator()
    out = est(_robot(), _target((0.1, 0.0, 0.0), (0.01, 0.0, 0.0)), _env())
    assert np.all(out > 0.8)


def test_far_fast_target_scores_low():
    est = OpportunityEstimator()
    out = est(_robot(), _target((2.5, 0.0, 0.0), (1.5, 0.0, 0.0)), _env())
    assert np.all(out < 0.2)


def test_second_order_predicts_an_approaching_decelerating_window():
    # A target that is currently a bit far and moving fast toward the EE,
    # but decelerating, will be close-and-slow soon. A 2nd-order estimator
    # (which sees the deceleration) should score the FUTURE higher than
    # the present; a 1st-order (constant velocity) one cannot.
    est2 = OpportunityEstimator(OpportunityConfig(order=2, horizons_s=(0.5, 1.0, 1.5)))
    est1 = OpportunityEstimator(OpportunityConfig(order=1, horizons_s=(0.5, 1.0, 1.5)))
    est2.set_decision_dt(0.5)
    est1.set_decision_dt(0.5)

    # Feed two steps so the 2nd-order estimator can form an acceleration:
    # rel velocity goes from -1.0 (fast approach) to -0.5 (decelerating).
    # (target moving in -x toward the EE at the origin, slowing down.)
    for vel_x in (-1.0, -0.5):
        target = _target((1.0, 0.0, 0.0), (vel_x, 0.0, 0.0))
        out2 = est2(_robot(), target, _env())
        out1 = est1(_robot(), target, _env())

    # On the second call, 2nd-order sees the deceleration (accel =
    # +1.0 m/s^2, target slowing toward a turning point) and predicts a
    # transient close-and-slow window near the turning point -- so its
    # PEAK future opportunity across horizons is much higher than the
    # constant-velocity model's, which is blind to the deceleration and
    # just sees a target sailing past at constant speed. (The window
    # lands at a near horizon, not the far one, because the target
    # reverses; hence max-over-horizons, not a single horizon.)
    assert out2.max() > out1.max() + 0.2


def test_reset_clears_velocity_history():
    est = OpportunityEstimator(OpportunityConfig(order=2))
    est.set_decision_dt(0.5)
    est(_robot(), _target((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)), _env())
    est.reset()
    # After reset there is no past velocity, so the first call falls back
    # to order-1 (accel=0): the output must equal a fresh estimator's
    # first call on the same state.
    fresh = OpportunityEstimator(OpportunityConfig(order=2))
    fresh.set_decision_dt(0.5)
    same_state = _target((0.5, 0.0, 0.0), (-0.2, 0.0, 0.0))
    after_reset = est(_robot(), same_state, _env())
    fresh_out = fresh(_robot(), same_state, _env())
    assert np.array_equal(after_reset, fresh_out)


def test_only_reads_provided_state_deterministic():
    # Leakage guard (partial): the estimator's output is a pure function
    # of the states passed in -- calling twice with identical inputs (and
    # a reset between, to clear history) gives identical output. It has no
    # channel to the simulator's future.
    est = OpportunityEstimator()
    args = (_robot((0.1, 0.1, 0.0)), _target((0.4, 0.0, 0.0), (0.05, 0.0, 0.0)), _env())
    est.reset()
    a = est(*args)
    est.reset()
    b = est(*args)
    assert np.array_equal(a, b)


def test_distance_term_matches_the_feasibility_predicate():
    """The estimator is a smooth analogue of evaluate_feasibility, so at
    horizon d=0 (a zero-length extrapolation) its notion of distance must
    be the predicate's own -- i.e. EE to target, both in the vessel-base
    frame.

    Regression guard: the estimator used to subtract a vessel-base-frame
    ee_position from a WORLD-frame target position, leaving the vessel's
    displacement in the distance term. Anchoring the vessel away from the
    origin below is what makes that mistake visible at all -- with the
    vessel at (0,0,0) the two frames coincide and the bug hides.
    """
    from opportunity_rl.sim.episode import FeasibilityConfig, evaluate_feasibility

    robot = _robot((1.1, 0.3, 1.8))
    target = _target((6.4, -2.6, 2.1), (0.0, 0.0, 0.0))
    environment = EnvironmentState(
        vessel_position=(5.0, -3.0, 0.4),
        vessel_orientation=(0.0, 0.0, 0.0, 1.0),
        vessel_linear_velocity=(0.0, 0.0, 0.0),
        vessel_angular_velocity=(0.0, 0.0, 0.0),
        sim_time=1.0,
    )

    feas_cfg = FeasibilityConfig()
    target_in_vessel_frame = tuple(
        target.position[i] - environment.vessel_position[i] for i in range(3)
    )
    expected = evaluate_feasibility(
        robot, target_in_vessel_frame, (0.0, 0.0, 0.0), feas_cfg
    ).distance_m

    opp_cfg = OpportunityConfig(horizons_s=(0.0,), order=1)
    est = OpportunityEstimator(opp_cfg, feas_cfg)
    score = float(est(robot, target, environment)[0])

    # Rebuild the score the estimator should produce, from the predicate's
    # own distance. A stationary target makes the speed term a constant,
    # so any mismatch is a distance-term (i.e. frame) error.
    def sigmoid(x):
        return 1.0 / (1.0 + np.exp(-x))

    dist_width = opp_cfg.smoothness * feas_cfg.workspace_radius_m
    speed_width = opp_cfg.smoothness * feas_cfg.max_relative_speed_mps
    expected_score = sigmoid((feas_cfg.workspace_radius_m - expected) / dist_width) * sigmoid(
        feas_cfg.max_relative_speed_mps / speed_width
    )
    assert score == pytest.approx(float(expected_score), abs=1e-6)


def test_score_is_invariant_to_where_the_vessel_is_in_the_world():
    """Only relative geometry may affect opportunity. Translating the
    whole scene (vessel + target together) must leave the score
    unchanged; it did not when the two frames were mixed."""
    robot = _robot((1.1, 0.3, 1.8))

    def score_with_vessel_at(vessel_xyz):
        offsets = (1.4, 0.4, 1.7)
        target = _target(
            tuple(vessel_xyz[i] + off for i, off in enumerate(offsets)), (0.1, 0.0, 0.0)
        )
        environment = EnvironmentState(
            vessel_position=vessel_xyz,
            vessel_orientation=(0.0, 0.0, 0.0, 1.0),
            vessel_linear_velocity=(0.0, 0.0, 0.0),
            vessel_angular_velocity=(0.0, 0.0, 0.0),
            sim_time=1.0,
        )
        est = OpportunityEstimator()
        return est(robot, target, environment)

    at_origin = score_with_vessel_at((0.0, 0.0, 0.0))
    far_away = score_with_vessel_at((-532.0, 162.0, -0.07))  # a real sydney_regatta vessel pose
    assert np.allclose(at_origin, far_away, atol=1e-6)
