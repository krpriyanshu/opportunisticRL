import pytest
from marine_state_estimator.kinematics import TimedPosition, finite_difference_velocity


def test_recovers_known_constant_velocity():
    p0 = TimedPosition(t=0.0, position=(0.0, 0.0, 0.0))
    p1 = TimedPosition(t=1.0, position=(2.0, -1.0, 0.5))
    v = finite_difference_velocity(p0, p1)
    assert v == pytest.approx((2.0, -1.0, 0.5))


def test_zero_dt_returns_none():
    p0 = TimedPosition(t=1.0, position=(0.0, 0.0, 0.0))
    p1 = TimedPosition(t=1.0, position=(1.0, 1.0, 1.0))
    assert finite_difference_velocity(p0, p1) is None


def test_negative_dt_returns_none():
    # out-of-order samples must not silently produce a sign-flipped velocity
    p0 = TimedPosition(t=2.0, position=(0.0, 0.0, 0.0))
    p1 = TimedPosition(t=1.0, position=(1.0, 0.0, 0.0))
    assert finite_difference_velocity(p0, p1) is None


def test_tiny_dt_below_threshold_returns_none():
    p0 = TimedPosition(t=0.0, position=(0.0, 0.0, 0.0))
    p1 = TimedPosition(t=1e-6, position=(1.0, 0.0, 0.0))
    assert finite_difference_velocity(p0, p1, min_dt=1e-3) is None
