import pytest

from opportunity_rl.sim.manipulator_proxy import clamp_to_reach, step_toward


def test_point_within_reach_is_unchanged():
    mount = (0.0, 0.0, 0.0)
    point = (0.3, 0.2, 0.1)
    assert clamp_to_reach(point, mount, reach_radius_m=0.85) == point


def test_point_outside_reach_is_clamped_to_sphere_surface():
    mount = (0.0, 0.0, 0.0)
    point = (2.0, 0.0, 0.0)
    clamped = clamp_to_reach(point, mount, reach_radius_m=0.85)
    assert clamped == pytest.approx((0.85, 0.0, 0.0))


def test_clamp_preserves_direction_from_mount():
    mount = (1.0, 1.0, 1.0)
    point = (1.0, 1.0, 5.0)  # straight up from mount, distance 4
    clamped = clamp_to_reach(point, mount, reach_radius_m=0.5)
    assert clamped == pytest.approx((1.0, 1.0, 1.5))


def test_step_toward_reaches_target_when_close_enough():
    new_pos, vel = step_toward((0.0, 0.0, 0.0), (0.01, 0.0, 0.0), max_speed_mps=0.3, dt=0.1)
    assert new_pos == (0.01, 0.0, 0.0)


def test_step_toward_moves_at_bounded_speed_when_far():
    current = (0.0, 0.0, 0.0)
    target = (10.0, 0.0, 0.0)
    new_pos, vel = step_toward(current, target, max_speed_mps=0.3, dt=1.0)
    assert new_pos == pytest.approx((0.3, 0.0, 0.0))
    assert vel == pytest.approx((0.3, 0.0, 0.0))


def test_step_toward_never_overshoots():
    current = (0.0, 0.0, 0.0)
    target = (0.05, 0.0, 0.0)
    new_pos, _ = step_toward(current, target, max_speed_mps=0.3, dt=1.0)
    # max_step (0.3) > distance (0.05): should land exactly on target, not overshoot.
    assert new_pos == (0.05, 0.0, 0.0)
