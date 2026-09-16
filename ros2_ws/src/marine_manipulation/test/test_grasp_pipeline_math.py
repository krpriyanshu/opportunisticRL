"""Plain pytest for the pure geometry helpers in grasp_pipeline.py."""

import pytest
from marine_manipulation.quaternion_math import quat_conjugate, quat_rotate


def test_quat_conjugate_negates_vector_part():
    assert quat_conjugate((0.1, 0.2, 0.3, 0.9)) == (-0.1, -0.2, -0.3, 0.9)


def test_identity_quaternion_rotate_is_noop():
    v = (1.0, 2.0, 3.0)
    assert quat_rotate((0.0, 0.0, 0.0, 1.0), v) == pytest.approx(v)


def test_rotate_then_rotate_by_conjugate_is_identity():
    # A somewhat arbitrary, non-trivial unit quaternion.
    import math

    axis = (0.0, 0.0, 1.0)
    angle = math.pi / 3
    s = math.sin(angle / 2)
    q = (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))
    v = (1.0, 0.5, -0.25)

    rotated = quat_rotate(q, v)
    back = quat_rotate(quat_conjugate(q), rotated)
    assert back == pytest.approx(v)


def test_90_degree_z_rotation_matches_known_result():
    import math

    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # +90 deg about Z
    result = quat_rotate(q, (1.0, 0.0, 0.0))
    assert result == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
