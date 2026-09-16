import pytest
from marine_state_estimator.joint_filter import ARM_JOINT_ORDER, filter_arm_joints


def test_filters_and_reorders():
    # Deliberately shuffled and interleaved with non-arm joints, mirroring
    # the real /wamv/joint_states ordering (which is not alphabetical or
    # otherwise predictable).
    names = [
        "panda_finger_joint1",
        "wamv/left_chassis_engine_joint",
        "panda_joint3",
        "panda_joint1",
        "wamv/gps_wamv_joint",
        "panda_joint7",
        "panda_joint2",
        "panda_joint5",
        "panda_joint4",
        "panda_finger_joint2",
        "panda_joint6",
    ]
    positions = list(range(len(names)))
    velocities = [p * 10 for p in positions]

    out_names, out_positions, out_velocities = filter_arm_joints(names, positions, velocities)

    assert tuple(out_names) == ARM_JOINT_ORDER
    for name, pos, vel in zip(out_names, out_positions, out_velocities, strict=True):
        expected_idx = names.index(name)
        assert pos == expected_idx
        assert vel == expected_idx * 10


def test_missing_joint_raises():
    names = list(ARM_JOINT_ORDER[:-1])  # drop panda_joint7
    positions = [0.0] * len(names)
    velocities = [0.0] * len(names)
    with pytest.raises(ValueError, match="panda_joint7"):
        filter_arm_joints(names, positions, velocities)
