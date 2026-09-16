"""Filter a combined JointState (vessel joints + arm joints, as published by
robot_state_publisher on /wamv/joint_states) down to just the manipulator's
own arm joints, in a fixed order.

/wamv/joint_states carries every named joint in the URDF -- the Panda arm's
7 revolute joints, its 2 finger joints, and several vessel-internal fixed
sensor-mount and thruster/propeller joints that are not manipulator DOF at
all (verified empirically: 9 wamv/* joints alongside the 9 panda_*
joints -- see docs/ros2.md). rl-formulation.md's observation schema wants
"joint_state (2*N) for N manipulator joints" -- this is exactly N=7 (the
arm's revolute joints; the two finger joints are gripper state, not part of
the arm's own configuration for grasp-timing purposes), in a fixed,
documented order so the resulting vector's indexing is stable across runs.

No ROS2 imports -- operates on plain parallel lists (names/positions/
velocities), the same shape rclpy hands back from a sensor_msgs/JointState,
so it is directly testable without constructing a real message type.
"""

from __future__ import annotations

ARM_JOINT_ORDER: tuple[str, ...] = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)


def filter_arm_joints(
    names: list[str], positions: list[float], velocities: list[float]
) -> tuple[list[str], list[float], list[float]]:
    """Return (names, positions, velocities) reordered to ARM_JOINT_ORDER,
    keeping only those joints. Raises ValueError if any expected arm joint
    is missing from `names` -- silently returning a partial/reordered
    vector would be worse than failing loudly, since a missing joint would
    otherwise shift every downstream index."""
    index = {n: i for i, n in enumerate(names)}
    missing = [j for j in ARM_JOINT_ORDER if j not in index]
    if missing:
        raise ValueError(f"missing expected arm joints in input: {missing}")

    out_names = list(ARM_JOINT_ORDER)
    out_positions = [positions[index[j]] for j in ARM_JOINT_ORDER]
    out_velocities = [velocities[index[j]] for j in ARM_JOINT_ORDER]
    return out_names, out_positions, out_velocities
