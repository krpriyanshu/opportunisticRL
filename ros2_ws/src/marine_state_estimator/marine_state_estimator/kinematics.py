"""Pure kinematics helpers -- no ROS2 imports, plain-pytest-testable.

The vessel's world-frame *linear* velocity has no direct sensor source in
this simulation (no working odometry plugin -- see pose_stream.py's
docstring), so it is derived by finite-differencing successive
Gazebo-reported world positions. Angular velocity, by contrast, comes
directly from the vessel IMU's rate gyroscope (sensor_msgs/Imu.angular_velocity)
in state_estimator_node.py -- differentiating orientation quaternions
numerically is unnecessary and less accurate when a direct measurement
already exists.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TimedPosition:
    t: float  # seconds
    position: tuple[float, float, float]


def finite_difference_velocity(
    previous: TimedPosition, current: TimedPosition, min_dt: float = 1e-3
) -> tuple[float, float, float] | None:
    """Backward-difference velocity between two timestamped positions.

    Returns None if the time delta is non-positive or smaller than `min_dt`
    (guards against a divide-by-near-zero blowing up the estimate when two
    samples land at effectively the same timestamp)."""
    dt = current.t - previous.t
    if dt < min_dt:
        return None
    return tuple((c - p) / dt for c, p in zip(current.position, previous.position, strict=True))  # type: ignore[return-value]
