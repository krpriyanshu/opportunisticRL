"""Pure quaternion helpers -- no ROS2 imports, plain-pytest-testable.
Quaternion convention throughout: (x, y, z, w), matching
geometry_msgs/Quaternion field order (REP-103)."""

from __future__ import annotations

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]


def quat_conjugate(q: Quat) -> Quat:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)
