"""Parse a single Gazebo Transport ``gz.msgs.Pose_V`` text dump (as printed
by ``gz topic -e -t <topic> -n 1``) and extract one named top-level entity's
pose.

Why this exists at all, not just TF2: in this VRX setup, ROS2 TF2 only
covers the robot's own kinematic chain rooted at ``wamv/wamv/base_link`` --
there is no ``world``/``map`` frame connecting the vessel to a fixed
reference (verified empirically, not assumed; see docs/ros2.md and the
"vessel world pose" gotcha in docs/environment.md). The vessel's world-frame
pose is only available via Gazebo Transport's ``/world/<world>/pose/info``
topic, and bridging that topic into ROS2 with ``ros_gz_bridge`` loses the
per-entity ``name`` field (verified: every ``frame_id``/``child_frame_id``
comes back empty), which makes the bridged version useless for reliably
picking out "wamv" among the many per-link sub-entries in the same message.
This module is the pure-parsing half of the workaround: read the CLI text
form directly (which does retain ``name: "..."``), find the top-level block
matching the requested entity name, and return its position/orientation.

No ROS2 imports here on purpose -- this is plain, ordinary Python, testable
with plain ``pytest`` in any environment, including a shell that has never
sourced ROS2's ``setup.bash``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

_POSITION_RE = re.compile(r"position\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)")
_ORIENTATION_RE = re.compile(
    r"orientation\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)\s*w:\s*([-\d.eE]+)"
)
# A generous window: enough to contain one entity's position+orientation
# blocks, small enough to not accidentally cross into the *next* entity's
# "pose {" block when short link names repeat many times in one dump.
_ENTITY_WINDOW_CHARS = 400


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]  # x, y, z, w


def parse_named_pose(text: str, entity_name: str) -> Pose | None:
    """Return the top-level pose for `entity_name` in one Pose_V text dump,
    or None if that name doesn't appear. Only matches an exact,
    newline-terminated ``name: "<entity_name>"`` line, so e.g. requesting
    "wamv" will not match a link named "wamv_something"."""
    needle = f'name: "{entity_name}"\n'
    idx = text.find(needle)
    if idx == -1:
        return None
    window = text[idx : idx + _ENTITY_WINDOW_CHARS]

    pos_m = _POSITION_RE.search(window)
    if pos_m is None:
        return None
    position = tuple(float(g) for g in pos_m.groups())

    ori_m = _ORIENTATION_RE.search(window)
    orientation = tuple(float(g) for g in ori_m.groups()) if ori_m else (0.0, 0.0, 0.0, 1.0)

    return Pose(position=position, orientation=orientation)  # type: ignore[arg-type]
