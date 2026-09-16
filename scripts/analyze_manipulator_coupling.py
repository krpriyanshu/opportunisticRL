"""M1 diagnostic: does vessel motion propagate to the manipulator's end effector?

Structural argument first, empirical evidence second:

The manipulator is mounted on the vessel through exactly one joint
(``manipulator_mount_joint``, type ``fixed``, in
wamv_manipulator_description/urdf/wamv_with_arm.urdf.xacro), and no
controller is running yet (ros2_control is not installed at time of
writing -- see docs/implementation-status.md). A fixed joint has no
degrees of freedom, so there is no physical mechanism by which the
vessel could move while the arm's mount point does not move with it,
and with no controller there is nothing to compensate the resulting
motion away. So the end effector's world pose *must* track the vessel's
world pose, offset by whatever the (currently passive, gravity-driven)
arm posture happens to be.

This script measures that directly rather than just asserting it, by
combining two independently-sourced quantities every sample:

1. The vessel's world-frame pose, read from Gazebo's own pose broadcast
   (`gz topic -e -t /world/<world>/pose/info`) -- the entity named
   exactly "wamv" (a model-level entry, reported in world coordinates;
   nested *links* in that same topic, e.g. "panda_link7", are reported
   in a different, model-relative frame, which is why this script does
   NOT read the arm's pose from that topic).
2. The rigid-body offset from the vessel's base_link to the end effector
   (wamv/panda_hand), read from tf2 (populated by robot_state_publisher
   from /wamv/joint_states + the URDF -- this reflects the arm's actual
   simulated posture, including gravity sag).

and composing them (world_ee = world_vessel + R(world_vessel) @
local_offset) to get the end effector's true world-frame position.
"""

import re
import subprocess
import sys
import time

WORLD = "sydney_regatta"


def quat_rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]):
    """Rotate vector v by quaternion q=(x,y,z,w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 * cross(q_xyz, v)
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    # v' = v + qw * t + cross(q_xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def get_vessel_world_pose(
    ros2_env: dict,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    result = subprocess.run(
        ["gz", "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"],
        capture_output=True,
        text=True,
        timeout=10,
        env=ros2_env,
    )
    text = result.stdout
    idx = text.index('name: "wamv"\n')
    block = text[idx : idx + 400]
    pos_m = re.search(
        r"position\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)", block
    )
    ori_m = re.search(
        r"orientation\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)\s*w:\s*([-\d.eE]+)",
        block,
    )
    position = tuple(float(g) for g in pos_m.groups())
    if ori_m:
        orientation = tuple(float(g) for g in ori_m.groups())
    else:
        orientation = (0.0, 0.0, 0.0, 1.0)
    return position, orientation


def get_base_to_hand_offset(ros2_env: dict) -> tuple[float, float, float] | None:
    result = subprocess.run(
        [
            "timeout",
            "2",
            "stdbuf",
            "-oL",
            "ros2",
            "run",
            "tf2_ros",
            "tf2_echo",
            "wamv/wamv/base_link",
            "wamv/panda_hand",
        ],
        capture_output=True,
        text=True,
        timeout=6,
        env=ros2_env,
    )
    m = re.search(r"Translation:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]", result.stdout)
    if not m:
        return None
    return tuple(float(g) for g in m.groups())


def main() -> None:
    import os

    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    env = os.environ.copy()

    rows = []
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        t = time.monotonic() - start
        vessel_pos, vessel_ori = get_vessel_world_pose(env)
        offset = get_base_to_hand_offset(env)
        if offset is not None:
            world_ee = tuple(
                vp + c for vp, c in zip(vessel_pos, quat_rotate(vessel_ori, offset), strict=True)
            )
            rows.append((t, vessel_pos, world_ee))
            vw = f"vessel_world=({vessel_pos[0]:+8.3f},{vessel_pos[1]:+8.3f},{vessel_pos[2]:+6.3f})"
            ew = f"ee_world=({world_ee[0]:+8.3f},{world_ee[1]:+8.3f},{world_ee[2]:+6.3f})"
            print(f"t={t:6.2f}  {vw}  {ew}")

    if len(rows) < 2:
        print("Not enough samples captured.")
        sys.exit(1)

    def spread(vals):
        return max(vals) - min(vals)

    vessel_z = [r[1][2] for r in rows]
    ee_z = [r[2][2] for r in rows]
    vessel_x = [r[1][0] for r in rows]
    ee_x = [r[2][0] for r in rows]
    print(f"\ncaptured {len(rows)} samples over {duration_s:.0f}s")
    print(f"vessel world x span: {spread(vessel_x):.4f} m   z span: {spread(vessel_z):.4f} m")
    print(f"end-effector world x span: {spread(ee_x):.4f} m   z span: {spread(ee_z):.4f} m")


if __name__ == "__main__":
    main()
