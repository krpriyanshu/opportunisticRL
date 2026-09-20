# ros2_ws

ROS2 (Jazzy) colcon workspace for the robotics and simulation layers.

Packages in `src/` (first-party):

| Package | Added at | Purpose |
|---|---|---|
| `wamv_manipulator_description` | M1 | Mounts a Franka Panda on VRX's WAM-V via a single fixed joint; adds the `gz_ros2_control` wiring. |
| `marine_state_estimator` | M2 | Publishes vessel odometry, filtered arm joint state, and the vessel-relative end-effector pose. |
| `wamv_manipulator_moveit_config` | M3 | MoveIt2 planning config for the vessel-mounted arm (SRDF with the virtual joint fixed to the vessel's base link rather than `world`). |
| `marine_manipulation` | M3 | Conventional grasp pipeline: plan + execute via MoveIt2, then a scripted `DetachableJoint` attach. Ships the target model. |

Third-party packages (`vrx`, `moveit_resources`) are fetched via vcstool
from the pinned refs in `*.repos` and are gitignored, not vendored:

```bash
vcs import src < vrx.repos
vcs import src < moveit_resources.repos
```

No package is added here before it has content to validate, on the principle that an empty ROS2 package is scaffolding with nothing to check against. Three packages named in early planning — `marine_sim_bringup`, `marine_rl_policy` and `marine_interfaces` — were consequently never created; see [docs/project-plan.md §3](../docs/project-plan.md#3-decisions-deferred-and-what-became-of-them).

### Tests

The ROS2-free halves of these packages (quaternion math, joint filtering,
Gazebo pose-text parsing, finite differencing) are covered by plain
pytest and run as part of the repository's ordinary `pytest` invocation —
`pyproject.toml` puts the two `ament_python` packages on `pythonpath`, so
no colcon build or ROS2 environment is needed for them.

Build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Python environment note.** ROS2 Jazzy on this system is built against system Python 3.12 (`/usr/bin/python3`). If your shell activates a different interpreter — a conda environment, for instance — `rclpy` fails to import with a `_rclpy_pybind11` extension-module error. Build and run ROS2 nodes with system Python active. This is why the RL research code in `src/opportunity_rl` and the ROS2 packages here are kept in separate Python environments by design: the RL training path does not require ROS2 at all.
