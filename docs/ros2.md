# ROS2 State Pipeline: Frames, Conventions and Topic Contract

This document records the TF2 frame tree, the coordinate and unit conventions, and the `marine_state_estimator` topic contract. Every finding below was established by inspecting the running simulation rather than by reading upstream documentation, and the distinction matters: three of the four candidate mechanisms for obtaining vessel world-frame state are documented upstream as working and do not work here.

## 1. TF2 frame tree, established by inspection

**Finding, established by direct inspection (`ros2 topic echo /tf_static`), not by reading VRX's docs:** `wamv/wamv/base_link` is the **root** of the entire TF2 tree published by `robot_state_publisher` in this simulation. It appears only as a `frame_id` (parent) in every static transform, never as a `child_frame_id` — nothing connects it to a `world`/`map`/`odom` frame. Concretely:

```text
wamv/wamv/base_link  (TF root — no parent frame exists)
├── wamv/panda_link0 → panda_link1 → ... → panda_link7 → panda_link8 → panda_hand
│     (Panda arm chain — 7 revolute joints + fixed hand mount; no "wamv/" prefix,
│      since the Panda's own xacro doesn't bake in a namespace the way WAM-V's does)
├── wamv/wamv/front_left_camera_post_link → ... → front_left_camera_link → _optical
├── wamv/wamv/front_right_camera_post_link → ... (same pattern)
├── wamv/wamv/middle_right_camera_post_link → ... (same pattern)
├── wamv/wamv/lidar_wamv_post_link → lidar_wamv_post_arm_link → lidar_wamv_link
├── wamv/wamv/imu_wamv_link → imu_wamv_link/imu_wamv_sensor
├── wamv/wamv/gps_wamv_link → gps_wamv_link/navsat
└── wamv/wamv/{left,right}_battery_link, cpu_cases_link  (fixed mounts)
```

**The double `wamv/wamv/` prefix is a pre-existing VRX quirk** (documented first in `docs/implementation-status.md` during M1): `robot_state_publisher` runs in the `/wamv` ROS2 namespace (prefixing every frame_id with `wamv/`), and WAM-V's own xacro *also* bakes `${namespace}` (`wamv`) into its link names, so WAM-V's links end up double-prefixed while the Panda's links (no baked-in namespace) get exactly one prefix. Not something this project introduced or should "fix" — VRX's own tooling and this project's code both already account for it.

**There is no world-fixed TF frame at all in this simulation.** This was checked directly, not assumed: `ros2 run tf2_ros tf2_echo world wamv/wamv/base_link` never resolves, and `/tf_static` confirms `wamv/wamv/base_link` has no parent. The vessel's world-frame pose is **only** available via Gazebo Transport (`gz topic -e -t /world/<world>/pose/info`), not through ROS2 TF2 — see the "Vessel world-frame state has no ROS2-native source" section below for the full finding and what `marine_state_estimator` does about it.

**Practical consequence:** any "world frame" pose/orientation this project publishes over ROS2 (e.g. `marine/vessel/odom`'s `frame_id: world`) is a **nominal** label, not a TF2-resolvable frame — `tf2_ros` cannot `lookup_transform` from it. Don't write code that assumes otherwise (e.g. `tf_buffer.lookup_transform("world", ...)` will always fail here). Consumers that need the vessel's world pose must read `marine/vessel/odom` directly, not go through TF2.

## 2. Vessel world-frame state has no ROS2-native source

Four candidate mechanisms were checked empirically before a workaround was adopted. Each is documented or implied upstream to provide the vessel's world pose; none does so in this environment:

1. **TF2** — ruled out above (no world-connected frame exists).
2. **`/wamv/pose`** (`tf2_msgs/msg/TFMessage`, bridged from Gazebo's `/model/wamv/pose`) — inspected directly via `ros2 topic echo`, and it does **not** contain the vessel's world pose. It carries parent-link-relative sensor-mount transforms (e.g. `wamv/wamv/imu_wamv_link → .../imu_wamv_sensor`), essentially overlapping with `/tf_static`'s own content — a `Pose_V` message scoped to *one model's internal links*, not the model's own world-frame pose.
3. **VRX's own `OdometryPublisher` Gazebo plugin** (`gz::sim::systems::OdometryPublisher`, wired into `wamv_gazebo.urdf.xacro` via the `wamv_p3d` macro — this is a real, already-shipped ground-truth-odometry plugin, not something missing from VRX) — **confirmed present and correctly configured in the xacro, but does not actually publish anything in this environment.** The plugin's `.so` exists on disk at the expected path (`libgz-sim8-odometry-publisher-system.so`), no error appears in the simulation log, and no `odometry` topic ever appears in `gz topic -l`. Root cause not fully diagnosed (a silent core-plugin failure, unlike every other gotcha in `docs/environment.md`, which all had an identifiable cause) — recorded here as an open item rather than glossed over. Worth revisiting if a future milestone needs higher-rate or lower-latency vessel velocity than the workaround below provides.
4. **Bridging `/world/<world>/pose/info` (`gz.msgs.Pose_V`) directly** via `ros_gz_bridge` — works in the sense that the topic appears and carries data, but the bridge's `Pose_V → tf2_msgs/msg/TFMessage` conversion **drops every entity's name** (`frame_id`/`child_frame_id` both come back empty on every entry, verified by echoing it). Since this message lists *every* model and sub-link in the world as an unordered/unnamed sequence, there is no reliable way to pick out "wamv" from a bridged instance of this topic.

**Working solution, implemented in `marine_state_estimator`:** poll `gz topic -e -t /world/<world>/pose/info -n 1` (the plain CLI tool, which *does* retain names in its text output) from a dedicated background thread, parse the "wamv" entry's position/orientation (`pose_stream.parse_named_pose`, pure Python, unit-tested), and finite-difference successive position samples for linear velocity (`kinematics.finite_difference_velocity`). Angular velocity comes directly from the vessel IMU (`sensor_msgs/Imu.angular_velocity`) instead of differentiating orientation quaternions, since a direct rate-gyro measurement is more accurate and was already available.

**Measured cost of this workaround** (relevant to anyone tuning the `publish_rate_hz` parameter): each `gz topic -e -n 1` call costs 130–350ms, dominated by Gazebo Transport discovery/process-startup overhead, not wait-for-message time. This bounds the vessel odometry topic to roughly **3–7 Hz** in practice, regardless of the node's publish timer rate — confirmed by measurement (`ros2 topic hz /marine/vessel/odom` ≈ 3.1 Hz), not assumed. The node explicitly detects and skips a stale (already-published) poller sample rather than republishing a false near-zero velocity at the timer's full rate — see the comment above `_publish_vessel_odom` in `state_estimator_node.py` for why a naive "just publish every timer tick" implementation would have silently produced fake data most ticks. The other two topics (`marine/robot/arm_joint_state`, `marine/robot/ee_pose`) do not depend on this poller and publish at the full configured rate (default 10 Hz).

## 3. `marine_state_estimator` topic contract

| Topic | Type | Rate | Frame | Source |
|---|---|---|---|---|
| `marine/vessel/odom` | `nav_msgs/Odometry` | ~3–7 Hz (see above) | `frame_id: world` (nominal, not TF-resolvable), `child_frame_id: wamv/wamv/base_link` | Position/orientation: `gz topic` poll. Linear velocity: finite difference. Angular velocity: vessel IMU. |
| `marine/robot/arm_joint_state` | `sensor_msgs/JointState` | node's configured `publish_rate_hz` (default 10 Hz) | n/a (joint values only) | Filtered from `/wamv/joint_states`, reordered to a fixed 7-joint arm-only order (`joint_filter.ARM_JOINT_ORDER`) — excludes the gripper's 2 finger joints and every vessel-internal joint. |
| `marine/robot/ee_pose` | `geometry_msgs/PoseStamped` | node's configured `publish_rate_hz` (default 10 Hz) | `frame_id: wamv/wamv/base_link` | Direct TF2 lookup, `wamv/wamv/base_link → wamv/panda_hand`. This **is** the "relative state" named in the M2 milestone: the end effector's pose relative to the (non-inertial) vessel, matching `rl-formulation.md`'s `ee_pose (7) ... vessel-base frame` observation field exactly. |

No custom message types were introduced (`marine_interfaces`, named in `architecture.md`'s package list, remains correctly unbuilt) — every field needed at this milestone is expressible with standard `nav_msgs`/`sensor_msgs`/`geometry_msgs` types. That package should only be created once a genuine need for a custom composite message appears (e.g. bundling robot+vessel+target state into one RL-observation-shaped message at M4/M5) — not spun up speculatively now.

**Why `arm_joint_state` filters instead of just reusing `/wamv/joint_states` directly:** the raw topic mixes the Panda's 7 arm joints, its 2 finger joints, and several vessel-internal joints (thruster/propeller, sensor-mount) that are not manipulator DOF at all — 9 `wamv/*` joints alongside the 9 `panda_*` joints, confirmed by inspecting a live sample. `rl-formulation.md`'s observation schema wants exactly "joint_state (2·N) for N manipulator joints" (N=7); publishing the raw 18-joint mix would leak irrelevant, unstably-ordered data into whatever consumes this downstream.

## 4. Coordinate and unit conventions

- **Units**: SI throughout — meters, radians, seconds, m/s, rad/s. Matches `architecture.md`'s standing requirement.
- **Orientation**: quaternion `(x, y, z, w)` everywhere a ROS2 message is involved (`geometry_msgs/Quaternion` field order), matching REP-103. No other orientation representation (Euler angles, rotation matrices) is used at a message boundary anywhere in this pipeline.
- **Frames**: REP-105/REP-103 axis conventions (right-handed, Z-up) apply within each individual frame; there is deliberately **no** attempt to force a `map`/`odom`/`base_link` REP-105 frame hierarchy onto this simulation, since (per the finding above) no world-fixed frame actually exists here to hang one off of. `marine/vessel/odom`'s `frame_id: world` is a plain descriptive label for "the frame Gazebo itself uses internally," not a claim that a `world` frame is TF2-navigable.
- **The vessel-base frame is the practically important reference frame in this project**, not any world frame: since the vessel is non-inertial (wave/current-disturbed), quantities expressed relative to it (like `ee_pose`) are what the manipulation and RL layers actually need to reason about, and this is exactly what `marine/robot/ee_pose` provides directly via TF2 rather than requiring every consumer to re-derive it from two world-frame poses.

## 5. Validation performed

Validation was empirical, against the live simulation, rather than a build check alone:

- 11 pure-Python unit tests (`pose_stream`, `kinematics`, `joint_filter`) pass with plain `pytest`, no ROS2 dependency (`/usr/bin/python3 -m pytest ros2_ws/src/marine_state_estimator/test/`).
- `colcon build --packages-select marine_state_estimator` succeeds cleanly (no warnings after fixing `setup.cfg`'s deprecated dash-separated keys).
- All three topics confirmed publishing live, sensible data (not zeros, not garbage): vessel position/orientation matched values independently observed via `tf2_echo` earlier in the session; vessel linear velocity observed oscillating smoothly through positive and negative values over several seconds, consistent with wave-driven bobbing (not constant-zero, not diverging); arm joint positions matched the exact configured `initial_positions.yaml` defaults with near-zero velocity (arm stationary, as expected with no command issued); `ee_pose` matched, to high precision, the same translation independently observed via `tf2_echo` at the very start of this milestone's investigation (`[1.157, 0.350, 1.890]`).
- A real bug was caught and fixed during this validation, not glossed over: the first implementation republished the last-known vessel pose at the full 10 Hz timer rate regardless of whether the background poller had produced fresh data, which (since the poller only refreshes every ~130–350ms) caused most ticks to compute a near-zero finite-difference velocity from a duplicate sample. Fixed by detecting and skipping stale poller samples; confirmed fixed by re-sampling `linear.z` over 8 consecutive ~1.2s intervals and observing genuine variation (range roughly −0.003 to +0.014 m/s) rather than mostly-exact-zero.
