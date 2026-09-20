# Environment (Milestone M1)

This documents the actual, verified simulator setup in this repository's
reference development environment — what was installed, how, and every
non-obvious gotcha hit along the way. Per `docs/project-plan.md`'s
ground rules, nothing here is asserted until it was actually exercised;
where something is still unverified, it's marked as such.

See `docs/architecture.md`'s
[Simulator selection](architecture.md#simulator-selection-milestone-m1-decision-recorded-now-per-section-517)
section for the decision this environment setup implements.

## What's installed

- **ROS2 Jazzy Jalisco** (`/opt/ros/jazzy`), pre-existing.
- **Gazebo Harmonic** (`gz-sim` 8.11.0), installed via
  `sudo apt-get install ros-jazzy-ros-gz`. This pulled in ROS's *vendor*
  packages (`ros-jazzy-gz-sim-vendor`, `ros-jazzy-gz-*-vendor`, etc.),
  which build Gazebo Harmonic inside the ROS install prefix
  (`/opt/ros/jazzy/opt/gz_*_vendor/`) rather than as standalone system
  packages. The `gz` CLI binary itself lives at
  `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` and is only on `PATH` once
  `/opt/ros/jazzy/setup.bash` is sourced.
- **The OSRF Gazebo apt repo**
  (`packages.osrfoundation.org/gazebo/ubuntu-stable`), added alongside
  the above. Not strictly required for the ROS-vendored Gazebo install,
  but needed for `python3-sdformat14` and `libsdformat14` (see below) —
  the ROS Jazzy `sdformat-vendor` package builds the C++ library only,
  without Python bindings.
- **VRX**, imported via `vcstool` from the upstream `jazzy` branch
  (`ros2_ws/vrx.repos` → `ros2_ws/src/vrx/`, gitignored — see
  `ros2_ws/README.md`). This branch is explicitly the maintainers'
  "recommended configuration for new users" for Gazebo Harmonic + ROS2
  Jazzy (confirmed from the branch's own README, not assumed). Built
  successfully with `colcon build` (5/5 packages: `vrx_gazebo`,
  `wamv_description`, `wamv_gazebo`, `vrx_ros`, `vrx_gz`).

## Install steps that required `sudo` (run by the user, not this agent)

This agent's shell has no interactive TTY, so every `sudo`-requiring step
was handed to the user as an explicit command to run themselves. In
order, what was actually needed:

1. `sudo apt-get install -y ros-jazzy-ros-gz` — Gazebo Harmonic + ROS2
   bridge (also added the OSRF repo + key in the same round, though the
   key install silently failed the first time — see gotcha below).
2. `sudo apt-get install -y ros-jazzy-ament-cmake-pycodestyle` — the one
   dependency `rosdep install --from-paths src --ignore-src -r -y
   --simulate` reported missing for VRX; everything else was already
   satisfied.
3. `sudo apt-get install -y python3-sdformat14` — needed at *launch*
   time, not build time (`vrx_gz`'s launch scripts `import sdformat14`
   directly in Python; this isn't declared in any package.xml `<depend>`,
   so `rosdep` never flagged it — a real gap in VRX's declared
   dependencies, not something rosdep missed by our error).
4. `sudo apt-get install -y libsdformat14` — `python3-sdformat14`'s own
   `.deb` control metadata does not declare `libsdformat14` as a
   dependency, even though its compiled extension module directly needs
   `libsdformat14.so.14` (`ldd` confirms `=> not found` without it). This
   looks like an upstream packaging gap in the OSRF `.deb`, not a
   mistake on our end — recorded here in case it recurs after a package
   update.

## Gotchas hit (each cost real debugging time — recorded so they aren't
re-discovered from scratch)

1. **`sudo` needs a real TTY.** This agent's `Bash` tool has no
   controlling terminal; `sudo <cmd>` fails immediately with `sudo: a
   terminal is required to read the password`, it does not hang waiting
   for one. All privileged steps had to be relayed to the user as exact
   commands.

2. **The OSRF repo's GPG key can fail to install silently.** The first
   `sudo curl https://packages.osrfoundation.org/gazebo.gpg -o
   /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg` never actually
   wrote the file (no error surfaced to the user at the time, but the
   file was simply absent afterward — `apt-get update` then failed with
   `NO_PUBKEY 67170598AF249743` for that repo). A retry over `https://`
   then hit a transient `SSL connection timeout`, even though `apt`
   itself could reach the same host over plain `http://` moments later
   for the repo's `InRelease` file. Fix: since this agent's shell and the
   user's shell are **the same machine**, the agent fetched the key
   itself (`curl` worked fine from here) and had the user `sudo cp` the
   already-downloaded file into place instead of re-fetching over the
   network a third time. Verify a downloaded keyring with
   `gpg --show-keys <file>` and check the fingerprint suffix matches
   apt's reported missing key ID before trusting it.

3. **`CONDA_PREFIX` breaks CMake's Python discovery even after fixing
   `PATH`.** `colcon build` initially invoked
   `~/anaconda3/bin/python3` (no `catkin_pkg` installed
   there) instead of the system Python ROS2 needs
   (`/usr/bin/python3`, which has `catkin_pkg`), causing every
   `ament_cmake_python`-using package to fail with
   `ModuleNotFoundError: No module named 'catkin_pkg'`. Simply removing
   `anaconda3` from `PATH` was **not sufficient** — CMake's `FindPython3`
   module explicitly checks the `CONDA_PREFIX` environment variable as a
   preferred interpreter location (conda-awareness added in CMake 3.24+),
   independent of `PATH` order. Fix: unset `CONDA_PREFIX` (and
   `CONDA_EXE`, `CONDA_PYTHON_EXE`, `CONDA_DEFAULT_ENV`, `CONDA_SHLVL`) in
   addition to cleaning `PATH`, e.g.:
   ```bash
   CLEAN_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd: -)
   env -u CONDA_PREFIX -u CONDA_EXE -u CONDA_PYTHON_EXE -u CONDA_DEFAULT_ENV \
       -u CONDA_SHLVL PATH="$CLEAN_PATH" bash -c \
       'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
   ```
   This is the same underlying conda-vs-ROS2-Python conflict noted in
   `ros2_ws/README.md` at M0, now with the additional `CONDA_PREFIX`
   detail. It affects *building*, not just *running*, ROS2 Python
   packages.

4. **`gz sim`'s stdout is fully buffered when not attached to a TTY.**
   Redirecting `gz sim ... > file.log 2>&1 &` produced almost no output
   even at `-v 4` (max verbosity) — only `[Wrn]`/stderr lines appeared,
   because glibc switches stdout to block-buffering when `isatty()` is
   false, and gz's `[Msg]`/`[Dbg]` logging goes to stdout. This looked
   exactly like a hang. Fix: prefix with `stdbuf -oL -eL` to force line
   buffering when redirecting Gazebo's output to a file for inspection.

5. **`pkill -f` matches its own invoking shell.** `pkill -9 -f "gz sim"`,
   intended to clean up stray simulation processes, instead killed the
   *current* shell process running the cleanup script itself — because
   `-f` matches the full command line of every process, and the
   `bash -c "...pkill -9 -f 'gz sim'..."` process invoking the script has
   that exact substring in its own argv (the whole script text is the
   process's command line). This reproduces with *any* pattern, even
   ones chosen specifically not to match anything real, as long as the
   pattern text is also written inside the same script (confirmed with a
   deliberately nonsensical pattern — `pgrep -f` matched the invoking
   shell every time). Symptom was total silence and a bare "Exit code 1"
   with no other output, which first looked like a resource or discovery
   problem before the actual cause was found. Fix: prefer
   `ps -eo pid,comm | grep -w <exact-binary-name>` (matches the process's
   short name only, not full argv) over `pkill -f`/`pgrep -f` for
   cleanup inside scripts that themselves contain the target pattern as
   literal text.

6. **VRX's `sydney_regatta` world downloads its main model from Fuel
   (Gazebo's model hosting service) on first load** — a `.zip` fetched
   from an S3 presigned URL, not a fast local operation. No progress is
   logged between "Downloading model [...]" and completion, so this also
   looks like a hang if you don't know to expect it. Confirmed working
   (not stuck) by comparing against Gazebo's trivial bundled `shapes.sdf`
   demo world, which loaded and published topics immediately — isolating
   the slowness to the Fuel download specifically, not a broken Gazebo
   Transport/discovery setup (multicast etc. all confirmed working via
   that same `shapes.sdf` test).

7. **`gz_ros2_control`'s `<robot_param>` tag does not do what its name
   suggests, and its own log message actively misleads about it.** The
   plugin's log always prints `Subscribing to '/robot_description' topic
   for robot description`, using that literal hardcoded string,
   regardless of what `<robot_param>` is actually set to — this looks
   like confirmation that it's subscribing to the (wrong, unnamespaced)
   `/robot_description` topic, but it is not actually informative either
   way. What actually determines behavior:
   - Setting `<robot_param>` to a namespaced value
     (`${namespace}/robot_description`) and *not* namespacing the
     `gz_ros2_control::GazeboSimROS2ControlPlugin` node itself: the
     resulting `controller_manager` node genuinely subscribed to the
     unnamespaced `/robot_description` (confirmed via `ros2 topic info
     <topic> --verbose`, checking `Subscription count` on each
     candidate topic directly — not via the log line, which is useless
     for this). Since `robot_state_publisher` only publishes on
     `/wamv/robot_description` in this setup, `controller_manager` waits
     forever (`Waiting for data on 'robot_description' topic to finish
     initialization`, repeating).
   - Adding `<ros><namespace>${namespace}</namespace></ros>` to the
     plugin block does move `controller_manager` into the `/wamv`
     namespace and fixes the subscription — but then breaks something
     else: `moveit_resources_panda_moveit_config`'s own (unmodified,
     reused) `ros2_controllers.yaml` has an unqualified top-level
     `controller_manager:` key, which under standard ROS2
     parameter-YAML-file node-name matching only matches a node
     literally named `/controller_manager` (root namespace). A
     namespaced `/wamv/controller_manager` node does not match that key,
     so none of the file's controller `type` declarations actually load
     (confirmed via `ros2 param list`/`ros2 param get` on the running
     node: `panda_arm_controller.type` was entirely absent, not merely
     unset) — `update_rate` appeared to load correctly and was a red
     herring: that specific value is read directly from the YAML file's
     text by the plugin's own code for its internal loop timing,
     independent of the ROS2 parameter server, so it "worked" for a
     reason unrelated to whether the rest of the file loaded.
   - **Working fix**: keep `controller_manager` in the root namespace
     (no `<ros><namespace>`, so the reused `ros2_controllers.yaml`
     matches as-is) and add
     `<ros><remapping>robot_description:=/${namespace}/robot_description</remapping></ros>`
     instead. This redirects whatever internal (relative, unnamespaced)
     topic name the plugin actually subscribes to, without needing to
     know or trust what `<robot_param>` alone does. Verified end-to-end:
     `controller_manager` receives the robot description immediately,
     `ros2 param get /controller_manager panda_arm_controller.type`
     returns the correct value, and `ros2 run controller_manager
     spawner joint_state_broadcaster -c /controller_manager` /
     `... spawner panda_arm_controller -c /controller_manager` both load
     and activate successfully.
   - A related trap while debugging this: repeated `kill -9` restarts of
     the whole simulation (many of them, across a long session) left
     hundreds of stale `/dev/shm/fastrtps_*` segments and at least once
     coincided with `ros2 service call`/spawner hangs accompanied by an
     unusual `rcl node's context is invalid` error. Restarting the ROS2
     daemon (`ros2 daemon stop && ros2 daemon start`) and clearing stale
     `/dev/shm/fastrtps_*` files did not, on their own, turn out to be
     the actual fix here (the remapping was), but ended up being a
     reasonable first thing to rule out before chasing a deeper bug —
     cheap to try, and it plausibly matters on a machine that's had many
     ungraceful kills across a long session even when it isn't the
     specific cause.

8. **There is no ROS2-native source of the vessel's world-frame pose,
   and VRX's own odometry plugin is wired in but silently does nothing**
   (Milestone M2). Full investigation and the working fix in
   [docs/ros2.md](ros2.md#vessel-world-frame-state-has-no-ros2-native-source) —
   short version: TF2's tree is rooted at `wamv/wamv/base_link` with no
   world-connected parent (verified, not assumed); `/wamv/pose` looks
   promising but only carries sensor-mount statics; the
   `gz::sim::systems::OdometryPublisher` plugin is correctly configured
   in `wamv_gazebo.urdf.xacro` (the `.so` exists, no error is logged) but
   never actually advertises a topic — root cause not found, flagged as
   an open item rather than chased indefinitely; and naively bridging
   `/world/<world>/pose/info` with `ros_gz_bridge` loses every entity's
   `name` field, making the bridged version unusable for picking "wamv"
   out of the crowd. Working fix: poll `gz topic -e -n 1` (the CLI tool,
   which *does* retain names) from a background thread, ~3–7 Hz achievable
   (130–350ms per call, mostly process-startup/discovery overhead) — and
   explicitly detect+skip a stale/duplicate sample rather than
   republishing it at a faster nominal rate, which a first-draft
   implementation didn't do and silently produced fake near-zero
   velocities on most publish ticks as a result (caught by re-sampling
   and noticing suspiciously many exact `0.0` readings, not by code
   review — worth remembering as a general lesson: a background-poller
   value that "doesn't look wrong" at a glance can still be stale).

9. **Milestone M3 (conventional grasping) hit five distinct, real bugs
   between "code compiles" and "the grasp actually works" — recorded in
   full since each cost real debugging time and none were guessable in
   advance:**
   - **A stale CMake cache reintroduces the conda-vs-system-Python
     conflict from gotcha #3, even after fixing `PATH`.** A first build
     attempt of a new `ament_cmake` package (`wamv_manipulator_moveit_config`)
     failed with `ModuleNotFoundError: No module named 'catkin_pkg'`
     because `execute_process` invoked `/home/.../anaconda3/bin/python3`
     — even *after* stripping `anaconda3` from `PATH` the same way
     `scripts/launch_vrx_sim.sh` does, the error persisted, because the
     first (failed) configure had already written the wrong
     `PYTHON_EXECUTABLE` into `build/<pkg>/CMakeCache.txt`, and CMake
     does not re-probe cached values on a subsequent run regardless of
     the environment. Fix: `rm -rf build/<pkg> install/<pkg>` and
     rebuild fresh with `PATH` already clean — cleaning `PATH` *after* a
     tainted cache exists is not enough.
   - **MoveIt2's SRDF `virtual_joint` must describe the URDF's actual
     kinematics, not be copied blindly.** `moveit_resources_panda_moveit_config`'s
     stock SRDF declares `type="floating"` from `panda_link0` to a
     `"world"` frame — correct for the Panda used standalone (unknown
     mounting), wrong here, where `panda_link0` is already rigidly fixed
     to the vessel by a real joint in the URDF. Fix documented in full
     in `config/wamv_panda.srdf`'s own header comment
     (`wamv_manipulator_moveit_config`): `type="fixed"`,
     `parent_frame="wamv/wamv/base_link"`.
   - **A URDF link connected only by fixed joints may not exist as an
     SDF `<link>` at all — it can be silently lumped into its parent.**
     Adding a `DetachableJoint` plugin with `<parent_link>panda_hand</parent_link>`
     failed at Gazebo load time with `Link with name panda_hand not
     found in model wamv`, despite `panda_hand` working fine everywhere
     else (TF, `marine/robot/ee_pose`). Root cause, found by running the
     exact same `xacro | gz sdf -p` conversion VRX's own spawn tooling
     runs and inspecting the output directly rather than guessing:
     `panda_hand` and `panda_link8` are connected to `panda_link7` by
     fixed joints, and SDFormat's URDF-to-SDF conversion merges
     ("lumps") consecutive fixed-jointed links into one rigid body for
     physics purposes — `panda_hand` survives only as a pose `<frame>`
     (which is why TF/pose lookups for it keep working), not as an
     actual dynamics `<link>` a joint plugin can attach to. Fix:
     reference `panda_link7` (the surviving rigid body) instead — no
     physical difference, since `panda_hand` is fixed relative to it.
   - **`move_group` needs `use_sim_time: True` explicitly — it does not
     inherit it from the rest of the ROS2 graph, and the failure mode
     doesn't look like a clock problem.** With `use_sim_time` left at
     its default (`False`), every trajectory execution failed with
     `CONTROL_FAILED`, logged as `Didn't receive robot state (joint
     angles) with recent timestamp within 1.000000 seconds. Requested
     time 1786454936.25..., but latest received state has time
     33.396000.` — i.e. `move_group`'s own clock was running on wall
     time (Unix epoch, ~1.79 billion) while `joint_state_broadcaster`
     stamps messages with sim time (seconds since the simulation
     started, driven by the bridged `/clock` topic), so *every* joint
     state looked impossibly stale by comparison, regardless of how
     current it actually was. Fix: `{"use_sim_time": True}` in the
     `move_group` launch file's parameters.
   - **`compute_ik`'s `pose_stamped.header.frame_id` must be left empty,
     even when you know and name the correct planning frame.** Setting
     `frame_id` explicitly to `wamv/wamv/base_link` (exactly the SRDF's
     own `virtual_joint` parent frame, i.e. not a wrong value) still
     failed every time with `MoveItErrorCodes.FRAME_TRANSFORM_FAILURE`
     (-21). `/compute_ik` apparently attempts a TF2-mediated transform
     whenever `frame_id` is non-empty at all, regardless of whether it
     already matches the model frame — and per gotcha #8, TF2 has no
     resolvable path to `wamv/wamv/base_link` from wherever it's looking.
     Empty string is the documented MoveIt2 convention for "already in
     the planning frame" and skips that lookup entirely. Confirmed via a
     direct `ros2 service call` comparison (`frame_id: ''` succeeded
     immediately with a valid IK solution; the same request with
     `frame_id: 'wamv/wamv/base_link'` failed) before changing the real
     script, not guessed.
   - **`DetachableJoint` cannot attach to a fully `<static>` link, and
     the failure is silent (no error, the child object just never
     moves).** The grasp target was first modeled as `<static>true</static>`
     (simplest way to make it hold still). The attach call ran without
     error, but the target's world pose never changed to track the
     gripper afterward. Root cause (inferred from DART's general
     behavior, not from an explicit error message — a static link has
     no dynamics state for a physics joint to constrain in the first
     place): a joint needs two real dynamic bodies. Fix: make the link a
     normal dynamic body with `<gravity>false</gravity>` instead of
     `<static>true</static>` — holds it motionless (no forces act on it)
     while remaining a real body a joint can attach to. Confirmed fixed
     by commanding a *second*, different arm pose after attaching and
     verifying the target's world position moved by a comparable
     distance in the same direction as the commanded motion, not by
     checking proximity after a single grasp alone (proximity right
     after a grasp is also consistent with "landed near there by
     coincidence" — a second, independent motion is what actually rules
     that out).
   - One more finding worth a line even though it didn't block anything:
     **a model can have more than one `DetachableJoint` system plugin,
     and Gazebo does not appear to disambiguate their attach topics by
     instance** — both VRX's own pre-existing "platform lock" instance
     and this project's new "grasp" instance (both declared on the same
     `wamv` entity) advertise the identical topic name
     `/model/wamv/detachable_joint/attach`. Harmless in this specific
     case only because VRX's `platform` model is never spawned in
     `sydney_regatta`, so its instance has nothing to act on — worth
     revisiting if a future milestone ever needs two independent,
     simultaneously-usable attach points on the same model.

## Status

Gazebo Harmonic install: **verified working** (`gz sim --version` →
`8.11.0`; `shapes.sdf` demo world loads and publishes topics correctly,
including confirming Gazebo Transport discovery/multicast works fine —
ruling that out as a suspect once the `sydney_regatta` stall below was
first observed).

VRX build: **verified working** (5/5 packages build cleanly with
`colcon build`).

VRX `sydney_regatta` world launch: **verified working.** First load
requires downloading the world's Fuel-hosted assets (~159 MiB total
across the main terrain model, 128,980,600 bytes / ~123 MiB, plus several
smaller buoy/marker models) via
`gz-fuel-resources-production.s3.amazonaws.com`. On this network that
download is genuinely slow and bursty rather than broken: two early
attempts were killed after appearing to stall indefinitely at ~9.1 MiB
(~7%) with no progress for 30–60s, which looked like a hard proxy block.
A user-requested longer observation (10 min, polling `rchar` in
`/proc/<pid>/io` every 20s) showed the actual pattern: **flat for ~5.3
minutes, then a fast burst** (~131 MiB in 20s) that carried it past the
original stall point, followed by another flat stretch before the world
finished initializing (`World [sydney_regatta] initialized`, confirmed in
the log) — so this is long, uneven network stalls (consistent with an
institutional proxy relaying slowly/in bursts for a domain outside its
usual traffic, not a hard size cap or policy block as first suspected.
**Once downloaded, Gazebo caches the asset locally**
(`~/.gz/fuel/fuel.gazebosim.org/...`), so subsequent launches do not
re-download it. Confirmed via `gz topic -l` against the running world:
VRX-specific topics are live, including `/vrx/wavefield/parameters` and
`/vrx/debug/wind/{speed,direction}` — the wave/current disturbance model
from Section 14 of the ticket is real infrastructure here, not just
config.

Lesson: don't conclude "hard block" from a single ~30-60s stall window on
a large first-time download over an institutional network; the actual
failure mode here was patience, not a broken setup. If this recurs on a
fresh machine/cache, budget several minutes for the first
`sydney_regatta` (or any other VRX world) launch before assuming
something is wrong.

Full pipeline (world + WAM-V vessel spawn + ROS2↔Gazebo bridge via
`ros2 launch vrx_gz competition.launch.py`) — **verified working.**
`ros2 topic list` against the running system shows a complete state
pipeline: `/tf`, `/tf_static`, `/wamv/pose`, `/wamv/joint_states`,
`/wamv/robot_description`, per-sensor topics (twin front cameras + one
mid camera, GPS, IMU, lidar, acoustic receiver), thruster command/state
topics (`/wamv/thrusters/{left,right}/{pos,thrust}`), and VRX task/
disturbance topics (`/vrx/task/info`, `/vrx/contacts`,
`/vrx/debug/wind/*`). This is most of Milestone M2's "ROS2 state
pipeline" requirement already present for free from VRX/`ros_gz`, not
something this project has to build from scratch.

### Mesh resource path bug (found and fixed)

The WAM-V's own accessory meshes (GPS antenna, engines, propellers)
initially failed to load:
`Unable to find file with URI [model://wamv_gazebo/models/gps/mesh/gps.dae]`
(and similarly for `wamv_description/models/{engine,propeller}`). The
files exist correctly in the colcon install tree
(`install/wamv_gazebo/share/wamv_gazebo/models/gps/mesh/gps.dae`, etc.);
the problem is that VRX's `wamv_description`/`wamv_gazebo` packages
register their own `GZ_SIM_RESOURCE_PATH` ament environment hooks one
directory level too shallow (pointing at `.../share/wamv_description/models`
instead of `.../share/wamv_description`, which is what's needed for a
`model://wamv_description/...` URI to resolve), and `wamv_gazebo` doesn't
register a resource-path hook at all. This looks like an upstream
packaging gap in VRX itself, not a mistake on our end. Confirmed by
inspecting the actual `GZ_SIM_RESOURCE_PATH` value after sourcing
`install/setup.bash`.

**Fix:** append the correct paths explicitly —
`GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:<ws>/install/wamv_description/share:<ws>/install/wamv_gazebo/share"`
— before launching. Verified: mesh-loading error count went from several
per launch to zero, and all WAM-V geometry (hull, GPS mast, engines,
propellers) now renders correctly (see visuals below). This fix is baked
into `scripts/launch_vrx_sim.sh` (see next section) so it doesn't need to
be rediscovered or applied by hand.

### `scripts/launch_vrx_sim.sh`

Milestone M1's Definition of Done ("one command launches simulation") is
implemented as `scripts/launch_vrx_sim.sh`, which wraps
`ros2 launch vrx_gz competition.launch.py` with every environment fix
from this document applied automatically (conda/`PATH`/`CONDA_PREFIX`
cleanup, the `GZ_SIM_RESOURCE_PATH` mesh fix). Usage:

```bash
scripts/launch_vrx_sim.sh headless:=True world:=sydney_regatta
```

Verified working end-to-end (world loads, WAM-V spawns with correct
meshes, zero `Unable to find file` errors).

### Visuals (for research write-ups; captured 2026-08-10, regenerated 2026-08-11)

Two images captured from the running simulation, saved to `docs/images/`:

- `docs/images/m1_wamv_sydney_regatta.png` — third-person establishing
  shot of the WAM-V (with the Panda arm now visible on deck, since the
  2026-08-11 regeneration was captured with the manipulator mounted)
  floating in Sydney Regatta harbor, captured from a temporary
  camera-only model spawned into the running world via
  `ros2 run ros_gz_sim create -file '<sdf>' -x -y -z -R -P -Y ...` (not
  part of the permanent simulation setup — a one-off tool for getting an
  external viewpoint, since VRX's own sensors are all vessel-mounted).
  Shows the full vessel geometry (twin pontoons, sensor mast, GPS puck,
  mounted arm), water reflections, marker buoys, and shoreline.
- `docs/images/m1_wamv_onboard_camera.png` — real sensor output from the
  WAM-V's own `front_left_camera_sensor`
  (`/wamv/sensors/cameras/front_left_camera_sensor/image_raw`, 1280×720,
  `rgb8`), captured with a small `rclpy` subscriber
  (`sensor_msgs/Image` → `PIL.Image.frombytes`, deliberately avoiding
  `cv_bridge`/`cv2` — see gotcha below). Shows the vessel's own pontoons
  in frame, open water, a marker buoy, and the harbor treeline.

Both were rendered via Gazebo's off-screen/EGL headless rendering
(`--headless-rendering`, no GUI/X server needed — confirmed working
without any real display attached: `Unable to open display: . Trying to
run in headless mode.` is an expected message, not an error, in this
mode) — so these are cheap to regenerate on any machine with the same
Gazebo Harmonic install, no GPU or virtual framebuffer setup required.

**Gotcha:** the system's `cv_bridge` (`/opt/ros/jazzy/...`) imports `cv2`,
which is compiled against NumPy 1.x; this environment's
`~/.local/lib/python3.12/site-packages/numpy` is 2.4.6, and the ABI
mismatch crashes on import
(`A module that was compiled using NumPy 1.x cannot be run in NumPy
2.4.6`). Avoided entirely by decoding the raw `rgb8` `sensor_msgs/Image`
bytes directly with `PIL.Image.frombytes("RGB", (w, h), data)` — no
`cv_bridge`/`cv2`/`numpy` dependency needed for a single-frame capture.

**Gotcha (found during the 2026-08-11 regeneration):** `python3` on
`PATH` resolves to Anaconda's interpreter (3.13), which does not have
`rclpy`'s compiled C extension (`_rclpy_pybind11`, built for the system's
Python 3.12) — importing `rclpy` fails with `ModuleNotFoundError:
No module named 'rclpy._rclpy_pybind11'` even with ROS2's `setup.bash`
sourced and `PYTHONPATH` intact. This is the same conda-vs-system-Python
conflict as the CMake/`FindPython3` issue earlier in this document, just
hitting a different tool. Fix: invoke capture/helper scripts with
`/usr/bin/python3` explicitly rather than bare `python3` (this is exactly
what `scripts/launch_vrx_sim.sh` already does for the simulator itself
via its `PATH` cleanup — ad hoc `rclpy` scripts need the same treatment,
which the script does not currently do for you).

**Gotcha (found during the 2026-08-11 regeneration):** `ros2 run
ros_gz_sim create -file <sdf> -x -y -z -R -P -Y ...` — the `-x -y -z -R
-P -Y` flags **override** whatever `<pose>` is written inside the SDF
file, entirely silently (no warning, no error). Passing `-x 0 -y 0 -z 0`
"just to be explicit" while relying on an embedded `<pose>` for the real
position spawns the model at the world origin with identity orientation
instead — confirmed by reading the spawned entity's actual pose back
from `/world/<world>/pose/info` (`position {}` with no fields, i.e. all
zero) after two capture attempts produced suspiciously identical images.
Fix: pick one mechanism, not both — either bake the full pose into the
SDF's `<pose>` element and omit the CLI pose flags, or omit `<pose>` from
the SDF and always pass `-x -y -z -R -P -Y` explicitly (used throughout
the regenerated captures). Separately, confirmed empirically: Gazebo's
SDF `pitch` (rotation about local Y in the `<pose>`'s roll-pitch-yaw)
is **positive-down** — a positive pitch value tilts the camera's forward
axis toward the water, not the sky, matching the standard SDF/Gazebo
right-hand-rule convention rather than an aerospace nose-up-positive one.

### Video (`docs/images/m1_wamv_underway.mp4`)

A short clip of the WAM-V actually moving under thrust, for research
write-ups (6.25s in the original 2026-08-10 capture; regenerated
2026-08-11, 7.6s, same method). Method: with the sim running, a second
temporary camera-only model (`video_cam`, same technique as the
establishing-shot image above) was spawned at a fixed vantage point;
`/wamv/thrusters/{left,right}/thrust` were driven continuously at 1500 N
via `ros2 topic pub -r 20`; a small `rclpy` subscriber saved each
incoming frame from the camera topic as a PNG for 60 wall-clock seconds;
the sequence was assembled with `ffmpeg` (`-framerate 8 ... -c:v
libx264`).

Things worth recording about this capture:

- The camera's configured sensor `update_rate` was 10 Hz, but only
  ~60-80 frames arrive over 60 wall-clock seconds (~1-1.3 Hz actual,
  varies run to run with host load) — this environment's real-time
  factor is well below 1 under headless/software rendering with many
  worker threads, not a bug in the capture script. Recording duration
  needs to account for this (a naive "record for N seconds, expect N×rate
  frames" assumption will under-count substantially).
- The WAM-V has real inertia — a first attempt at 16s of capture showed
  almost no visible displacement because the vessel was still
  accelerating from rest. Extending to 60s was what actually produced
  visible motion across the sequence (confirmed by inspecting first/
  middle/last frames directly, not assumed from thrust being "on").
- **1500 N sustained for 60s moves the vessel a lot** (confirmed in the
  2026-08-11 regeneration: roughly 85 m of travel, enough to drive it
  completely out of a ~15 m-standoff camera's frame partway through the
  recording window) — the resulting clip legitimately shows the vessel
  driving out of frame rather than staying centered, which is expected
  given the sustained thrust and not a framing bug. To bring the vessel
  back to a known, centered position afterward (needed before the
  close-up arm-motion capture below, since that shot's camera position
  is computed from the vessel's current pose), a brief reverse-thrust
  pulse (-1200 N on both thrusters for ~8s, then 0) was enough to arrest
  most of the drift — residual settling motion (wave bobbing plus a
  small amount of leftover translation) drops to well under 1 m per few
  seconds afterward, which is precise enough for framing a static camera
  shot.

### Video (`docs/images/m1_panda_commanded_motion.mp4`)

A close-up clip of the mounted Panda arm executing a commanded joint
trajectory (4.6s in the 2026-08-11 capture). Method: with
`joint_state_broadcaster` and `panda_arm_controller` loaded and active
(see the `gz_ros2_control` section above), a third temporary camera-only
model (`arm_cam`) was spawned close to the arm's mount point — positioned
by reading the vessel's current world pose off
`/world/sydney_regatta/pose/info`, transforming the arm mount's known
local offset (`manipulator_mount_joint`'s `<origin xyz="0.85 0.35
1.3">`, from `wamv_with_arm.urdf.xacro`) into world coordinates by the
vessel's current yaw, then placing the camera a few meters back along
the *vessel's own heading axis* (not an arbitrary world-frame offset) so
the framing stays valid even if the vessel drifts slightly further
during the ~5s recording window. A single-point `JointTrajectory`
(target `[1.0, -0.5, 0.3, -2.0, 0.0, 1.8, 0.5]` rad, `time_from_start:
4s`) was published to `/panda_arm_controller/joint_trajectory` while a
small `rclpy` subscriber recorded frames from the camera topic; `ffmpeg`
assembled the sequence the same way as the underway video. Convergence
was verified numerically, not just visually, by reading
`/wamv/joint_states` after the trajectory completed: all 7 joints
matched the commanded target to within ~1e-8 rad.
