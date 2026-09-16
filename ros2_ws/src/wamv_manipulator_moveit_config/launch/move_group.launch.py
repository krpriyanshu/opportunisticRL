"""Launch MoveIt2's move_group for the vessel-mounted Panda arm (M3).

Assumes the VRX simulation (scripts/launch_vrx_sim.sh, with the
wamv_with_arm.urdf.xacro override) and the ros2_control controllers
(joint_state_broadcaster, panda_arm_controller) are already running --
this only adds the planning layer on top, it does not spawn anything or
start any controller itself.

All config file paths are resolved as absolute paths up front (via
ament_index) rather than relative to this package's own share directory
-- most of them live in moveit_resources_panda_moveit_config or
wamv_manipulator_description, not here, and MoveItConfigsBuilder's
relative-path resolution is w.r.t. *this* package only.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_ARM_DESC = Path(get_package_share_directory("wamv_manipulator_description"))
_PANDA_MOVEIT = Path(get_package_share_directory("moveit_resources_panda_moveit_config"))
_THIS_PKG = Path(get_package_share_directory("wamv_manipulator_moveit_config"))


def generate_launch_description() -> LaunchDescription:
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="wamv_manipulator_moveit_config")
        .robot_description(
            file_path=str(_ARM_DESC / "urdf" / "wamv_with_arm.urdf.xacro"),
            mappings={
                "namespace": "wamv",
                "locked": "true",
                "vrx_sensors_enabled": "true",
                "thruster_config": "H",
            },
        )
        .robot_description_semantic(file_path=str(_THIS_PKG / "config" / "wamv_panda.srdf"))
        .robot_description_kinematics(file_path=str(_PANDA_MOVEIT / "config" / "kinematics.yaml"))
        .joint_limits(file_path=str(_PANDA_MOVEIT / "config" / "joint_limits.yaml"))
        .trajectory_execution(file_path=str(_PANDA_MOVEIT / "config" / "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # True, not False: joint_state_broadcaster/robot_state_publisher
        # stamp messages using sim time (the bridged /clock topic, per
        # gz_ros2_control) -- with use_sim_time False here, move_group's
        # own clock runs on wall time while the joint states it watches
        # are stamped in sim time (seconds since the simulation started,
        # e.g. "33.4"), so its staleness check compares a wall-clock
        # "now" against a sim-time "latest sample" and always sees the
        # latter as impossibly old. Verified empirically: this was the
        # actual cause of a CONTROL_FAILED ("didn't receive robot state
        # with recent timestamp") on every trajectory execution attempt.
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription([move_group_node])
