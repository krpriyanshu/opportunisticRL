"""Launch the M2 state estimator node. Assumes the VRX simulation
(scripts/launch_vrx_sim.sh) is already running -- this node only
subscribes/publishes ROS2 topics and polls Gazebo Transport, it does not
launch or depend on any particular world beyond the `world_name` param
matching whatever world is actually running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    world_name_arg = DeclareLaunchArgument("world_name", default_value="sydney_regatta")

    node = Node(
        package="marine_state_estimator",
        executable="state_estimator_node",
        name="state_estimator_node",
        output="screen",
        parameters=[{"world_name": LaunchConfiguration("world_name")}],
    )

    return LaunchDescription([world_name_arg, node])
