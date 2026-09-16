"""M3: conventional grasp pipeline against a static target.

One-shot script (not a long-running node): plan a motion to the target's
*current* position (read fresh, not the position computed at spawn time,
since the vessel may have drifted since -- see model.sdf's own docstring
about the target being world-static), execute it via MoveIt2's move_group
action, then command the scripted "magnetic latch" attach (the
DetachableJoint plugin wired into wamv_with_arm.urdf.xacro), matching the
project directive's Section 7: grasp success is a deterministic pose
check plus a scripted attach, not physical finger-contact dynamics.

Assumes already running: the simulation with grasp_target spawned,
ros2_control (joint_state_broadcaster + panda_arm_controller), and
move_group (wamv_manipulator_moveit_config). This script only orchestrates
existing services/actions, it does not launch anything itself.
"""

from __future__ import annotations

import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from marine_state_estimator.pose_stream import parse_named_pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from marine_manipulation.quaternion_math import quat_conjugate, quat_rotate

GROUP_NAME = "panda_arm"
ARM_JOINTS = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]


def fetch_world_pose(world: str, entity_name: str, timeout_s: float = 2.0):
    result = subprocess.run(
        ["gz", "topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1"],
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    return parse_named_pose(result.stdout, entity_name)


class GraspPipeline(Node):
    def __init__(self) -> None:
        super().__init__("grasp_pipeline")
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._latest_joint_state: JointState | None = None
        self.create_subscription(JointState, "/wamv/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def wait_for_joint_state(self, timeout_s: float = 5.0) -> JointState:
        deadline = time.monotonic() + timeout_s
        while self._latest_joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._latest_joint_state is None:
            raise RuntimeError("no /wamv/joint_states received")
        return self._latest_joint_state

    def compute_target_vessel_relative_position(
        self, world: str, vessel_entity: str, target_entity: str
    ) -> tuple[float, float, float]:
        vessel = fetch_world_pose(world, vessel_entity)
        target = fetch_world_pose(world, target_entity)
        if vessel is None or target is None:
            raise RuntimeError(f"could not locate '{vessel_entity}' and/or '{target_entity}'")
        # relative = R(vessel)^-1 * (target_world - vessel_world)
        delta = tuple(t - v for t, v in zip(target.position, vessel.position, strict=True))
        inv_q = quat_conjugate(vessel.orientation)
        return quat_rotate(inv_q, delta)  # type: ignore[return-value]

    def compute_ik(
        self, target_position: tuple[float, float, float], seed: JointState
    ) -> JointState | None:
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/compute_ik service not available")

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2
        req.ik_request.robot_state.joint_state = seed

        pose = PoseStamped()
        # Empty frame_id, not VESSEL_BASE_FRAME: verified empirically that
        # naming the frame explicitly makes /compute_ik fail with
        # FRAME_TRANSFORM_FAILURE (error -21) even though it names exactly
        # the SRDF's own planning/model frame -- MoveIt2's IK service
        # apparently still attempts (and fails) a TF2 lookup whenever
        # frame_id is non-empty, regardless of whether it matches the
        # model frame. Empty frame_id is the documented MoveIt2 convention
        # for "already expressed in the planning frame" and skips that
        # lookup entirely, which is exactly correct here since
        # target_position is already computed in the vessel-base frame.
        pose.header.frame_id = ""
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target_position
        # Keep the arm's current resting orientation -- only the position
        # changes. Good enough for an M3 demonstration; approach-axis /
        # grasp-orientation planning is not this milestone's point.
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0
        req.ik_request.pose_stamped = pose

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("compute_ik call did not complete")
        resp = future.result()
        if resp.error_code.val != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            self.get_logger().error(f"IK failed, error code {resp.error_code.val}")
            return None
        return resp.solution.joint_state

    def plan_and_execute_joint_goal(self, joint_state: JointState) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/move_action server not available")

        constraints = Constraints()
        for name, position in zip(joint_state.name, joint_state.position, strict=True):
            if name not in ARM_JOINTS:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.goal_constraints.append(constraints)
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 5

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("MoveGroup goal did not complete in time")
            return False
        error_code = result.result.error_code.val
        if error_code != 1:
            self.get_logger().error(f"MoveGroup execution failed, error code {error_code}")
            return False
        return True

    def attach_target(self, timeout_s: float = 3.0) -> None:
        # /model/wamv/detachable_joint/attach is a *topic* the
        # DetachableJoint system subscribes to (gz.msgs.Empty), not a
        # service -- verified empirically: calling it with `gz service`
        # timed out every time (nothing serves it), while publishing to
        # it with `gz topic -t ... -p ""` succeeds immediately and is
        # exactly what the system's own debug log ("Attach topic is: ...")
        # says it is.
        result = subprocess.run(
            [
                "gz",
                "topic",
                "-t",
                "/model/wamv/detachable_joint/attach",
                "-m",
                "gz.msgs.Empty",
                "-p",
                "",
            ],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        if result.returncode != 0:
            self.get_logger().error(f"attach publish failed: {result.stderr}")


def main() -> None:
    rclpy.init()
    node = GraspPipeline()
    try:
        relative_target = node.compute_target_vessel_relative_position(
            "sydney_regatta", "wamv", "grasp_target"
        )
        node.get_logger().info(f"target position (vessel frame): {relative_target}")

        seed = node.wait_for_joint_state()
        solution = node.compute_ik(relative_target, seed)
        if solution is None:
            node.get_logger().error("IK found no solution for the target pose")
            sys.exit(1)

        node.get_logger().info("IK solved, sending MoveGroup goal...")
        ok = node.plan_and_execute_joint_goal(solution)
        if not ok:
            node.get_logger().error("plan/execute failed")
            sys.exit(1)

        node.get_logger().info("motion complete, attaching target")
        node.attach_target()
        node.get_logger().info("grasp pipeline complete")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
