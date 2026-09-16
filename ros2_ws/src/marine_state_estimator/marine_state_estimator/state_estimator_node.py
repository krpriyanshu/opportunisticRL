"""M2 state estimator: publishes vessel odometry, filtered arm joint state,
and the vessel-relative end-effector pose. See docs/ros2.md for the full
topic/frame/units contract this node implements.

Architecture note: fetching the vessel's world pose (see pose_stream.py's
docstring for why this needs a `gz topic` subprocess rather than TF2 or a
plain ROS2 topic) runs on a dedicated background thread, not inside the
publish timer callback. A single `gz topic -e -n 1` call's latency is
dominated by Gazebo Transport discovery/process-startup overhead, which is
variable and can exceed the publish period -- running it synchronously in
the timer callback would risk stalling rclpy's executor and silently
degrading every other callback's rate. The background thread continuously
refreshes a lock-protected "latest sample"; the timer callback only ever
does a fast, non-blocking read of that sample.
"""

from __future__ import annotations

import subprocess
import threading
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, JointState

from marine_state_estimator.joint_filter import filter_arm_joints
from marine_state_estimator.kinematics import TimedPosition, finite_difference_velocity
from marine_state_estimator.pose_stream import parse_named_pose


class _VesselPosePoller:
    """Background-thread poller for the vessel's world-frame pose. See
    module docstring for why this isn't done inline in a timer callback."""

    def __init__(self, world_name: str, entity_name: str, timeout_s: float, logger) -> None:
        self._world_name = world_name
        self._entity_name = entity_name
        self._timeout_s = timeout_s
        self._logger = logger
        self._lock = threading.Lock()
        self._latest: tuple[float, tuple, tuple] | None = None  # (t, position, orientation)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def latest(self) -> tuple[float, tuple, tuple] | None:
        with self._lock:
            return self._latest

    def _run(self) -> None:
        topic = f"/world/{self._world_name}/pose/info"
        while not self._stop.is_set():
            try:
                result = subprocess.run(
                    ["gz", "topic", "-e", "-t", topic, "-n", "1"],
                    capture_output=True,
                    text=True,
                    timeout=self._timeout_s,
                )
                pose = parse_named_pose(result.stdout, self._entity_name)
            except subprocess.TimeoutExpired:
                pose = None
            except Exception as exc:  # noqa: BLE001 -- log and keep polling
                self._logger.warning(f"vessel pose poll failed: {exc}")
                pose = None

            if pose is not None:
                with self._lock:
                    self._latest = (time.monotonic(), pose.position, pose.orientation)
            # No fixed sleep: a fresh `gz topic -e -n 1` call is itself the
            # pacing mechanism (it returns once the next message arrives).


class StateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_estimator_node")

        self.declare_parameter("world_name", "sydney_regatta")
        self.declare_parameter("vessel_entity_name", "wamv")
        self.declare_parameter("vessel_base_frame", "wamv/wamv/base_link")
        self.declare_parameter("ee_frame", "wamv/panda_hand")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("gz_topic_timeout_s", 1.0)

        self._vessel_base_frame = self.get_parameter("vessel_base_frame").value
        self._ee_frame = self.get_parameter("ee_frame").value
        rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._odom_pub = self.create_publisher(Odometry, "marine/vessel/odom", 10)
        self._arm_joint_pub = self.create_publisher(JointState, "marine/robot/arm_joint_state", 10)
        self._ee_pose_pub = self.create_publisher(PoseStamped, "marine/robot/ee_pose", 10)

        self._latest_joint_state: JointState | None = None
        self.create_subscription(JointState, "/wamv/joint_states", self._on_joint_state, 10)
        self._latest_imu: Imu | None = None
        self.create_subscription(
            Imu,
            "/wamv/sensors/imu/imu/data",
            self._on_imu,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._prev_position_sample: TimedPosition | None = None

        self._poller = _VesselPosePoller(
            world_name=self.get_parameter("world_name").value,
            entity_name=self.get_parameter("vessel_entity_name").value,
            timeout_s=float(self.get_parameter("gz_topic_timeout_s").value),
            logger=self.get_logger(),
        )
        self._poller.start()

        self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(
            f"state_estimator_node started (rate={rate_hz} Hz, "
            f"vessel_base_frame={self._vessel_base_frame}, ee_frame={self._ee_frame})"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _on_timer(self) -> None:
        self._publish_vessel_odom()
        self._publish_arm_joint_state()
        self._publish_ee_pose()

    def _publish_vessel_odom(self) -> None:
        sample = self._poller.latest()
        if sample is None:
            return
        t, position, orientation = sample

        # The poller's achievable rate (~3-7 Hz measured -- each `gz topic -e
        # -n 1` call costs 130-350ms of process-startup/discovery overhead,
        # not just wait-for-message time) is slower than this node's 10 Hz
        # publish timer. Without this guard, most timer ticks would see the
        # *same* poller sample as the previous tick, compute a near-zero dt,
        # and silently publish a fake near-zero velocity -- worse than not
        # publishing, since it looks like real data. Skip the tick entirely
        # until the poller has produced a genuinely new sample.
        if self._prev_position_sample is not None and t == self._prev_position_sample.t:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = self._vessel_base_frame

        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = orientation

        current = TimedPosition(t=t, position=position)
        if self._prev_position_sample is not None:
            velocity = finite_difference_velocity(self._prev_position_sample, current)
            if velocity is not None:
                (
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ) = velocity
        self._prev_position_sample = current

        if self._latest_imu is not None:
            msg.twist.twist.angular = self._latest_imu.angular_velocity

        self._odom_pub.publish(msg)

    def _publish_arm_joint_state(self) -> None:
        js = self._latest_joint_state
        if js is None:
            return
        try:
            names, positions, velocities = filter_arm_joints(
                list(js.name), list(js.position), list(js.velocity)
            )
        except ValueError as exc:
            self.get_logger().warning(f"arm joint filter failed: {exc}", throttle_duration_sec=5.0)
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = positions
        out.velocity = velocities
        self._arm_joint_pub.publish(out)

    def _publish_ee_pose(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._vessel_base_frame, self._ee_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warning(f"ee_pose TF lookup failed: {exc}", throttle_duration_sec=5.0)
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._vessel_base_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self._ee_pose_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._poller.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
