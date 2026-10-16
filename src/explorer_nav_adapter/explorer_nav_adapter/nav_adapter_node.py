"""ROS 2 Node subscribing to selected targets, delegating to Nav2, and publishing TargetResult."""
from __future__ import annotations

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from explorer_interfaces.msg import ExplorationTarget, TargetResult
from explorer_nav_adapter.nav2_client import Nav2Client, NavState


class NavAdapterNode(Node):
    """Bridges Symbolic Explorer target selection to Nav2 NavigateToPose and reports results."""

    def __init__(self) -> None:
        super().__init__('nav_adapter_node')

        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('min_goal_distance_threshold', 0.5)

        timeout = self.get_parameter('goal_timeout_sec').value
        self.min_goal_dist = self.get_parameter('min_goal_distance_threshold').value

        self.client = Nav2Client(self, goal_timeout_sec=timeout)
        self.client.on_succeeded = self._on_goal_succeeded
        self.client.on_failed = self._on_goal_failed

        self.current_target: ExplorationTarget | None = None
        self.last_goal_pose: PoseStamped | None = None

        # Subscribe to selected targets from mission planner
        self.sub_selected_target = self.create_subscription(
            ExplorationTarget,
            '/mission_node/selected_target',
            self._on_target_received,
            10
        )

        # Publisher for target execution feedback
        self.pub_target_result = self.create_publisher(
            TargetResult,
            '~/target_result',
            10
        )

        # Liveness & timeout check timer (2 Hz)
        self.timer = self.create_timer(0.5, self._on_timer)

        self.get_logger().info('NavAdapterNode initialized with TargetResult feedback.')

    def _on_target_received(self, msg: ExplorationTarget) -> None:
        if not msg.is_safe:
            self.get_logger().warn(f"Ignoring unsafe target {msg.target_id}")
            return

        if not self.client.server_is_ready():
            self.get_logger().info(
                "Nav2 action server 'navigate_to_pose' not yet active (waiting for SLAM localization)...",
                throttle_duration_sec=5.0
            )
            return

        # Check if we are already navigating towards approximately the same location
        if self.client.state == NavState.NAVIGATING and self.last_goal_pose is not None:
            dx = msg.pose.pose.position.x - self.last_goal_pose.pose.position.x
            dy = msg.pose.pose.position.y - self.last_goal_pose.pose.position.y
            dist = math.hypot(dx, dy)
            if dist < self.min_goal_dist:
                # Same target region, don't preempt unnecessarily
                return

        self.get_logger().info(
            f"Dispatching goal target {msg.target_id} at "
            f"({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}) to Nav2."
        )
        self.current_target = msg
        self.last_goal_pose = msg.pose
        self.client.send_goal(msg.pose)

    def _on_goal_succeeded(self) -> None:
        elapsed = time.time() - self.client._goal_start_time if self.client._goal_start_time else 0.0
        res = TargetResult()
        if self.current_target is not None:
            res.target_id = self.current_target.target_id
            res.pose = self.current_target.pose
        elif self.last_goal_pose is not None:
            res.target_id = 0
            res.pose = self.last_goal_pose
        res.success = True
        res.error_msg = ""
        res.execution_time_seconds = float(elapsed)

        self.pub_target_result.publish(res)
        self.get_logger().info(
            f"Published TargetResult: SUCCESS for target {res.target_id} in {elapsed:.2f}s"
        )
        self.current_target = None
        self.last_goal_pose = None

    def _on_goal_failed(self, reason: str) -> None:
        elapsed = time.time() - self.client._goal_start_time if self.client._goal_start_time else 0.0
        res = TargetResult()
        if self.current_target is not None:
            res.target_id = self.current_target.target_id
            res.pose = self.current_target.pose
        elif self.last_goal_pose is not None:
            res.target_id = 0
            res.pose = self.last_goal_pose
        res.success = False
        res.error_msg = reason
        res.execution_time_seconds = float(elapsed)

        self.pub_target_result.publish(res)
        self.get_logger().warn(
            f"Published TargetResult: FAILURE for target {res.target_id}: {reason}"
        )
        self.current_target = None
        self.last_goal_pose = None

    def _on_timer(self) -> None:
        self.client.check_timeout()


def main(args=None):
    rclpy.init(args=args)
    node = NavAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
