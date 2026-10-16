"""Nav2 action client managing NavigateToPose execution, recovery, and revalidation."""
from __future__ import annotations

from enum import Enum
import math
import time
from typing import Callable, Optional
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavState(str, Enum):
    IDLE = "IDLE"
    WAITING_SERVER = "WAITING_SERVER"
    GOAL_SENT = "GOAL_SENT"
    NAVIGATING = "NAVIGATING"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"


class Nav2Client:
    """Manages NavigateToPose action lifecycle, cancellation, and timeout detection."""

    def __init__(
        self,
        node: Node,
        action_name: str = 'navigate_to_pose',
        goal_timeout_sec: float = 60.0,
    ) -> None:
        self.node = node
        self.action_name = action_name
        self.goal_timeout_sec = goal_timeout_sec

        self._client = ActionClient(self.node, NavigateToPose, self.action_name)
        self.state = NavState.IDLE
        self._current_goal_handle = None
        self._goal_start_time = 0.0
        self._distance_remaining = 0.0
        self._number_of_recoveries = 0

        self.on_succeeded: Optional[Callable[[], None]] = None
        self.on_failed: Optional[Callable[[str], None]] = None

    def server_is_ready(self) -> bool:
        return self._client.server_is_ready()

    def send_goal(self, target_pose: PoseStamped) -> bool:
        """Send navigation goal to Nav2."""
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn(f"Nav2 action server '{self.action_name}' not available!")
            self.state = NavState.FAILED
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.state = NavState.GOAL_SENT
        self._goal_start_time = time.time()
        self._number_of_recoveries = 0

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def cancel_goal(self) -> None:
        """Cancel current active navigation goal."""
        if self._current_goal_handle is not None and self.state == NavState.NAVIGATING:
            self.node.get_logger().info("Cancelling current Nav2 navigation goal...")
            self._current_goal_handle.cancel_goal_async()
            self.state = NavState.CANCELED

    def check_timeout(self) -> bool:
        """Check if navigation has exceeded maximum allowed duration."""
        if self.state == NavState.NAVIGATING:
            elapsed = time.time() - self._goal_start_time
            if elapsed > self.goal_timeout_sec:
                self.node.get_logger().warn(
                    f"Navigation goal timed out after {elapsed:.1f}s. Cancelling..."
                )
                self.cancel_goal()
                self.state = NavState.ABORTED
                if self.on_failed:
                    self.on_failed(f"Navigation timed out after {elapsed:.1f}s")
                return True
        return False

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("Nav2 rejected exploration goal!")
            self.state = NavState.FAILED
            if self.on_failed:
                self.on_failed("Goal rejected by Nav2")
            return

        self.node.get_logger().info("Nav2 accepted exploration goal.")
        self._current_goal_handle = goal_handle
        self.state = NavState.NAVIGATING

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._distance_remaining = feedback.distance_remaining
        self._number_of_recoveries = feedback.number_of_recoveries

    def _get_result_callback(self, future) -> None:
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Nav2 successfully reached target goal!")
            self.state = NavState.SUCCEEDED
            if self.on_succeeded:
                self.on_succeeded()
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info("Nav2 goal was canceled.")
            self.state = NavState.CANCELED
            if self.on_failed:
                self.on_failed("Goal was canceled")
        else:
            self.node.get_logger().warn(f"Nav2 goal failed with status code {status}.")
            self.state = NavState.ABORTED
            if self.on_failed:
                self.on_failed(f"Nav2 status code {status}")

    @property
    def distance_remaining(self) -> float:
        return self._distance_remaining

    @property
    def number_of_recoveries(self) -> int:
        return self._number_of_recoveries