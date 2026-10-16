"""ROS 2 Node managing mission objectives, frontier evaluation, explanations, and closed-loop results."""
from __future__ import annotations

import json
import os
from typing import List, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from explorer_interfaces.msg import (
    DecisionExplanation,
    ExplorationTarget,
    ExplorationTargetArray,
    MissionStatus,
    SymbolicFact,
    TargetResult,
)
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission_planner import MissionPlanner


class MissionNode(Node):
    """Subscribes to candidates, evaluates with mission constraints, dispatches goals, and tracks execution."""

    def __init__(self) -> None:
        super().__init__('mission_node')

        self.planner = MissionPlanner()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goals_reached = 0
        self.goals_failed = 0
        self.frontiers_evaluated_count = 0

        self.is_navigating = False
        self.active_target_id: Optional[int] = None
        self.latest_candidates: List[FrontierCandidate] = []

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom,
            10
        )

        self.sub_candidates = self.create_subscription(
            ExplorationTargetArray,
            '/frontier_node/candidate_targets',
            self._on_candidates,
            10
        )

        self.sub_result = self.create_subscription(
            TargetResult,
            '/nav_adapter_node/target_result',
            self._on_target_result,
            10
        )

        # Publishers
        self.pub_selected_target = self.create_publisher(
            ExplorationTarget,
            '~/selected_target',
            10
        )

        self.pub_explanation = self.create_publisher(
            DecisionExplanation,
            '~/decision_explanation',
            10
        )

        self.pub_status = self.create_publisher(
            MissionStatus,
            '~/mission_status',
            10
        )

        # Periodic status publisher (1 Hz)
        self.timer_status = self.create_timer(1.0, self._publish_status)

        # Log directory for JSONL decision tracking
        self.log_dir = os.path.expanduser('~/.ros/symbolic_explorer_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.jsonl_path = os.path.join(self.log_dir, 'decisions.jsonl')

        self.get_logger().info(f'MissionNode initialized. Logging decisions to {self.jsonl_path}')

    def _on_odom(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _on_target_result(self, msg: TargetResult) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if msg.success:
            self.goals_reached += 1
            self.get_logger().info(
                f"Target {msg.target_id} at ({x:.2f}, {y:.2f}) SUCCEEDED in {msg.execution_time_seconds:.1f}s!"
            )
            self.planner.record_target_success(x, y)
        else:
            self.goals_failed += 1
            self.get_logger().warn(
                f"Target {msg.target_id} at ({x:.2f}, {y:.2f}) FAILED: {msg.error_msg}"
            )
            self.planner.record_route_failure(x, y, msg.error_msg)

        self.is_navigating = False
        self.active_target_id = None

        # Immediately trigger next target evaluation
        if self.latest_candidates:
            self._evaluate_and_dispatch()

    def _on_candidates(self, msg: ExplorationTargetArray) -> None:
        candidates = []
        for t in msg.targets:
            cand = FrontierCandidate(
                target_id=t.target_id,
                wx=t.pose.pose.position.x,
                wy=t.pose.pose.position.y,
                yaw=0.0,
                cluster_size=10,
                expected_info_gain=t.expected_info_gain or 10.0,
                clearance=0.4,
            )
            candidates.append(cand)

        self.latest_candidates = candidates
        self.frontiers_evaluated_count += len(candidates)

        # Goal stability hysteresis: do not replan if already actively navigating to a safe target
        if not self.is_navigating:
            self._evaluate_and_dispatch()

    def _evaluate_and_dispatch(self) -> None:
        if not self.latest_candidates:
            return

        record = self.planner.plan_next_target(self.latest_candidates, self.robot_x, self.robot_y)

        # Log decision to JSONL
        try:
            with open(self.jsonl_path, 'a') as f:
                f.write(json.dumps(record.to_dict()) + '\n')
        except Exception as e:
            self.get_logger().warn(f'Failed to log decision to JSONL: {e}')

        # Publish explanation msg
        exp_msg = DecisionExplanation()
        exp_msg.header.stamp = self.get_clock().now().to_msg()
        exp_msg.header.frame_id = 'map'
        exp_msg.mission_objective = record.active_objective.name if record.active_objective else 'None'
        exp_msg.hard_constraint_results = record.hard_constraint_results
        exp_msg.resource_feasible = record.resource_feasible
        exp_msg.inference_rules = record.inference_rules
        exp_msg.selection_reason = record.selection_reason

        if record.selected_score and record.selected_candidate:
            exp_msg.mission_relevance_score = record.selected_score.m_score
            exp_msg.information_gain_score = record.selected_score.i_score
            exp_msg.travel_cost = record.selected_score.d_score
            exp_msg.risk_estimate = record.selected_score.r_score

            t_msg = ExplorationTarget()
            t_msg.target_id = record.selected_candidate.target_id
            t_msg.pose.header = exp_msg.header
            t_msg.pose.pose.position.x = record.selected_candidate.wx
            t_msg.pose.pose.position.y = record.selected_candidate.wy
            t_msg.expected_info_gain = record.selected_candidate.expected_info_gain
            t_msg.mission_relevance = record.selected_score.m_score
            t_msg.total_utility = record.selected_score.utility
            t_msg.is_safe = True
            exp_msg.selected_target = t_msg

            self.is_navigating = True
            self.active_target_id = record.selected_candidate.target_id
            self.pub_selected_target.publish(t_msg)

        self.pub_explanation.publish(exp_msg)
        self.get_logger().info(f'Target Decision: {record.selection_reason}')

    def _publish_status(self) -> None:
        active_obj = self.planner.mission.get_active_objective(self.planner.kb)
        status_msg = MissionStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.mission_id = self.planner.mission.mission_id
        status_msg.state = "ACTIVE" if active_obj else "COMPLETED"
        status_msg.completed_objectives = [
            o.name for o in self.planner.mission.objectives if o.is_completed
        ]
        status_msg.pending_objectives = [
            o.name for o in self.planner.mission.objectives if not o.is_completed
        ]
        status_msg.remaining_energy_percent = self.planner.resource_model.percentage
        status_msg.return_route_feasible = (
            self.planner.resource_model.percentage >= self.planner.resource_model.reserve_margin
        )
        status_msg.frontiers_evaluated = self.frontiers_evaluated_count
        status_msg.goals_reached = self.goals_reached
        status_msg.goals_failed = self.goals_failed

        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
