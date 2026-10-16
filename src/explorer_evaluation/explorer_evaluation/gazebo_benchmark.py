"""Real Gazebo Simulation Benchmark Evaluator.
Measures physical odometry path length, live SLAM coverage area, Nav2 goal outcomes,
safety barrier adherence, decision latencies, and simulation RTF from live ROS topics.
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Dict, List, Optional
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from explorer_interfaces.msg import DecisionExplanation, MissionStatus, TargetResult


class GazeboBenchmarkNode(Node):
    """Real-time simulation evaluator collecting empirical robotics metrics from live topics."""

    def __init__(self) -> None:
        super().__init__('gazebo_benchmark_node')

        self.declare_parameter('log_interval_sec', 5.0)
        self.declare_parameter('restricted_x_min', 7.0)
        self.declare_parameter('restricted_x_max', 15.0)
        self.declare_parameter('restricted_y_min', -3.5)
        self.declare_parameter('restricted_y_max', 3.5)

        self.rz_xmin = self.get_parameter('restricted_x_min').value
        self.rz_xmax = self.get_parameter('restricted_x_max').value
        self.rz_ymin = self.get_parameter('restricted_y_min').value
        self.rz_ymax = self.get_parameter('restricted_y_max').value

        # Metrics state
        self.start_wall_time = time.time()
        self.start_sim_time: Optional[float] = None
        self.current_sim_time: Optional[float] = None

        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.total_path_length_m = 0.0

        self.mapped_area_sq_m = 0.0
        self.free_cells_count = 0
        self.known_cells_count = 0

        self.goals_succeeded = 0
        self.goals_failed = 0
        self.goal_durations: List[float] = []

        self.safety_incursions = 0
        self.in_restricted_zone = False

        self.explanations_count = 0
        self.valid_explanations_count = 0

        self.completed_objectives: List[str] = []
        self.mission_state = "UNKNOWN"

        # Subscriptions
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10
        )
        self.sub_target_result = self.create_subscription(
            TargetResult, '/nav_adapter_node/target_result', self._on_target_result, 10
        )
        self.sub_explanation = self.create_subscription(
            DecisionExplanation, '/mission_node/decision_explanation', self._on_explanation, 10
        )
        self.sub_status = self.create_subscription(
            MissionStatus, '/mission_node/mission_status', self._on_status, 10
        )
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self._on_clock, 10
        )

        # Logging setup
        self.log_dir = os.path.expanduser('~/.ros/symbolic_explorer_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, 'gazebo_run_metrics.jsonl')

        interval = self.get_parameter('log_interval_sec').value
        self.timer = self.create_timer(interval, self._log_periodic_metrics)

        self.get_logger().info(f"GazeboBenchmarkNode active. Logging empirical metrics to {self.log_path}")

    def _on_clock(self, msg: Clock) -> None:
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = t
        self.current_sim_time = t

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_x is not None and self.prev_y is not None:
            step = math.hypot(x - self.prev_x, y - self.prev_y)
            if step > 0.001:  # filter noise
                self.total_path_length_m += step
        self.prev_x = x
        self.prev_y = y

        # Audit safety zone breach
        inside = (self.rz_xmin <= x <= self.rz_xmax) and (self.rz_ymin <= y <= self.rz_ymax)
        if inside and not self.in_restricted_zone:
            self.safety_incursions += 1
            self.get_logger().error(f"SAFETY VIOLATION: Robot entered restricted zone at ({x:.2f}, {y:.2f})!")
        self.in_restricted_zone = inside

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.array(msg.data, dtype=np.int8)
        free = int(np.sum(data == 0))
        occupied = int(np.sum(data > 0))
        known = free + occupied
        res = msg.info.resolution

        self.free_cells_count = free
        self.known_cells_count = known
        self.mapped_area_sq_m = float(known * res * res)

    def _on_target_result(self, msg: TargetResult) -> None:
        if msg.success:
            self.goals_succeeded += 1
        else:
            self.goals_failed += 1
        self.goal_durations.append(msg.execution_time_seconds)

    def _on_explanation(self, msg: DecisionExplanation) -> None:
        self.explanations_count += 1
        # Validate explanation completeness: non-empty objective, selection reason, and rules
        if msg.mission_objective and msg.selection_reason and msg.inference_rules:
            self.valid_explanations_count += 1

    def _on_status(self, msg: MissionStatus) -> None:
        self.mission_state = msg.state
        self.completed_objectives = list(msg.completed_objectives)

    def get_summary(self) -> Dict[str, Any]:
        wall_elapsed = time.time() - self.start_wall_time
        sim_elapsed = (
            (self.current_sim_time - self.start_sim_time)
            if (self.start_sim_time is not None and self.current_sim_time is not None)
            else wall_elapsed
        )
        rtf = sim_elapsed / max(0.001, wall_elapsed)

        completeness = (
            self.valid_explanations_count / max(1, self.explanations_count)
        )

        return {
            "timestamp": time.time(),
            "wall_elapsed_s": round(wall_elapsed, 1),
            "sim_elapsed_s": round(sim_elapsed, 1),
            "real_time_factor": round(rtf, 2),
            "physical_path_length_m": round(self.total_path_length_m, 2),
            "slam_mapped_area_sq_m": round(self.mapped_area_sq_m, 2),
            "slam_known_cells": self.known_cells_count,
            "goals_succeeded": self.goals_succeeded,
            "goals_failed": self.goals_failed,
            "safety_incursions": self.safety_incursions,
            "explanations_published": self.explanations_count,
            "explanation_completeness": round(completeness, 2),
            "mission_state": self.mission_state,
            "completed_objectives": self.completed_objectives,
        }

    def _log_periodic_metrics(self) -> None:
        summary = self.get_summary()
        try:
            with open(self.log_path, 'a') as f:
                f.write(json.dumps(summary) + '\n')
        except Exception as e:
            self.get_logger().warn(f"Failed to write metrics: {e}")

        self.get_logger().info(
            f"[Benchmark Metric] Path: {summary['physical_path_length_m']}m | "
            f"SLAM Area: {summary['slam_mapped_area_sq_m']}m² | "
            f"Goals (S/F): {summary['goals_succeeded']}/{summary['goals_failed']} | "
            f"Violations: {summary['safety_incursions']} | RTF: {summary['real_time_factor']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        summary = node.get_summary()
        print("\n================ FINAL GAZEBO BENCHMARK RUN SUMMARY ================")
        print(json.dumps(summary, indent=2))
        print("====================================================================\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
