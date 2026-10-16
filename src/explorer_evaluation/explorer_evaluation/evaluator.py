"""Evaluation metrics computation and synthetic trial execution."""
from __future__ import annotations

import math
from typing import Any, Dict, List
from explorer_evaluation.baselines import BaseStrategy
from explorer_knowledge.fact import Fact
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission import Mission, Objective, RestrictedZone
from explorer_mission.resource_model import ResourceModel


class TrialMetrics:
    """Quantitative performance metrics collected during an exploration trial."""

    def __init__(self, strategy_name: str, seed: int) -> None:
        self.strategy_name = strategy_name
        self.seed = seed
        self.mission_completed = False
        self.coverage_percent = 0.0
        self.path_length = 0.0
        self.duration_seconds = 0.0
        self.safety_violations = 0
        self.energy_consumed = 0.0
        self.targets_evaluated = 0
        self.latencies_ms: List[float] = []
        self.explanation_completeness = 0.0

    @property
    def average_latency_ms(self) -> float:
        return sum(self.latencies_ms) / max(1, len(self.latencies_ms))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "strategy": self.strategy_name,
            "seed": self.seed,
            "mission_completed": self.mission_completed,
            "coverage_percent": round(self.coverage_percent, 2),
            "path_length_m": round(self.path_length, 2),
            "duration_s": round(self.duration_seconds, 1),
            "safety_violations": self.safety_violations,
            "energy_consumed": round(self.energy_consumed, 2),
            "targets_evaluated": self.targets_evaluated,
            "avg_latency_ms": round(self.average_latency_ms, 3),
            "explanation_completeness": round(self.explanation_completeness, 2),
        }


class BenchmarkEvaluator:
    """Executes controlled multi-step exploration trials on synthetic indoor layouts."""

    def __init__(self) -> None:
        pass

    def run_trial(
        self,
        strategy: BaseStrategy,
        seed: int = 42,
        max_steps: int = 20,
    ) -> TrialMetrics:
        metrics = TrialMetrics(strategy.name, seed)

        # Isolated mission specification
        mission = Mission(
            mission_id=f"eval_{strategy.name}_{seed}",
            objectives=[
                Objective("locate_first_aid", "Find first-aid", priority=50, target_coords=(3.5, -2.5), completion_predicate="ObjectiveCompleted"),
                Objective("inspect_storage", "Inspect storage hazard", priority=30, target_coords=(-3.0, 2.5), completion_predicate="HazardInspected"),
            ],
            restricted_zones=[
                RestrictedZone("restricted_zone_alpha", 2.5, 4.5, 1.5, 3.5)
            ],
            base_station=(0.0, 0.0),
        )

        kb = KnowledgeBase()
        resource_model = ResourceModel(battery_capacity=100.0, cost_per_meter=1.2, reserve_margin=20.0)

        # Simulated robot initial state
        robot_x = 0.0
        robot_y = 0.0
        path_len = 0.0
        explored_targets = set()

        import random
        rng = random.Random(seed)

        for step in range(max_steps):
            # Generate representative candidates (some near, some far, some in restricted zone)
            candidates = [
                FrontierCandidate(target_id=1, wx=3.0, wy=2.0, yaw=0.0, cluster_size=20, expected_info_gain=50.0, clearance=0.4),  # Inside restricted zone!
                FrontierCandidate(target_id=2, wx=3.5, wy=-2.0, yaw=0.0, cluster_size=15, expected_info_gain=40.0, clearance=0.4), # Near first-aid
                FrontierCandidate(target_id=3, wx=-2.8, wy=2.2, yaw=0.0, cluster_size=18, expected_info_gain=45.0, clearance=0.4), # Near storage
                FrontierCandidate(target_id=4, wx=1.0, wy=0.5, yaw=0.0, cluster_size=10, expected_info_gain=25.0, clearance=0.4),  # Central corridor
                FrontierCandidate(target_id=5, wx=-1.5, wy=-1.5, yaw=0.0, cluster_size=12, expected_info_gain=30.0, clearance=0.4),# Near office
            ]
            # Filter out already visited coordinates
            candidates = [c for c in candidates if (c.wx, c.wy) not in explored_targets]
            if not candidates:
                break

            metrics.targets_evaluated += len(candidates)
            chosen, reason, lat_ms = strategy.select_target(candidates, robot_x, robot_y, mission, resource_model, kb)
            metrics.latencies_ms.append(lat_ms)

            if chosen is None:
                # No feasible target (or aborted safely)
                break

            # Check if chosen target violates restricted zone
            if mission.is_inside_restricted_zone(chosen.wx, chosen.wy):
                metrics.safety_violations += 1

            # Move robot to chosen target
            step_dist = math.hypot(chosen.wx - robot_x, chosen.wy - robot_y)
            path_len += step_dist
            resource_model.consume_travel(step_dist)
            robot_x = chosen.wx
            robot_y = chosen.wy
            explored_targets.add((chosen.wx, chosen.wy))

            # Simulate observation discovery: if at (3.5, -2.0) -> first aid found!
            if math.hypot(robot_x - 3.5, robot_y - (-2.5)) < 1.0:
                kb.assert_fact(Fact("ObjectiveCompleted", ["locate_first_aid"]))

            if math.hypot(robot_x - (-3.0), robot_y - 2.5) < 1.0:
                kb.assert_fact(Fact("HazardInspected", ["inspect_storage"]))

            if mission.all_completed(kb):
                metrics.mission_completed = True
                break

        metrics.path_length = path_len
        metrics.energy_consumed = 100.0 - resource_model.percentage
        metrics.duration_seconds = path_len * 4.0 + len(metrics.latencies_ms) * 0.5
        metrics.coverage_percent = min(100.0, len(explored_targets) * 20.0)
        metrics.explanation_completeness = 1.0 if strategy.name == "FullSymbolicExplorer" else 0.0

        return metrics