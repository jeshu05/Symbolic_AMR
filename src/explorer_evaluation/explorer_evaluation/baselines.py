"""Baseline exploration strategies for comparative evaluation."""
from __future__ import annotations

import math
import random
import time
from typing import Optional, Sequence, Tuple
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission import Mission
from explorer_mission.resource_model import ResourceModel
from explorer_mission.target_evaluator import TargetEvaluator


class BaseStrategy:
    """Abstract interface for exploration target selection strategies."""

    def __init__(self, name: str) -> None:
        self.name = name

    def select_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> Tuple[Optional[FrontierCandidate], str, float]:
        raise NotImplementedError


class RandomFrontierStrategy(BaseStrategy):
    """Selects a uniformly random candidate from detected frontiers."""

    def __init__(self, seed: Optional[int] = 42) -> None:
        super().__init__("RandomFrontier")
        self.rng = random.Random(seed)

    def select_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> Tuple[Optional[FrontierCandidate], str, float]:
        t0 = time.perf_counter()
        if not candidates:
            return None, "No candidates", (time.perf_counter() - t0) * 1000.0

        chosen = self.rng.choice(list(candidates))
        latency = (time.perf_counter() - t0) * 1000.0
        return chosen, f"Randomly selected target {chosen.target_id}", latency


class NearestFrontierStrategy(BaseStrategy):
    """Greedily selects the closest frontier by Euclidean distance."""

    def __init__(self) -> None:
        super().__init__("NearestFrontier")

    def select_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> Tuple[Optional[FrontierCandidate], str, float]:
        t0 = time.perf_counter()
        if not candidates:
            return None, "No candidates", (time.perf_counter() - t0) * 1000.0

        best = min(candidates, key=lambda c: math.hypot(c.wx - robot_x, c.wy - robot_y))
        dist = math.hypot(best.wx - robot_x, best.wy - robot_y)
        latency = (time.perf_counter() - t0) * 1000.0
        return best, f"Nearest target {best.target_id} at dist={dist:.2f}m", latency


class MaxInfoGainStrategy(BaseStrategy):
    """Greedily selects the frontier with maximum expected information gain."""

    def __init__(self) -> None:
        super().__init__("MaxInfoGain")

    def select_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> Tuple[Optional[FrontierCandidate], str, float]:
        t0 = time.perf_counter()
        if not candidates:
            return None, "No candidates", (time.perf_counter() - t0) * 1000.0

        best = max(candidates, key=lambda c: c.expected_info_gain)
        latency = (time.perf_counter() - t0) * 1000.0
        return best, f"Max info gain target {best.target_id} with gain={best.expected_info_gain:.1f}", latency


class HeuristicBaselineStrategy(BaseStrategy):
    """Heuristic linear utility combining distance and gain without symbolic reasoning or safety."""

    def __init__(self, w_i: float = 0.6, w_d: float = 0.4) -> None:
        super().__init__("HeuristicBaseline")
        self.w_i = w_i
        self.w_d = w_d

    def select_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> Tuple[Optional[FrontierCandidate], str, float]:
        t0 = time.perf_counter()
        if not candidates:
            return None, "No candidates", (time.perf_counter() - t0) * 1000.0

        max_dist = max(max(math.hypot(c.wx - robot_x, c.wy - robot_y) for c in candidates), 1.0)
        max_gain = max(max(c.expected_info_gain for c in candidates), 1.0)

        def score(c: FrontierCandidate) -> float:
            d = math.hypot(c.wx - robot_x, c.wy - robot_y) / max_dist
            g = c.expected_info_gain / max_gain
            return self.w_i * g - self.w_d * d

        best = max(candidates, key=score)
        latency = (time.perf_counter() - t0) * 1000.0
        return best, f"Heuristic score={score(best):.2f}", latency


class FullSymbolicStrategy(BaseStrategy):
    """Full proposed system with hard safety constraints, symbolic reasoning, and explainability."""

    def __init__(self) -> None:
        super().__init__("FullSymbolicExplorer")
        self.evaluator = TargetEvaluator()

    def select_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> Tuple[Optional[FrontierCandidate], str, float]:
        t0 = time.perf_counter()
        record = self.evaluator.evaluate(
            candidates=candidates,
            robot_x=robot_x,
            robot_y=robot_y,
            mission=mission,
            resource_model=resource_model,
            kb=kb,
        )
        latency = (time.perf_counter() - t0) * 1000.0
        return record.selected_candidate, record.selection_reason, latency