"""Mission objectives, safety constraints, and completion tracking for research facility."""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple
from explorer_knowledge.knowledge_base import KnowledgeBase


class RestrictedZone:
    """A rectangular 2D spatial keep-out zone."""

    def __init__(self, zone_id: str, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        self.zone_id = zone_id
        self.x_min = min(x_min, x_max)
        self.x_max = max(x_min, x_max)
        self.y_min = min(y_min, y_max)
        self.y_max = max(y_min, y_max)

    def contains(self, wx: float, wy: float, margin: float = 0.0) -> bool:
        return (
            (self.x_min - margin) <= wx <= (self.x_max + margin) and
            (self.y_min - margin) <= wy <= (self.y_max + margin)
        )

    def __repr__(self) -> str:
        return f"RestrictedZone({self.zone_id}: x=[{self.x_min:.1f}, {self.x_max:.1f}], y=[{self.y_min:.1f}, {self.y_max:.1f}])"


class Objective:
    """A high-level mission objective with symbolic completion condition."""

    def __init__(
        self,
        name: str,
        description: str,
        priority: int,
        target_coords: Optional[Tuple[float, float]] = None,
        completion_predicate: Optional[str] = None,
    ) -> None:
        self.name = name
        self.description = description
        self.priority = priority
        self.target_coords = target_coords
        self.completion_predicate = completion_predicate
        self.is_completed = False

    def check_completion(self, kb: KnowledgeBase) -> bool:
        if self.is_completed:
            return True
        if self.completion_predicate and kb.is_known_true(self.completion_predicate, [self.name]):
            self.is_completed = True
        return self.is_completed

    def __repr__(self) -> str:
        status = "COMPLETED" if self.is_completed else "PENDING"
        return f"Objective({self.name}, prio={self.priority}, status={status})"


class Mission:
    """Manages structured research facility mission specification and active objectives."""

    def __init__(
        self,
        mission_id: str,
        objectives: Sequence[Objective],
        restricted_zones: Sequence[RestrictedZone] = (),
        base_station: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.mission_id = mission_id
        self.objectives = list(objectives)
        self.restricted_zones = list(restricted_zones)
        self.base_station = base_station

    def get_active_objective(self, kb: KnowledgeBase) -> Optional[Objective]:
        """Returns the highest priority uncompleted objective."""
        for obj in self.objectives:
            obj.check_completion(kb)

        pending = [obj for obj in self.objectives if not obj.is_completed]
        if not pending:
            return None
        pending.sort(key=lambda o: o.priority, reverse=True)
        return pending[0]

    def is_inside_restricted_zone(self, wx: float, wy: float, margin: float = 0.2) -> Optional[RestrictedZone]:
        for zone in self.restricted_zones:
            if zone.contains(wx, wy, margin=margin):
                return zone
        return None

    def all_completed(self, kb: KnowledgeBase) -> bool:
        return self.get_active_objective(kb) is None