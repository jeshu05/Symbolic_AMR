"""High-level mission planner coordinating reasoning, target evaluation, adaptation, and safety."""
from __future__ import annotations

import logging
import math
from typing import List, Optional, Sequence, Set, Tuple
from explorer_knowledge.fact import EpistemicStatus, Fact
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_knowledge.inference_engine import InferenceEngine, create_default_robotics_rules
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission import Mission, Objective, RestrictedZone
from explorer_mission.resource_model import ResourceModel
from explorer_mission.target_evaluator import EvaluationRecord, TargetEvaluator

logger = logging.getLogger(__name__)


def create_default_exploration_mission() -> Mission:
    """Create representative demonstration mission for 36m x 36m Research Facility."""
    objectives = [
        Objective(
            name="explore_facility",
            description="Explore initially unknown rooms and corridors",
            priority=10,
        ),
        Objective(
            name="locate_first_aid",
            description="Find and verify the first-aid station in East Wing",
            priority=60,
            target_coords=(11.0, 11.0),
            completion_predicate="ObjectiveCompleted",
        ),
        Objective(
            name="inspect_hazard",
            description="Inspect designated storage room for chemical/spill hazards",
            priority=45,
            target_coords=(11.0, -11.0),
            completion_predicate="HazardInspected",
        ),
        Objective(
            name="inspect_server_room",
            description="Inspect IT server infrastructure room in North-West wing",
            priority=40,
            target_coords=(-11.0, 11.0),
            completion_predicate="ServerRoomInspected",
        ),
    ]

    # Restricted keep-out zone: Quarantine Bay Alpha at x in [7.0, 15.0], y in [-3.5, 3.5]
    restricted_zones = [
        RestrictedZone(
            zone_id="restricted_zone_alpha",
            x_min=7.0,
            x_max=15.0,
            y_min=-3.5,
            y_max=3.5,
        )
    ]

    return Mission(
        mission_id="research_facility_mission",
        objectives=objectives,
        restricted_zones=restricted_zones,
        base_station=(0.0, 0.0),
    )


class MissionPlanner:
    """Coordinates symbolic reasoning, candidate evaluation, and adaptive replanning."""

    def __init__(
        self,
        mission: Optional[Mission] = None,
        resource_model: Optional[ResourceModel] = None,
        evaluator: Optional[TargetEvaluator] = None,
        kb: Optional[KnowledgeBase] = None,
    ) -> None:
        self.mission = mission or create_default_exploration_mission()
        self.resource_model = resource_model or ResourceModel(battery_capacity=100.0, cost_per_meter=0.8, reserve_margin=20.0)
        self.evaluator = evaluator or TargetEvaluator()
        self.kb = kb or KnowledgeBase()
        self.engine = InferenceEngine(self.kb)

        self.visited_targets: Set[Tuple[int, int]] = set()  # quantized coords (int(wx), int(wy))
        self.failed_targets: Set[Tuple[int, int]] = set()

        for r in create_default_robotics_rules():
            self.engine.register_rule(r)

        # Assert prior symbolic knowledge: restricted zone
        for rz in self.mission.restricted_zones:
            self.kb.assert_fact(
                Fact(
                    predicate="RestrictedZone",
                    arguments=[rz.zone_id],
                    confidence=1.0,
                    evidence_source="mission_prior",
                    epistemic_status=EpistemicStatus.OBSERVED,
                )
            )

        self.engine.run_to_fixpoint()

    def record_target_success(self, wx: float, wy: float) -> None:
        """Called when robot physically reaches a target."""
        q = (int(round(wx)), int(round(wy)))
        self.visited_targets.add(q)

        # Check proximity to mission goals
        if math.hypot(wx - 11.0, wy - 11.0) < 2.0:
            logger.info("First Aid Station verified at (11, 11)!")
            self.kb.assert_fact(Fact("FirstAidStationObserved", ["medical_room_east"]))
            self.engine.run_to_fixpoint()

        if math.hypot(wx - 11.0, wy - (-11.0)) < 2.0:
            logger.info("Chemical Hazard inspected at (11, -11)!")
            self.kb.assert_fact(Fact("HazardInspected", ["inspect_hazard"]))
            self.engine.run_to_fixpoint()

        if math.hypot(wx - (-11.0), wy - 11.0) < 2.0:
            logger.info("Server Room inspected at (-11, 11)!")
            self.kb.assert_fact(Fact("ServerRoomInspected", ["inspect_server_room"]))
            self.engine.run_to_fixpoint()

    def record_route_failure(self, wx: float, wy: float, reason: str = "") -> None:
        """Called when Nav2 reports navigation abort / path blockage."""
        q = (int(round(wx)), int(round(wy)))
        self.failed_targets.add(q)
        logger.warning(f"Nav2 aborted target at ({wx:.2f}, {wy:.2f}). Reason: {reason}")
        self.kb.assert_fact(
            Fact(
                predicate="RouteBlocked",
                arguments=[f"pos_{q[0]}_{q[1]}", reason],
                confidence=1.0,
                evidence_source="nav2_abort",
                epistemic_status=EpistemicStatus.OBSERVED,
            )
        )
        self.engine.run_to_fixpoint()

    def plan_next_target(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
    ) -> EvaluationRecord:
        # Step 1: Run inference fixpoint
        self.engine.run_to_fixpoint()

        # Step 2: Filter out previously visited or failed candidate coordinates
        valid_candidates = []
        for c in candidates:
            q = (int(round(c.wx)), int(round(c.wy)))
            if q in self.visited_targets:
                continue
            if q in self.failed_targets:
                continue
            valid_candidates.append(c)

        # Step 3: Evaluate candidates with hard safety constraints & utility
        record = self.evaluator.evaluate(
            candidates=valid_candidates,
            robot_x=robot_x,
            robot_y=robot_y,
            mission=self.mission,
            resource_model=self.resource_model,
            kb=self.kb,
        )

        return record