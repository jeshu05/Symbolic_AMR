"""Forward-chaining inference engine with provenance tracking."""
from __future__ import annotations

import logging
from typing import List, Optional, Sequence, Tuple
from explorer_knowledge.fact import EpistemicStatus, Fact
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_knowledge.rule import Pattern, Rule

logger = logging.getLogger(__name__)


class InferenceEngine:
    """Runs forward-chaining rules over a KnowledgeBase until fixpoint."""

    def __init__(self, kb: Optional[KnowledgeBase] = None) -> None:
        self.kb = kb if kb is not None else KnowledgeBase()
        self.rules: List[Rule] = []

    def register_rule(self, rule: Rule) -> None:
        self.rules.append(rule)
        # Sort by priority descending
        self.rules.sort(key=lambda r: r.priority, reverse=True)

    def step(self) -> int:
        """Execute one pass over all rules. Returns count of newly inferred facts."""
        new_facts_count = 0
        all_facts = self.kb.get_all_facts()

        for rule in self.rules:
            derivations = rule.evaluate(all_facts)
            for conclusion, antecedents in derivations:
                # If conclusion is novel or updates KB
                if self.kb.assert_fact(conclusion):
                    new_facts_count += 1
                    all_facts = self.kb.get_all_facts()

        return new_facts_count

    def run_to_fixpoint(self, max_iterations: int = 50) -> int:
        """Run until no new facts are inferred or max iterations reached."""
        total_inferred = 0
        iteration = 0
        while iteration < max_iterations:
            added = self.step()
            if added == 0:
                break
            total_inferred += added
            iteration += 1
        return total_inferred


def create_default_robotics_rules() -> List[Rule]:
    """Default rule base for autonomous indoor exploration and safety."""
    rules: List[Rule] = []

    # Rule 1: Robot in room -> Room is explored
    rules.append(Rule(
        rule_id="R_ROOM_EXPLORED",
        name="RoomExploredFromPresence",
        antecedents=[
            Pattern("RobotInRoom", ["?room"]),
        ],
        consequent_generator=lambda b, f: Fact(
            predicate="RoomExplored",
            arguments=[b["?room"]],
            confidence=0.95,
            evidence_source="rule:R_ROOM_EXPLORED",
        ),
        priority=10,
        description="A room is marked explored once the robot enters it",
    ))

    # Rule 2: Hazard observed -> Location is high risk
    rules.append(Rule(
        rule_id="R_HAZARD_RISK",
        name="HighRiskFromHazard",
        antecedents=[
            Pattern("HazardObserved", ["?loc", "?type"]),
        ],
        consequent_generator=lambda b, f: Fact(
            predicate="HighRiskArea",
            arguments=[b["?loc"]],
            confidence=1.0,
            evidence_source="rule:R_HAZARD_RISK",
        ),
        priority=20,
        description="Hazard detection marks the area as high risk",
    ))

    # Rule 3: Restricted zone -> Forbid target selection inside
    rules.append(Rule(
        rule_id="R_RESTRICTED_FORBIDDEN",
        name="ForbidRestrictedZone",
        antecedents=[
            Pattern("RestrictedZone", ["?zone"]),
        ],
        consequent_generator=lambda b, f: Fact(
            predicate="ForbidNavigationIn",
            arguments=[b["?zone"]],
            confidence=1.0,
            evidence_source="rule:R_RESTRICTED_FORBIDDEN",
        ),
        priority=30,
        description="Restricted zones strictly forbid robot entry",
    ))

    # Rule 4: First Aid Station located -> Mission sub-goal satisfied
    rules.append(Rule(
        rule_id="R_FIRST_AID_FOUND",
        name="FirstAidFoundGoalSatisfied",
        antecedents=[
            Pattern("FirstAidStationObserved", ["?room"]),
        ],
        consequent_generator=lambda b, f: Fact(
            predicate="ObjectiveCompleted",
            arguments=["locate_first_aid"],
            confidence=1.0,
            evidence_source="rule:R_FIRST_AID_FOUND",
        ),
        priority=15,
        description="Observing the first-aid station completes the locate_first_aid objective",
    ))

    # Rule 5: Low Battery Reserve -> Return to base required
    rules.append(Rule(
        rule_id="R_LOW_BATTERY_ABORT",
        name="ReturnToBaseOnLowReserve",
        antecedents=[
            Pattern("BatteryLow", []),
        ],
        consequent_generator=lambda b, f: Fact(
            predicate="ReturnToBaseRequired",
            arguments=["base_station"],
            confidence=1.0,
            evidence_source="rule:R_LOW_BATTERY_ABORT",
        ),
        priority=50,
        description="Insufficient battery reserve forces an immediate return-to-base requirement",
    ))

    return rules