"""Target evaluator combining hard safety constraints, symbolic reasoning, and utility ranking."""
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple
from explorer_knowledge.fact import Fact
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission import Mission, Objective, RestrictedZone
from explorer_mission.resource_model import ResourceModel


class CandidateScore:
    def __init__(
        self,
        candidate: FrontierCandidate,
        is_safe: bool,
        rejection_reason: str = "",
        m_score: float = 0.0,
        i_score: float = 0.0,
        d_score: float = 0.0,
        r_score: float = 0.0,
        utility: float = -100.0,
    ) -> None:
        self.candidate = candidate
        self.is_safe = is_safe
        self.rejection_reason = rejection_reason
        self.m_score = m_score
        self.i_score = i_score
        self.d_score = d_score
        self.r_score = r_score
        self.utility = utility


class EvaluationRecord:
    """Complete traceable explanation record for target selection."""

    def __init__(
        self,
        selected_candidate: Optional[FrontierCandidate],
        active_objective: Optional[Objective],
        selected_score: Optional[CandidateScore],
        hard_constraint_results: Sequence[str],
        resource_feasible: bool,
        remaining_battery_percent: float,
        supporting_facts: Sequence[Fact],
        inference_rules: Sequence[str],
        rejected_candidates: Sequence[CandidateScore],
        selection_reason: str,
    ) -> None:
        self.timestamp = time.time()
        self.selected_candidate = selected_candidate
        self.active_objective = active_objective
        self.selected_score = selected_score
        self.hard_constraint_results = list(hard_constraint_results)
        self.resource_feasible = resource_feasible
        self.remaining_battery_percent = remaining_battery_percent
        self.supporting_facts = list(supporting_facts)
        self.inference_rules = list(inference_rules)
        self.rejected_candidates = list(rejected_candidates)
        self.selection_reason = selection_reason

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "selected_target_id": self.selected_candidate.target_id if self.selected_candidate else None,
            "selected_coords": (
                (self.selected_candidate.wx, self.selected_candidate.wy)
                if self.selected_candidate else None
            ),
            "mission_objective": self.active_objective.name if self.active_objective else "None",
            "scores": {
                "mission_relevance": self.selected_score.m_score if self.selected_score else 0.0,
                "information_gain": self.selected_score.i_score if self.selected_score else 0.0,
                "travel_cost": self.selected_score.d_score if self.selected_score else 0.0,
                "risk_estimate": self.selected_score.r_score if self.selected_score else 0.0,
                "total_utility": self.selected_score.utility if self.selected_score else -100.0,
            },
            "hard_constraint_results": self.hard_constraint_results,
            "resource_feasibility": {
                "is_feasible": self.resource_feasible,
                "battery_percent": self.remaining_battery_percent,
            },
            "supporting_facts": [repr(f) for f in self.supporting_facts],
            "inference_rules": self.inference_rules,
            "rejected_alternatives": [
                {
                    "target_id": s.candidate.target_id,
                    "coords": (s.candidate.wx, s.candidate.wy),
                    "reason": s.rejection_reason,
                    "utility": s.utility,
                }
                for s in self.rejected_candidates
            ],
            "selection_reason": self.selection_reason,
        }


class TargetEvaluator:
    """Evaluates candidates against hard constraints and multi-criteria utility."""

    def __init__(
        self,
        w_m: float = 0.35,
        w_i: float = 0.30,
        w_d: float = 0.20,
        w_r: float = 0.15,
        min_clearance: float = 0.28,
    ) -> None:
        self.w_m = w_m
        self.w_i = w_i
        self.w_d = w_d
        self.w_r = w_r
        self.min_clearance = min_clearance

    def evaluate(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_x: float,
        robot_y: float,
        mission: Mission,
        resource_model: ResourceModel,
        kb: KnowledgeBase,
    ) -> EvaluationRecord:
        active_obj = mission.get_active_objective(kb)
        if not candidates:
            return EvaluationRecord(
                selected_candidate=None,
                active_objective=active_obj,
                selected_score=None,
                hard_constraint_results=["NoCandidateFrontiersAvailable"],
                resource_feasible=True,
                remaining_battery_percent=resource_model.percentage,
                supporting_facts=[],
                inference_rules=[],
                rejected_candidates=[],
                selection_reason="No frontier candidates detected by mapping.",
            )

        # Retrieve risk and safety facts from KB
        hazard_facts = kb.query("HighRiskArea")
        forbidden_facts = kb.query("ForbidNavigationIn")
        supporting_facts: List[Fact] = list(hazard_facts) + list(forbidden_facts)

        scored_candidates: List[CandidateScore] = []
        max_dist = max(math.hypot(c.wx - robot_x, c.wy - robot_y) for c in candidates)
        max_dist = max(1.0, max_dist)
        max_gain = max(c.expected_info_gain for c in candidates)
        max_gain = max(1.0, max_gain)

        for cand in candidates:
            # 1. HARD CONSTRAINT: Restricted Zone
            violated_zone = mission.is_inside_restricted_zone(cand.wx, cand.wy)
            if violated_zone is not None:
                scored_candidates.append(
                    CandidateScore(
                        candidate=cand,
                        is_safe=False,
                        rejection_reason=f"HardConstraintViolation: inside restricted zone {violated_zone.zone_id}",
                    )
                )
                continue

            # 2. HARD CONSTRAINT: Obstacle Clearance
            if cand.clearance < self.min_clearance:
                scored_candidates.append(
                    CandidateScore(
                        candidate=cand,
                        is_safe=False,
                        rejection_reason=(
                            f"HardConstraintViolation: clearance {cand.clearance:.2f}m < {self.min_clearance:.2f}m"
                        ),
                    )
                )
                continue

            # 3. HARD CONSTRAINT: Resource Feasibility (Reach target + return to base)
            feasible, req_energy, rem_after = resource_model.check_return_feasibility(
                robot_x=robot_x,
                robot_y=robot_y,
                target_x=cand.wx,
                target_y=cand.wy,
                base_x=mission.base_station[0],
                base_y=mission.base_station[1],
            )
            if not feasible:
                scored_candidates.append(
                    CandidateScore(
                        candidate=cand,
                        is_safe=False,
                        rejection_reason=(
                            f"ResourceInfeasible: required {req_energy:.1f}% leaves {rem_after:.1f}% < reserve"
                        ),
                    )
                )
                continue

            # All hard constraints passed! Compute normalized features in [0, 1]
            dist = math.hypot(cand.wx - robot_x, cand.wy - robot_y)
            d_norm = min(1.0, dist / max_dist)
            i_norm = min(1.0, cand.expected_info_gain / max_gain)

            # Mission relevance M(q)
            if active_obj and active_obj.target_coords:
                obj_dist = math.hypot(cand.wx - active_obj.target_coords[0], cand.wy - active_obj.target_coords[1])
                m_norm = 1.0 / (1.0 + obj_dist)
            else:
                m_norm = 0.5  # general unguided exploration

            # Risk estimate R(q)
            # Proximity to any known hazard area in KB
            r_score = 0.0
            for hf in hazard_facts:
                if len(hf.arguments) >= 1:
                    # If hazard location matches zone
                    r_score = max(r_score, 0.8)
            r_norm = min(1.0, r_score)

            # Multi-criteria utility: U = w_m*M + w_i*I - w_d*D - w_r*R
            utility = (
                self.w_m * m_norm +
                self.w_i * i_norm -
                self.w_d * d_norm -
                self.w_r * r_norm
            )

            scored_candidates.append(
                CandidateScore(
                    candidate=cand,
                    is_safe=True,
                    rejection_reason="",
                    m_score=m_norm,
                    i_score=i_norm,
                    d_score=d_norm,
                    r_score=r_norm,
                    utility=utility,
                )
            )

        # Filter feasible candidates
        feasible_candidates = [s for s in scored_candidates if s.is_safe]

        if not feasible_candidates:
            # All candidates rejected by hard constraints
            rejected = scored_candidates
            return EvaluationRecord(
                selected_candidate=None,
                active_objective=active_obj,
                selected_score=None,
                hard_constraint_results=["AllCandidatesViolatedHardConstraints"],
                resource_feasible=resource_model.percentage >= resource_model.reserve_margin,
                remaining_battery_percent=resource_model.percentage,
                supporting_facts=supporting_facts,
                inference_rules=["R_RESTRICTED_FORBIDDEN", "R_HAZARD_RISK"],
                rejected_candidates=rejected,
                selection_reason="All frontier candidates violated safety constraints or resource budget.",
            )

        # Select highest utility candidate
        feasible_candidates.sort(key=lambda s: s.utility, reverse=True)
        best = feasible_candidates[0]

        # Remaining feasible candidates are rejected as suboptimal utility
        rejected = [s for s in scored_candidates if s.candidate.target_id != best.candidate.target_id]
        for s in rejected:
            if s.is_safe and not s.rejection_reason:
                s.rejection_reason = f"SuboptimalUtility: score {s.utility:.2f} < {best.utility:.2f}"

        rules_fired = ["R_MULTI_CRITERIA_UTILITY"]
        if best.m_score > 0.6:
            rules_fired.append("R_MISSION_PROXIMITY_BOOST")

        reason = (
            f"Selected Target {best.candidate.target_id} at ({best.candidate.wx:.2f}, {best.candidate.wy:.2f}) "
            f"with utility={best.utility:.3f} (M={best.m_score:.2f}, I={best.i_score:.2f}, "
            f"D={best.d_score:.2f}, R={best.r_score:.2f}). Satisfies all hard safety & resource constraints."
        )

        return EvaluationRecord(
            selected_candidate=best.candidate,
            active_objective=active_obj,
            selected_score=best,
            hard_constraint_results=["PassedRestrictedZone", "PassedClearance", "PassedResourceFeasibility"],
            resource_feasible=True,
            remaining_battery_percent=resource_model.percentage,
            supporting_facts=supporting_facts,
            inference_rules=rules_fired,
            rejected_candidates=rejected,
            selection_reason=reason,
        )