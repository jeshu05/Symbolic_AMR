"""Unit tests for mission planning, hard safety constraints, and explainable target selection."""
import pytest
from explorer_knowledge.fact import Fact
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission import Mission, Objective, RestrictedZone
from explorer_mission.resource_model import ResourceModel
from explorer_mission.target_evaluator import TargetEvaluator
from explorer_mission.mission_planner import MissionPlanner, create_default_exploration_mission


def test_restricted_zone_hard_constraint():
    """Verify that a candidate inside a restricted zone is strictly rejected, regardless of gain."""
    planner = MissionPlanner()
    # Zone alpha is at x in [7.0, 15.0], y in [-3.5, 3.5]
    inside_candidate = FrontierCandidate(
        target_id=1, wx=10.0, wy=0.0, yaw=0.0, cluster_size=50, expected_info_gain=1000.0, clearance=0.5
    )
    outside_candidate = FrontierCandidate(
        target_id=2, wx=2.0, wy=2.0, yaw=0.0, cluster_size=10, expected_info_gain=10.0, clearance=0.5
    )

    record = planner.plan_next_target([inside_candidate, outside_candidate], robot_x=0.0, robot_y=0.0)

    # Best selected must NOT be target 1
    assert record.selected_candidate is not None
    assert record.selected_candidate.target_id == 2

    # Target 1 must be in rejected candidates with HardConstraintViolation
    rejected_ids = [r.candidate.target_id for r in record.rejected_candidates]
    assert 1 in rejected_ids
    t1_rejected = next(r for r in record.rejected_candidates if r.candidate.target_id == 1)
    assert not t1_rejected.is_safe
    assert "HardConstraintViolation" in t1_rejected.rejection_reason


def test_resource_feasibility_constraint():
    """Verify candidate is rejected if energy is insufficient to return to base station."""
    mission = create_default_exploration_mission()
    res_model = ResourceModel(battery_capacity=100.0, cost_per_meter=2.0, reserve_margin=20.0)
    res_model.current_battery = 25.0

    evaluator = TargetEvaluator()
    far_candidate = FrontierCandidate(
        target_id=1, wx=5.0, wy=0.0, yaw=0.0, cluster_size=20, expected_info_gain=50.0, clearance=0.5
    )

    record = evaluator.evaluate([far_candidate], robot_x=0.0, robot_y=0.0, mission=mission, resource_model=res_model, kb=MissionPlanner().kb)

    assert record.selected_candidate is None
    assert len(record.rejected_candidates) == 1
    assert "ResourceInfeasible" in record.rejected_candidates[0].rejection_reason


def test_mission_relevance_utility_weighting():
    """Verify candidate close to active objective gets higher utility."""
    planner = MissionPlanner()
    # First-aid station objective is at (11.0, 11.0)
    cand_near_obj = FrontierCandidate(
        target_id=1, wx=10.0, wy=10.0, yaw=0.0, cluster_size=10, expected_info_gain=20.0, clearance=0.5
    )
    cand_far = FrontierCandidate(
        target_id=2, wx=-10.0, wy=-10.0, yaw=0.0, cluster_size=10, expected_info_gain=20.0, clearance=0.5
    )

    record = planner.plan_next_target([cand_near_obj, cand_far], robot_x=0.0, robot_y=0.0)
    assert record.selected_candidate is not None
    assert record.selected_candidate.target_id == 1
    assert record.selected_score.m_score > 0.0


def test_structured_explanation_record():
    """Verify explainability record contains required fields and valid schema."""
    planner = MissionPlanner()
    cand = FrontierCandidate(
        target_id=10, wx=1.5, wy=0.5, yaw=0.0, cluster_size=15, expected_info_gain=30.0, clearance=0.4
    )
    record = planner.plan_next_target([cand], robot_x=0.0, robot_y=0.0)

    d = record.to_dict()
    assert "selected_target_id" in d
    assert d["selected_target_id"] == 10
    assert "mission_objective" in d
    assert "scores" in d
    assert "total_utility" in d["scores"]
    assert "hard_constraint_results" in d
    assert "resource_feasibility" in d
    assert "selection_reason" in d
