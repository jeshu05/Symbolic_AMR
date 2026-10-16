"""Unit tests for baseline exploration strategies and evaluation metrics."""
import pytest
from explorer_evaluation.baselines import (
    RandomFrontierStrategy,
    NearestFrontierStrategy,
    MaxInfoGainStrategy,
    HeuristicBaselineStrategy,
    FullSymbolicStrategy,
)
from explorer_evaluation.evaluator import BenchmarkEvaluator
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_mapping.frontier_detector import FrontierCandidate
from explorer_mission.mission import Mission, RestrictedZone
from explorer_mission.resource_model import ResourceModel


@pytest.fixture
def test_setup():
    mission = Mission(
        mission_id="test_mission",
        objectives=[],
        restricted_zones=[RestrictedZone("alpha", 2.0, 4.0, 2.0, 4.0)],
        base_station=(0.0, 0.0),
    )
    res_model = ResourceModel()
    kb = KnowledgeBase()

    # Candidate 1 is inside restricted zone, high info gain
    c1 = FrontierCandidate(1, wx=3.0, wy=3.0, yaw=0.0, cluster_size=30, expected_info_gain=100.0, clearance=0.5)
    # Candidate 2 is safe, low distance
    c2 = FrontierCandidate(2, wx=1.0, wy=0.0, yaw=0.0, cluster_size=5, expected_info_gain=10.0, clearance=0.5)
    # Candidate 3 is safe, far, medium gain
    c3 = FrontierCandidate(3, wx=5.0, wy=0.0, yaw=0.0, cluster_size=15, expected_info_gain=50.0, clearance=0.5)

    return [c1, c2, c3], mission, res_model, kb


def test_nearest_frontier_selection(test_setup):
    cands, mission, res_model, kb = test_setup
    strategy = NearestFrontierStrategy()
    chosen, _, _ = strategy.select_target(cands, robot_x=0.0, robot_y=0.0, mission=mission, resource_model=res_model, kb=kb)
    # Closest is candidate 2 (distance 1.0)
    assert chosen is not None
    assert chosen.target_id == 2


def test_max_info_gain_selection(test_setup):
    cands, mission, res_model, kb = test_setup
    strategy = MaxInfoGainStrategy()
    chosen, _, _ = strategy.select_target(cands, robot_x=0.0, robot_y=0.0, mission=mission, resource_model=res_model, kb=kb)
    # Max gain is candidate 1 (gain 100.0)
    assert chosen is not None
    assert chosen.target_id == 1


def test_full_symbolic_strictly_rejects_restricted_zone(test_setup):
    cands, mission, res_model, kb = test_setup
    strategy = FullSymbolicStrategy()
    chosen, reason, _ = strategy.select_target(cands, robot_x=0.0, robot_y=0.0, mission=mission, resource_model=res_model, kb=kb)
    # Must NOT select candidate 1 despite its highest gain!
    assert chosen is not None
    assert chosen.target_id != 1
    assert "HardConstraintViolation" in reason or "Selected Target" in reason


def test_benchmark_evaluator_run():
    evaluator = BenchmarkEvaluator()
    strat = FullSymbolicStrategy()
    metrics = evaluator.run_trial(strat, seed=1, max_steps=5)

    assert metrics.strategy_name == "FullSymbolicExplorer"
    assert metrics.safety_violations == 0  # Full symbolic must have zero restricted zone violations!
    assert metrics.coverage_percent > 0.0
    assert metrics.explanation_completeness == 1.0