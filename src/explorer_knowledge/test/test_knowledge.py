"""Unit tests for symbolic knowledge representation, truth maintenance, and inference."""
import pytest
from explorer_knowledge.fact import EpistemicStatus, Fact
from explorer_knowledge.rule import Pattern, Rule
from explorer_knowledge.knowledge_base import KnowledgeBase
from explorer_knowledge.inference_engine import InferenceEngine, create_default_robotics_rules


def test_fact_creation_and_matching():
    f = Fact("RoomExplored", ["room_1"], confidence=0.9, epistemic_status=EpistemicStatus.OBSERVED)
    assert f.predicate == "RoomExplored"
    assert f.arguments == ("room_1",)
    assert f.matches_pattern("RoomExplored", ["room_1"])
    assert f.matches_pattern("RoomExplored", [None])
    assert not f.matches_pattern("RoomExplored", ["room_2"])
    assert not f.matches_pattern("OtherPred", ["room_1"])


def test_unknown_is_not_false():
    """Verify core epistemic requirement: absence of evidence is not evidence of absence."""
    kb = KnowledgeBase()
    # No facts in KB
    assert kb.is_unknown("HazardObserved", ["room_1", "spill"])
    assert not kb.is_known_true("HazardObserved", ["room_1", "spill"])
    assert not kb.is_known_false("HazardObserved", ["room_1", "spill"])

    # Asserting safe does not happen automatically
    assert kb.is_unknown("AreaSafe", ["room_1"])


def test_assertion_and_retraction_cascade():
    kb = KnowledgeBase()
    engine = InferenceEngine(kb)
    for r in create_default_robotics_rules():
        engine.register_rule(r)

    # Robot enters room_A
    robot_in = Fact("RobotInRoom", ["room_A"])
    kb.assert_fact(robot_in)
    engine.run_to_fixpoint()

    # Should have inferred RoomExplored(room_A)
    assert kb.is_known_true("RoomExplored", ["room_A"])
    inferred_fact = kb.query("RoomExplored", ["room_A"])[0]
    assert inferred_fact.epistemic_status == EpistemicStatus.INFERRED
    assert inferred_fact.generating_rule == "R_ROOM_EXPLORED"
    assert len(inferred_fact.supporting_facts) == 1
    assert inferred_fact.supporting_facts[0] == robot_in

    # When premise is retracted, derived fact must cascade retract
    kb.retract_fact(robot_in)
    assert not kb.is_known_true("RobotInRoom", ["room_A"])
    assert not kb.is_known_true("RoomExplored", ["room_A"])
    assert kb.is_unknown("RoomExplored", ["room_A"])


def test_contradiction_handling():
    kb = KnowledgeBase()
    f1 = Fact("PassageClear", ["door_1"], confidence=0.6, is_negated=False)
    kb.assert_fact(f1)
    assert kb.is_known_true("PassageClear", ["door_1"])
    assert not kb.is_known_false("PassageClear", ["door_1"])

    # A newer observation sees that door_1 is blocked
    f2 = Fact("PassageClear", ["door_1"], confidence=0.9, is_negated=True)
    kb.assert_fact(f2)

    # Previous assertion was contradicted and replaced
    assert not kb.is_known_true("PassageClear", ["door_1"])
    assert kb.is_known_false("PassageClear", ["door_1"])


def test_multi_step_inference():
    kb = KnowledgeBase()
    engine = InferenceEngine(kb)
    for r in create_default_robotics_rules():
        engine.register_rule(r)

    # Assert observation of first aid station in room_2
    kb.assert_fact(Fact("FirstAidStationObserved", ["room_2"]))
    inferred_count = engine.run_to_fixpoint()
    assert inferred_count >= 1

    assert kb.is_known_true("ObjectiveCompleted", ["locate_first_aid"])
    fact = kb.query("ObjectiveCompleted", ["locate_first_aid"])[0]
    prov = kb.get_provenance(fact)
    assert prov["generating_rule"] == "R_FIRST_AID_FOUND"