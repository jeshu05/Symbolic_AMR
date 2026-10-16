"""Knowledge base maintaining facts, epistemic states, and truth maintenance."""
from __future__ import annotations

import logging
from typing import Dict, List, Optional, Sequence, Set, Tuple
from explorer_knowledge.fact import EpistemicStatus, Fact

logger = logging.getLogger(__name__)


class KnowledgeBase:
    """Stores symbolic facts with open-world semantics, provenance, and truth maintenance."""

    def __init__(self) -> None:
        # Key: (predicate, arguments, is_negated) -> Fact
        self._facts: Dict[Tuple[str, Tuple[str, ...], bool], Fact] = {}
        # Track dependency graph for truth maintenance: premise_key -> set of derived_keys
        self._derived_by: Dict[Tuple[str, Tuple[str, ...], bool], Set[Tuple[str, Tuple[str, ...], bool]]] = {}
        # Audit log of transactions
        self._history: List[Dict[str, str]] = []

    def assert_fact(self, fact: Fact) -> bool:
        """Assert a fact into the knowledge base.
        
        Returns True if a new fact was added or an existing fact was updated.
        Handles contradiction resolution (retracts opposite polarity if new fact is higher confidence or fresher).
        """
        key = fact.key()
        opposite_key = (fact.predicate, fact.arguments, not fact.is_negated)

        # Check for direct logical contradiction
        if opposite_key in self._facts:
            opp_fact = self._facts[opposite_key]
            logger.warning(
                "Contradiction detected: asserting %s against existing %s",
                fact, opp_fact
            )
            # Retract conflicting fact and its dependents
            self.retract_fact(opp_fact)

        if key in self._facts:
            existing = self._facts[key]
            # Update only if higher confidence or newer observation
            if fact.confidence > existing.confidence or fact.timestamp > existing.timestamp:
                self._facts[key] = fact
                self._record_history("UPDATE", fact)
                return True
            return False

        self._facts[key] = fact
        self._record_history("ASSERT", fact)

        # Record dependency tracking for inferred facts
        if fact.supporting_facts:
            for parent in fact.supporting_facts:
                p_key = parent.key()
                if p_key not in self._derived_by:
                    self._derived_by[p_key] = set()
                self._derived_by[p_key].add(key)

        return True

    def retract_fact(self, fact: Fact) -> bool:
        """Retract a fact and recursively retract all facts that depend on it."""
        key = fact.key()
        if key not in self._facts:
            return False

        # Cascade retraction to all dependent inferred facts
        dependents = self._derived_by.pop(key, set())
        for dep_key in list(dependents):
            if dep_key in self._facts:
                dep_fact = self._facts[dep_key]
                logger.info(
                    "Cascading retraction to dependent fact %s derived from %s",
                    dep_fact, fact
                )
                self.retract_fact(dep_fact)

        del self._facts[key]
        self._record_history("RETRACT", fact)
        return True

    def query(
        self,
        predicate: str,
        arguments: Optional[Sequence[Optional[str]]] = None,
        is_negated: Optional[bool] = None,
    ) -> List[Fact]:
        """Query all facts matching the given predicate, argument pattern, and polarity."""
        results: List[Fact] = []
        for fact in self._facts.values():
            if fact.matches_pattern(predicate, arguments, is_negated):
                results.append(fact)
        return results

    def is_known_true(self, predicate: str, arguments: Sequence[str] = ()) -> bool:
        """Check if predicate(arguments) is explicitly asserted as true."""
        key = (predicate, tuple(arguments), False)
        return key in self._facts

    def is_known_false(self, predicate: str, arguments: Sequence[str] = ()) -> bool:
        """Check if predicate(arguments) is explicitly asserted as false (negated)."""
        key = (predicate, tuple(arguments), True)
        return key in self._facts

    def is_unknown(self, predicate: str, arguments: Sequence[str] = ()) -> bool:
        """Critical Open World rule: Returns True if neither asserted true nor false."""
        return not self.is_known_true(predicate, arguments) and not self.is_known_false(predicate, arguments)

    def get_all_facts(self) -> List[Fact]:
        return list(self._facts.values())

    def get_provenance(self, fact: Fact) -> Dict[str, Any]:
        """Retrieve full provenance tree for a fact."""
        return {
            "fact": repr(fact),
            "epistemic_status": fact.epistemic_status.value,
            "evidence_source": fact.evidence_source,
            "generating_rule": fact.generating_rule,
            "supporting_facts": [self.get_provenance(p) for p in fact.supporting_facts],
        }

    def _record_history(self, action: str, fact: Fact) -> None:
        self._history.append({
            "action": action,
            "fact": repr(fact),
            "timestamp": str(fact.timestamp),
        })

    def clear(self) -> None:
        self._facts.clear()
        self._derived_by.clear()
        self._history.clear()