"""Rule definition for symbolic inference with variable unification."""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple
from explorer_knowledge.fact import EpistemicStatus, Fact


class Pattern:
    """A query pattern that can contain variables starting with '?'."""

    def __init__(
        self,
        predicate: str,
        arguments: Sequence[str] = (),
        is_negated: bool = False,
    ) -> None:
        self.predicate = predicate
        self.arguments = tuple(arguments)
        self.is_negated = is_negated

    def match(self, fact: Fact, bindings: Dict[str, str]) -> Optional[Dict[str, str]]:
        """Try matching fact against pattern under existing bindings."""
        if fact.predicate != self.predicate or fact.is_negated != self.is_negated:
            return None
        if len(fact.arguments) != len(self.arguments):
            return None

        new_bindings = dict(bindings)
        for pattern_arg, fact_arg in zip(self.arguments, fact.arguments):
            if pattern_arg.startswith("?"):
                # Variable
                if pattern_arg in new_bindings:
                    if new_bindings[pattern_arg] != fact_arg:
                        return None
                else:
                    new_bindings[pattern_arg] = fact_arg
            else:
                # Constant
                if pattern_arg != fact_arg:
                    return None

        return new_bindings


class Rule:
    """Forward-chaining inference rule with condition patterns and consequent generator."""

    def __init__(
        self,
        rule_id: str,
        name: str,
        antecedents: Sequence[Pattern],
        consequent_generator: Callable[[Dict[str, str], Sequence[Fact]], Optional[Fact]],
        priority: int = 10,
        description: str = "",
    ) -> None:
        self.rule_id = rule_id
        self.name = name
        self.antecedents = tuple(antecedents)
        self.consequent_generator = consequent_generator
        self.priority = priority
        self.description = description

    def evaluate(self, kb_facts: Sequence[Fact]) -> List[Tuple[Fact, Sequence[Fact]]]:
        """Find all valid substitutions that satisfy all antecedents and generate conclusions."""
        results: List[Tuple[Fact, Sequence[Fact]]] = []

        def match_chain(
            pattern_idx: int,
            current_bindings: Dict[str, str],
            matched_facts: List[Fact],
        ) -> None:
            if pattern_idx >= len(self.antecedents):
                # All antecedents matched
                consequent = self.consequent_generator(current_bindings, matched_facts)
                if consequent is not None:
                    # Attach provenance
                    consequent.supporting_facts = tuple(matched_facts)
                    consequent.generating_rule = self.rule_id
                    consequent.epistemic_status = EpistemicStatus.INFERRED
                    results.append((consequent, tuple(matched_facts)))
                return

            pattern = self.antecedents[pattern_idx]
            for fact in kb_facts:
                bindings = pattern.match(fact, current_bindings)
                if bindings is not None:
                    match_chain(pattern_idx + 1, bindings, matched_facts + [fact])

        match_chain(0, {}, [])
        return results

    def __repr__(self) -> str:
        return f"Rule({self.rule_id}: {self.name})"