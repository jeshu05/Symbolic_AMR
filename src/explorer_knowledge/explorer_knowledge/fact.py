"""Explicit symbolic fact representation with epistemic status and provenance."""
from __future__ import annotations

from enum import Enum
import time
from typing import Any, Dict, Optional, Sequence, Tuple


class EpistemicStatus(str, Enum):
    """Epistemic modality for symbolic facts.
    
    Critical Epistemic Rule: 'Unknown is not false'.
    - OBSERVED: Directly perceived through robot sensors or map detectors.
    - INFERRED: Derived through rule application on supporting facts.
    - ASSUMED: Initial prior or working hypothesis subject to verification.
    - UNKNOWN: Open-world unknown state; neither asserted nor refuted.
    """
    OBSERVED = "OBSERVED"
    INFERRED = "INFERRED"
    ASSUMED = "ASSUMED"
    UNKNOWN = "UNKNOWN"


class Fact:
    """Represents an atomic first-order symbolic fact with provenance and certainty."""

    def __init__(
        self,
        predicate: str,
        arguments: Sequence[str] = (),
        confidence: float = 1.0,
        evidence_source: str = "internal",
        timestamp: Optional[float] = None,
        is_negated: bool = False,
        epistemic_status: EpistemicStatus = EpistemicStatus.OBSERVED,
        supporting_facts: Optional[Sequence[Fact]] = None,
        generating_rule: Optional[str] = None,
    ) -> None:
        self.predicate = str(predicate).strip()
        self.arguments: Tuple[str, ...] = tuple(str(a).strip() for a in arguments)
        self.confidence = float(max(0.0, min(1.0, confidence)))
        self.evidence_source = str(evidence_source)
        self.timestamp = float(timestamp if timestamp is not None else time.time())
        self.is_negated = bool(is_negated)
        self.epistemic_status = (
            epistemic_status
            if isinstance(epistemic_status, EpistemicStatus)
            else EpistemicStatus(epistemic_status)
        )
        self.supporting_facts: Tuple[Fact, ...] = tuple(supporting_facts or ())
        self.generating_rule = generating_rule

    @property
    def signature(self) -> str:
        """Returns the predicate signature without polarity, e.g. RoomKnown/1."""
        return f"{self.predicate}/{len(self.arguments)}"

    def key(self) -> Tuple[str, Tuple[str, ...], bool]:
        """Unique logical key: (predicate, arguments, is_negated)."""
        return (self.predicate, self.arguments, self.is_negated)

    def matches_pattern(
        self,
        predicate: str,
        arguments: Optional[Sequence[Optional[str]]] = None,
        is_negated: Optional[bool] = None,
    ) -> bool:
        """Check if this fact matches a pattern with optional wildcards (None)."""
        if self.predicate != predicate:
            return False
        if is_negated is not None and self.is_negated != is_negated:
            return False
        if arguments is not None:
            if len(self.arguments) != len(arguments):
                return False
            for actual, pattern in zip(self.arguments, arguments):
                if pattern is not None and actual != pattern:
                    return False
        return True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "predicate": self.predicate,
            "arguments": list(self.arguments),
            "confidence": self.confidence,
            "evidence_source": self.evidence_source,
            "timestamp": self.timestamp,
            "is_negated": self.is_negated,
            "epistemic_status": self.epistemic_status.value,
            "generating_rule": self.generating_rule,
            "supporting_facts": [f.to_dict() for f in self.supporting_facts],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> Fact:
        supporting = [cls.from_dict(d) for d in data.get("supporting_facts", [])]
        return cls(
            predicate=data["predicate"],
            arguments=data.get("arguments", ()),
            confidence=data.get("confidence", 1.0),
            evidence_source=data.get("evidence_source", "deserialized"),
            timestamp=data.get("timestamp"),
            is_negated=data.get("is_negated", False),
            epistemic_status=EpistemicStatus(data.get("epistemic_status", "OBSERVED")),
            supporting_facts=supporting,
            generating_rule=data.get("generating_rule"),
        )

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, Fact):
            return False
        return self.key() == other.key()

    def __hash__(self) -> int:
        return hash(self.key())

    def __repr__(self) -> str:
        prefix = "¬" if self.is_negated else ""
        args_str = ", ".join(self.arguments)
        return (
            f"{prefix}{self.predicate}({args_str}) "
            f"[{self.epistemic_status.value}, conf={self.confidence:.2f}]"
        )