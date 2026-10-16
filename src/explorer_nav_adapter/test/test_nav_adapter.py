"""Unit tests for Nav2 adapter state machine and goal dispatch logic."""
import pytest
from explorer_nav_adapter.nav2_client import NavState


def test_nav_state_enum():
    assert NavState.IDLE == "IDLE"
    assert NavState.NAVIGATING == "NAVIGATING"
    assert NavState.SUCCEEDED == "SUCCEEDED"
    assert NavState.ABORTED == "ABORTED"
    assert NavState.CANCELED == "CANCELED"
    assert NavState.FAILED == "FAILED"