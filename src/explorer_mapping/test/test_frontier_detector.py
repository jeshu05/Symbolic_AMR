"""Unit tests for grid map and frontier detection on synthetic occupancy grids."""
import numpy as np
import pytest
from explorer_mapping.grid_map import GridMap
from explorer_mapping.frontier_detector import FrontierDetector


def test_grid_map_transforms_and_queries():
    # 20x20 grid at 0.1m resolution, origin at (-1.0, -1.0)
    data = np.full((20, 20), -1, dtype=np.int8)
    grid = GridMap(data, resolution=0.1, origin_x=-1.0, origin_y=-1.0)

    # Center (0,0) in world should map to grid (10, 10)
    gx, gy = grid.world_to_grid(0.0, 0.0)
    assert gx == 10 and gy == 10

    # Grid (10, 10) back to world should be near (0.05, 0.05)
    wx, wy = grid.grid_to_world(10, 10)
    assert pytest.approx(wx, abs=0.01) == 0.05
    assert pytest.approx(wy, abs=0.01) == 0.05

    # Check bounds
    assert grid.in_bounds(0, 0)
    assert grid.in_bounds(19, 19)
    assert not grid.in_bounds(20, 20)
    assert not grid.in_bounds(-1, 0)


def test_all_unknown_yields_zero_frontiers():
    data = np.full((30, 30), -1, dtype=np.int8)
    grid = GridMap(data, resolution=0.1, origin_x=0.0, origin_y=0.0)
    detector = FrontierDetector(min_cluster_size=3)
    candidates = detector.detect_candidates(grid)
    assert len(candidates) == 0


def test_all_free_yields_zero_frontiers():
    data = np.zeros((30, 30), dtype=np.int8)
    grid = GridMap(data, resolution=0.1, origin_x=0.0, origin_y=0.0)
    detector = FrontierDetector(min_cluster_size=3)
    candidates = detector.detect_candidates(grid)
    assert len(candidates) == 0


def test_synthetic_room_with_doorway():
    """A 40x40 grid where a 20x20 room is free, surrounded by walls, with one doorway (opening to unknown)."""
    # -1 is unknown
    data = np.full((40, 40), -1, dtype=np.int8)
    # Free interior [10:30, 10:30]
    data[10:30, 10:30] = 0
    # Add walls (100) around room except at doorway x in [18:22], y=29
    data[10, 10:30] = 100
    data[10:30, 10] = 100
    data[10:30, 29] = 100
    data[29, 10:18] = 100
    data[29, 22:30] = 100
    # At [29, 18:22], it is free, and y=30 is unknown!

    grid = GridMap(data, resolution=0.1, origin_x=0.0, origin_y=0.0)
    detector = FrontierDetector(min_cluster_size=3, min_clearance=0.1)
    candidates = detector.detect_candidates(grid)

    assert len(candidates) >= 1
    # Candidate should be situated near doorway (x approx 1.8 to 2.2m, y approx 2.9m)
    c = candidates[0]
    assert 1.5 <= c.wx <= 2.5
    assert 2.5 <= c.wy <= 3.2
    assert c.clearance >= 0.1


def test_obstacle_clearance_filtering():
    """Ensure candidate is not placed too close to a wall."""
    data = np.full((30, 30), -1, dtype=np.int8)
    # Small free strip adjacent to a wall
    data[10:20, 10:20] = 0
    data[10:20, 9] = 100  # wall on left

    grid = GridMap(data, resolution=0.1, origin_x=0.0, origin_y=0.0)
    # Require 0.35m clearance
    detector = FrontierDetector(min_cluster_size=4, min_clearance=0.35)
    candidates = detector.detect_candidates(grid)

    for c in candidates:
        assert c.clearance >= 0.35