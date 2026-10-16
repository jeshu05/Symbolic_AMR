"""Frontier detection, clustering, safety filtering, and target candidate generation."""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple
import numpy as np
from scipy.ndimage import label
from explorer_mapping.grid_map import GridMap


class FrontierCandidate:
    """A feasible candidate exploration target generated from a frontier cluster."""

    def __init__(
        self,
        target_id: int,
        wx: float,
        wy: float,
        yaw: float,
        cluster_size: int,
        expected_info_gain: float,
        clearance: float,
    ) -> None:
        self.target_id = target_id
        self.wx = float(wx)
        self.wy = float(wy)
        self.yaw = float(yaw)
        self.cluster_size = int(cluster_size)
        self.expected_info_gain = float(expected_info_gain)
        self.clearance = float(clearance)

    def distance_to(self, other_wx: float, other_wy: float) -> float:
        return math.hypot(self.wx - other_wx, self.wy - other_wy)

    def __repr__(self) -> str:
        return (
            f"FrontierCandidate(id={self.target_id}, pos=({self.wx:.2f}, {self.wy:.2f}), "
            f"gain={self.expected_info_gain:.1f}, clearance={self.clearance:.2f}m)"
        )


class FrontierCluster:
    """A connected component of frontier cells."""

    def __init__(self, cells: Sequence[Tuple[int, int]]) -> None:
        self.cells = list(cells)

    @property
    def size(self) -> int:
        return len(self.cells)


class FrontierDetector:
    """Extracts frontier boundaries, clusters them, filters unsafe poses, and returns candidates."""

    def __init__(
        self,
        min_cluster_size: int = 5,
        min_clearance: float = 0.28,
        min_candidate_distance: float = 0.7,
    ) -> None:
        self.min_cluster_size = min_cluster_size
        self.min_clearance = min_clearance
        self.min_candidate_distance = min_candidate_distance

    def find_frontier_mask(self, grid: GridMap) -> np.ndarray:
        """Finds boolean mask of frontier cells (free cells adjacent to unknown space)."""
        is_free_mask = (grid.data >= 0) & (grid.data < grid.free_threshold)
        is_unknown_mask = (grid.data < 0)

        # A free cell is a frontier if at least one 8-neighbor is unknown
        # Shift mask in 8 directions
        unknown_neighbor = np.zeros_like(is_free_mask, dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.zeros_like(is_unknown_mask)
                # Slice logic
                src_y_slice = slice(max(0, -dy), grid.height - max(0, dy))
                dst_y_slice = slice(max(0, dy), grid.height - max(0, -dy))
                src_x_slice = slice(max(0, -dx), grid.width - max(0, dx))
                dst_x_slice = slice(max(0, dx), grid.width - max(0, -dx))

                shifted[dst_y_slice, dst_x_slice] = is_unknown_mask[src_y_slice, src_x_slice]
                unknown_neighbor |= shifted

        # Frontier cells must be free AND have an unknown neighbor
        frontier_mask = is_free_mask & unknown_neighbor
        return frontier_mask

    def detect_candidates(
        self,
        grid: GridMap,
        robot_wx: Optional[float] = None,
        robot_wy: Optional[float] = None,
    ) -> List[FrontierCandidate]:
        """Detect and return filtered, deduplicated frontier candidate goals."""
        frontier_mask = self.find_frontier_mask(grid)
        if not np.any(frontier_mask):
            return []

        # Distance transform for obstacle clearance
        dist_map = grid.compute_obstacle_distance_map()

        # Connected component labeling (8-connectivity)
        structure = np.ones((3, 3), dtype=int)
        labeled_array, num_features = label(frontier_mask, structure=structure)

        candidates: List[FrontierCandidate] = []
        target_counter = 1

        for cluster_id in range(1, num_features + 1):
            cell_indices = np.argwhere(labeled_array == cluster_id)  # (gy, gx)
            cluster_size = len(cell_indices)
            if cluster_size < self.min_cluster_size:
                continue

            # Candidate point: choose the cell with the highest obstacle clearance in this cluster
            best_idx = 0
            best_clearance = -1.0
            for i, (gy, gx) in enumerate(cell_indices):
                c = dist_map[gy, gx]
                if c > best_clearance:
                    best_clearance = c
                    best_idx = i

            gy_chosen, gx_chosen = cell_indices[best_idx]
            clearance = float(dist_map[gy_chosen, gx_chosen])

            # Safety check: must meet minimum clearance from walls
            if clearance < self.min_clearance:
                continue

            wx, wy = grid.grid_to_world(int(gx_chosen), int(gy_chosen))

            # Compute orientation facing toward the unknown centroid of the cluster
            gy_mean = float(np.mean(cell_indices[:, 0]))
            gx_mean = float(np.mean(cell_indices[:, 1]))
            centroid_wx, centroid_wy = grid.grid_to_world(int(round(gx_mean)), int(round(gy_mean)))

            # If robot pose is provided, ensure candidate is facing outward
            if robot_wx is not None and robot_wy is not None:
                yaw = math.atan2(wy - robot_wy, wx - robot_wx)
            else:
                yaw = math.atan2(centroid_wy - wy, centroid_wx - wx)

            # Expected information gain scales with cluster size and visibility
            expected_info_gain = float(cluster_size * grid.resolution * 10.0)

            candidates.append(
                FrontierCandidate(
                    target_id=target_counter,
                    wx=wx,
                    wy=wy,
                    yaw=yaw,
                    cluster_size=cluster_size,
                    expected_info_gain=expected_info_gain,
                    clearance=clearance,
                )
            )
            target_counter += 1

        # Deduplicate candidates that are too close to each other
        return self._deduplicate(candidates)

    def _deduplicate(self, candidates: List[FrontierCandidate]) -> List[FrontierCandidate]:
        if not candidates:
            return []

        # Sort by expected info gain descending
        sorted_candidates = sorted(candidates, key=lambda c: c.expected_info_gain, reverse=True)
        accepted: List[FrontierCandidate] = []

        for cand in sorted_candidates:
            too_close = False
            for acc in accepted:
                if cand.distance_to(acc.wx, acc.wy) < self.min_candidate_distance:
                    too_close = True
                    break
            if not too_close:
                accepted.append(cand)

        # Re-index target IDs
        for i, cand in enumerate(accepted, 1):
            cand.target_id = i

        return accepted