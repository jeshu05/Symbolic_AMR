"""2D Occupancy Grid representation and coordinate transformations."""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple
import numpy as np


class GridMap:
    """Represents a 2D occupancy grid with ROS semantics.
    
    Conventions:
      - Unknown: -1
      - Free: 0 to 49 (typically 0)
      - Occupied: 50 to 100 (typically 100)
    """

    def __init__(
        self,
        data: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        occupied_threshold: int = 50,
        free_threshold: int = 25,
    ) -> None:
        if data.ndim != 2:
            raise ValueError(f"GridMap expects 2D array, got shape {data.shape}")
        self.data = np.asarray(data, dtype=np.int8)
        self.height, self.width = self.data.shape  # rows=height (y), cols=width (x)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.occupied_threshold = occupied_threshold
        self.free_threshold = free_threshold

    @property
    def shape(self) -> Tuple[int, int]:
        return (self.height, self.width)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def get_cell(self, gx: int, gy: int) -> int:
        if not self.in_bounds(gx, gy):
            return -1
        return int(self.data[gy, gx])

    def is_free(self, gx: int, gy: int) -> bool:
        v = self.get_cell(gx, gy)
        return 0 <= v < self.free_threshold

    def is_occupied(self, gx: int, gy: int) -> bool:
        v = self.get_cell(gx, gy)
        return v >= self.occupied_threshold

    def is_unknown(self, gx: int, gy: int) -> bool:
        return self.get_cell(gx, gy) < 0

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int(math.floor((wx - self.origin_x) / self.resolution))
        gy = int(math.floor((wy - self.origin_y) / self.resolution))
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return (wx, wy)

    def get_neighbors(self, gx: int, gy: int, connectivity: int = 8) -> Sequence[Tuple[int, int]]:
        neighbors = []
        if connectivity == 4:
            offsets = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        else:
            offsets = [
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (-1, 1), (1, -1), (-1, -1)
            ]
        for dx, dy in offsets:
            nx, ny = gx + dx, gy + dy
            if self.in_bounds(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def compute_obstacle_distance_map(self) -> np.ndarray:
        """Compute Euclidean distance transform from all occupied cells in meters."""
        from scipy.ndimage import distance_transform_edt
        # Free and unknown cells are 1, occupied cells are 0
        non_occupied = (self.data < self.occupied_threshold)
        dist_pixels = distance_transform_edt(non_occupied)
        return dist_pixels * self.resolution