"""Simulated resource and energy proxy model."""
from __future__ import annotations

import math
from typing import Tuple


class ResourceModel:
    """Simulated energy/battery proxy tracking consumption and return-to-base feasibility."""

    def __init__(
        self,
        battery_capacity: float = 100.0,
        cost_per_meter: float = 1.2,
        cost_per_second_idle: float = 0.02,
        reserve_margin: float = 20.0,
    ) -> None:
        self.battery_capacity = float(battery_capacity)
        self.current_battery = float(battery_capacity)
        self.cost_per_meter = float(cost_per_meter)
        self.cost_per_second_idle = float(cost_per_second_idle)
        self.reserve_margin = float(reserve_margin)

    def check_return_feasibility(
        self,
        robot_x: float,
        robot_y: float,
        target_x: float,
        target_y: float,
        base_x: float = 0.0,
        base_y: float = 0.0,
    ) -> Tuple[bool, float, float]:
        """Check if the robot has enough energy to visit target and then return to base.
        
        Returns:
            (is_feasible, total_energy_required, remaining_battery_after_return)
        """
        dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
        dist_target_to_base = math.hypot(base_x - target_x, base_y - target_y)
        total_dist = dist_to_target + dist_target_to_base

        energy_required = total_dist * self.cost_per_meter
        remaining_after = self.current_battery - energy_required

        is_feasible = remaining_after >= self.reserve_margin
        return (is_feasible, energy_required, remaining_after)

    def consume_travel(self, distance_meters: float) -> None:
        used = max(0.0, distance_meters * self.cost_per_meter)
        self.current_battery = max(0.0, self.current_battery - used)

    def consume_time(self, seconds: float) -> None:
        used = max(0.0, seconds * self.cost_per_second_idle)
        self.current_battery = max(0.0, self.current_battery - used)

    @property
    def percentage(self) -> float:
        return (self.current_battery / self.battery_capacity) * 100.0