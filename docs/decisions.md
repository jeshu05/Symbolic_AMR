# Architectural & Engineering Decisions (ADR) - Symbolic Explorer 2.0

## ADR-01: Gazebo Harmonic Laser Range Filtering
- **Context:** TurtleBot3 Waffle LiDAR rays reflected off its own chassis at 0.12m when `range_min: 0.00001` was used in `gz_waffle.sdf.xacro`. `slam_toolbox` default `min_laser_range: 0.0` ingested these self-hits, triggering Ceres scan-matching divergence.
- **Decision:** Explicitly set `min_laser_range: 0.22` in `slam_params.yaml` and `nav2_params.yaml`.
- **Status:** Verified. Eliminates scan-matching drift and enables accurate online SLAM.

## ADR-02: Deterministic Center Spawning & Dynamic SLAM Bounds
- **Context:** Upstream `tb3_simulation_launch.py` hardcoded spawn location `x_pose: -2.00` without declaring launch arguments. As a result, SLAM initialized at `[-0.95, -2.16]`, causing Nav2 to abort immediately with `START_OUTSIDE_MAP = 203`.
- **Decision:** Spawn robot explicitly at `(0.0, 0.0)` in the center of the base station hub in `simulation_and_nav.launch.py`.
- **Status:** Verified. Robot and costmap origins align at `(0,0)`.

## ADR-03: Decoupled Closed-Loop Target Result Feedback
- **Context:** `nav_adapter_node` dispatched goals to Nav2 but did not communicate terminal outcomes back to `mission_node`. `mission_node` had no awareness of whether a goal succeeded, failed, or timed out.
- **Decision:** Introduce `TargetResult.msg` (`target_id`, `pose`, `success`, `error_msg`, `execution_time_seconds`) published by `nav_adapter_node` and subscribed by `mission_node`.
- **Status:** Verified. Enables dynamic route blockage recording and immediate adaptive replanning.

## ADR-04: Strict Isolation of Ground-Truth World Geometry
- **Context:** Autonomous exploration benchmarks frequently leak ground-truth maps or target coordinates into the agent.
- **Decision:** The agent receives zero ground-truth geometric knowledge. Only the online SLAM occupancy grid `/map` is consumed by `frontier_node`. Restricted zone coordinates are provided purely as symbolic keep-out invariants.
- **Status:** Verified. Evaluation metrics are measured externally via `gazebo_benchmark.py`.

## ADR-05: Goal-Stability Hysteresis
- **Context:** As `/map` updates at 1 Hz, frontier cluster scores vary slightly, causing naive planners to thrash and preempt valid in-flight navigation goals.
- **Decision:** If the robot is actively navigating towards a target that remains safe and feasible, `mission_node` retains the active goal and ignores minor candidate fluctuations.
- **Status:** Verified. Eliminates path oscillation and ensures goals reach terminal completion.
