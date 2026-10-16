# Symbolic Autonomous Exploration and Mission Reasoning 2.0 (Symbolic Explorer)

A ROS 2 autonomous exploration and symbolic mission reasoning system running in WSL2 Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic (8.15.0). The system explores initially unknown indoor facilities, reasons about mission objectives and epistemic provenance under open-world semantics, enforces hard safety gates, executes physical navigation via Nav2, dynamically adapts to route blockages, and generates traceable explanations for every decision.

---

## 1. System Architecture and Packages

The system comprises 8 decoupled ROS 2 packages adhering to standard separation of concerns:

- [explorer_interfaces](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_interfaces): Custom ROS 2 IDL definitions:
  - `TargetResult.msg`: Closed-loop navigation execution feedback (`target_id`, `pose`, `success`, `error_msg`, `execution_time_seconds`).
  - `DecisionExplanation.msg`: Structured explanation record with provenance rules, scores, and constraint audits.
  - `ExplorationTarget.msg` & `ExplorationTargetArray.msg`: Frontier goal representations with geometry, information gain, and safety flags.
  - `MissionStatus.msg`: Real-time mission lifecycle, completed/pending objectives, and modeled energy reserves.
  - `SymbolicFact.msg`: First-order epistemic fact representation (`predicate`, `arguments`, `status`, `confidence`).
- [explorer_sim](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_sim): Gazebo Harmonic simulation environment. Contains `research_facility_world.sdf` (36m × 36m navigable indoor research facility) and the procedural generator `generate_facility.py`.
- [explorer_mapping](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_mapping): Online SLAM integration, 8-connected frontier cell clustering, obstacle clearance safety filtration, and spatial deduplication from live `/map` occupancy grids.
- [explorer_knowledge](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_knowledge): First-order symbolic knowledge base, epistemic status tracking (`OBSERVED`, `INFERRED`, `ASSUMED`, `UNKNOWN`), truth maintenance system (TMS) with alternative support tracking, and forward-chaining rule engine.
- [explorer_mission](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_mission): Mission planner, hard safety gate auditor, modeled energy budget proxy, multi-criteria utility function, and explainable decision logger (`~/.ros/symbolic_explorer_logs/decisions.jsonl`).
- [explorer_nav_adapter](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_nav_adapter): Action client bridge to Nav2 `NavigateToPose`. Handles server lifecycle, execution tracking, timeout monitoring, goal debouncing, and publishes `TargetResult` upon goal completion or abort.
- [explorer_evaluation](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_evaluation): Empirical benchmark suite. Includes `gazebo_benchmark.py` for live simulation telemetry (physical path length, mapped area, restricted zone breaches, RTF) and `benchmark_runner.py` for algorithmic baseline comparisons.
- [explorer_bringup](file:///home/jesvanth/symbolic_explorer_ws/src/explorer_bringup): Master launch scripts (`symbolic_explorer.launch.py`, `simulation_and_nav.launch.py`, `exploration_stack.launch.py`), tuned SLAM and Nav2 configuration files, and RViz layouts.

---

## 2. Root Cause Analysis & Stationary Robot Resolution

The baseline system exhibited complete immobility due to four interacting defects across the simulation and control stack:

1. **LiDAR Self-Hit Reflection:** In `gz_waffle.sdf.xacro`, `<min>0.00001</min>` caused LiDAR rays to reflect off the robot chassis at 0.12 m. `slam_toolbox` default `min_laser_range: 0.0` ingested these self-hits, triggering Ceres scan-matching divergence where estimated robot pose jumped to `x=436m, y=198m`.
   - *Fix:* Enforced `min_laser_range: 0.22` in `slam_params.yaml` and `nav2_params.yaml`, rejecting chassis self-reflections.
2. **Spawn Pose / Map Bounds Mismatch:** Upstream `tb3_simulation_launch.py` hardcoded `x_pose: -2.00` without declaring it as a launch argument. SLAM initialized its origin at `[-0.95, -2.16]`, placing the robot outside the active costmap boundary and triggering Nav2 `START_OUTSIDE_MAP = 203`.
   - *Fix:* Spawning explicitly at `(0.0, 0.0)` at the center of the base station hub, ensuring both odometry and initial map coordinates are aligned and centered.
3. **Orphan Multi-Instance Collision:** Residual `gz sim` background processes sharing the same Gazebo transport partition published conflicting simulation clocks, causing continuous `TF_OLD_DATA` transform buffer rejections in Nav2 costmaps.
   - *Fix:* Guaranteed single-instance process execution and cleanup scripts before launch.
4. **Missing Container in Nav2 Composition:** Direct invocation of `navigation_launch.py` with `use_composition:=True` had no active `nav2_container`.
   - *Fix:* Integrated `bringup_launch.py` with parameter injection, launching `nav2_container`, `slam_toolbox`, and navigation plugins together in an isolated composition container.

---

## 3. Environment: 36m × 36m Research Facility World

The small demonstration world was replaced with a comprehensive indoor research complex:
- **Dimensions**: 36 m × 36 m navigable indoor floorplan with 0.15 m wall thickness and 2.5 m ceiling height.
- **Key Functional Rooms**:
  - Base Hub & Charging Station at `(0.0, 0.0)`
  - Medical / First Aid Station at `(11.0, 11.0)`
  - Chemical Storage & Hazard Inspection Zone at `(11.0, -11.0)`
  - IT Server Infrastructure Room at `(-11.0, 11.0)`
  - Engineering Workshop at `(-11.0, 0.0)`
  - Main Logistics Warehouse at `(-11.0, -11.0)`
  - Conference & Operations Facility at `(0.0, 12.0)`
  - Maintenance Service Bay at `(0.0, -12.0)`
- **Safety Semantics**:
  - **Quarantine Zone Alpha**: Restricted keep-out area at $x \in [7.0, 15.0]$, $y \in [-3.5, 3.5]$ demarcated with physical hazard warning barriers.
  - The autonomous agent receives the keep-out boundary solely as a mission constraint, with zero access to hidden world geometry.

---

## 4. Build and Launch Instructions

### Prerequisites
- Ubuntu 24.04 LTS under WSL2
- ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
- Gazebo Harmonic (`gz sim 8.15.0`)

### Build
```bash
source /opt/ros/jazzy/setup.bash
cd ~/symbolic_explorer_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Full Autonomous Exploration Demo (Headless)
```bash
ros2 launch explorer_bringup symbolic_explorer.launch.py headless:=True use_rviz:=False
```

### Run Demonstration with Gazebo GUI and RViz Visualization
```bash
ros2 launch explorer_bringup symbolic_explorer.launch.py headless:=False use_rviz:=True
```

### Run Comparative Baselines Benchmark Suite
```bash
ros2 run explorer_evaluation run_benchmarks
```

### Run Empirical Gazebo Telemetry Benchmark Node
```bash
ros2 run explorer_evaluation run_gazebo_benchmark
```

---

## 5. Decision Explainability & Provenance

Every target selection emits a traceable `DecisionExplanation` logged to `~/.ros/symbolic_explorer_logs/decisions.jsonl`:
```json
{
  "timestamp": 1791980014.95,
  "selected_target_id": 1,
  "selected_coords": [3.24, -3.14],
  "mission_objective": "locate_first_aid",
  "scores": {
    "mission_relevance": 0.06,
    "information_gain": 1.0,
    "travel_cost": 0.65,
    "risk_estimate": 0.0,
    "total_utility": 0.19
  },
  "hard_constraint_results": ["PassedRestrictedZone", "PassedClearance", "PassedResourceFeasibility"],
  "resource_feasibility": {"is_feasible": true, "battery_percent": 100.0},
  "supporting_facts": ["ForbidNavigationIn(restricted_zone_alpha) [INFERRED, conf=1.00]"],
  "inference_rules": ["R_MULTI_CRITERIA_UTILITY"],
  "selection_reason": "Selected Target 1 at (3.24, -3.14) with utility=0.190. Satisfies all hard safety & resource constraints."
}
```

---

## 6. Automated Testing

Run the full test suite across all 8 packages:
```bash
colcon test && colcon test-result --verbose
```
**Status:** 19/19 unit and integration tests passing with 0 errors, 0 failures.
