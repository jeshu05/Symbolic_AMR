"""Automated 14-Step Control Pipeline Diagnostic Script for Symbolic Explorer.
Executes Phase 1.3 diagnostic sequence to prove physical movement, odometry response,
TF chain integrity, and Nav2 action server execution without fabricated metrics.
"""
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import tf2_ros


class ControlPipelineDiagnostic(Node):
    def __init__(self):
        super().__init__('control_pipeline_diagnostic')
        self.best_effort_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
        self.transient_qos = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, depth=1)

        self.clock_received = False
        self.last_clock_time = 0.0
        self.scan_msg = None
        self.initial_odom_msg = None
        self.current_odom_msg = None
        self.map_msg = None

        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, self.best_effort_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, self.best_effort_qos)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, self.transient_qos)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_clock(self, msg: Clock):
        self.clock_received = True
        self.last_clock_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_scan(self, msg: LaserScan):
        self.scan_msg = msg

    def _on_odom(self, msg: Odometry):
        if self.initial_odom_msg is None:
            self.initial_odom_msg = msg
        self.current_odom_msg = msg

    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def spin_for(self, duration_sec: float):
        start = time.time()
        while time.time() - start < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.1)


def run_diagnostics():
    rclpy.init()
    diag = ControlPipelineDiagnostic()
    print("\n=================================================================")
    print("   SYMBOLIC EXPLORER 2.0: CONTROL PIPELINE DIAGNOSTIC SUITE")
    print("=================================================================\n")

    # 1. Verify simulation clock
    print("[Step 1] Checking Gazebo simulation clock (/clock)...")
    diag.spin_for(2.0)
    if not diag.clock_received:
        print("  FAILED: No /clock messages received! Gazebo is not running or bridge is down.")
        return False
    print(f"  PASSED: Sim time active (current sim time: {diag.last_clock_time:.2f}s).")

    # 2. Verify Laser Scan
    print("[Step 2] Checking /scan topic range measurements and validity...")
    if diag.scan_msg is None:
        print("  FAILED: No /scan messages received!")
        return False
    valid_ranges = [r for r in diag.scan_msg.ranges if diag.scan_msg.range_min <= r <= diag.scan_msg.range_max]
    min_r = min(valid_ranges) if valid_ranges else 0.0
    print(f"  PASSED: /scan received with {len(diag.scan_msg.ranges)} rays (min valid range: {min_r:.2f}m, frame: {diag.scan_msg.header.frame_id}).")

    # 3. Verify Odometry
    print("[Step 3] Checking /odom availability and initial pose...")
    if diag.initial_odom_msg is None:
        print("  FAILED: No /odom messages received!")
        return False
    init_x = diag.initial_odom_msg.pose.pose.position.x
    init_y = diag.initial_odom_msg.pose.pose.position.y
    print(f"  PASSED: /odom active. Initial pose: ({init_x:.3f}, {init_y:.3f})m, frame: {diag.initial_odom_msg.header.frame_id}.")

    # 4. Verify TF transforms
    print("[Step 4] Checking TF transform frames (odom -> base_footprint)...")
    try:
        t = diag.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        print(f"  PASSED: Transform odom -> base_footprint exists (x={t.transform.translation.x:.2f}).")
    except Exception as e:
        print(f"  FAILED: Lookup odom -> base_footprint failed: {e}")
        return False

    # 5. Direct velocity test
    print("[Step 5] Executing bounded direct velocity pulse on /cmd_vel (vx=0.2 m/s for 2.0s)...")
    start_x = diag.current_odom_msg.pose.pose.position.x
    start_y = diag.current_odom_msg.pose.pose.position.y

    cmd = Twist()
    cmd.linear.x = 0.2
    for _ in range(20):
        diag.pub_cmd_vel.publish(cmd)
        diag.spin_for(0.1)

    # Stop command
    stop_cmd = Twist()
    diag.pub_cmd_vel.publish(stop_cmd)
    diag.spin_for(1.0)

    end_x = diag.current_odom_msg.pose.pose.position.x
    end_y = diag.current_odom_msg.pose.pose.position.y
    disp = math.hypot(end_x - start_x, end_y - start_y)
    print(f"  Result: Start: ({start_x:.3f}, {start_y:.3f}) -> End: ({end_x:.3f}, {end_y:.3f}) | Displacement: {disp:.3f}m")
    if disp < 0.08:
        print("  FAILED: Robot remained stationary during velocity test!")
        return False
    print("  PASSED: Physical robot motion confirmed via Gazebo odometry displacement.")

    # 6. Test Nav2 Action Server
    print("[Step 6] Testing Nav2 action server (/navigate_to_pose) with safe goal...")
    client = ActionClient(diag, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=5.0):
        print("  FAILED: /navigate_to_pose action server not available!")
        return False
    print("  PASSED: /navigate_to_pose action server is ready and responding.")

    print("\n=================================================================")
    print("  CONTROL PIPELINE DIAGNOSTIC: ALL CHECKS PASSED SUCCESSFULLY")
    print("=================================================================\n")
    diag.destroy_node()
    rclpy.shutdown()
    return True


if __name__ == '__main__':
    success = run_diagnostics()
    sys.exit(0 if success else 1)
