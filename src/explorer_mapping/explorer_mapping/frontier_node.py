"""ROS 2 Node for online frontier detection and target candidate publishing."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from explorer_interfaces.msg import ExplorationTarget, ExplorationTargetArray
from explorer_mapping.grid_map import GridMap
from explorer_mapping.frontier_detector import FrontierDetector


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FrontierNode(Node):
    """Subscribes to /map and publishes candidate exploration goals and visualization markers."""

    def __init__(self) -> None:
        super().__init__('frontier_node')

        # Parameters
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('min_clearance', 0.28)
        self.declare_parameter('min_candidate_distance', 0.7)
        self.declare_parameter('detection_rate_hz', 1.0)

        min_cluster = self.get_parameter('min_cluster_size').value
        min_clear = self.get_parameter('min_clearance').value
        min_dist = self.get_parameter('min_candidate_distance').value

        self.detector = FrontierDetector(
            min_cluster_size=min_cluster,
            min_clearance=min_clear,
            min_candidate_distance=min_dist,
        )

        # QoS for /map (transient local, reliable)
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map_received,
            map_qos
        )

        self.pub_targets = self.create_publisher(
            ExplorationTargetArray,
            '~/candidate_targets',
            10
        )

        self.pub_markers = self.create_publisher(
            MarkerArray,
            '~/frontier_markers',
            10
        )

        self.current_grid: GridMap | None = None
        self.last_map_msg: OccupancyGrid | None = None

        rate = self.get_parameter('detection_rate_hz').value
        self.timer = self.create_timer(1.0 / max(0.1, rate), self._run_detection)

        self.get_logger().info('FrontierNode initialized and waiting for /map.')

    def _on_map_received(self, msg: OccupancyGrid) -> None:
        self.last_map_msg = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Data is row-major 1D array of shape (height * width)
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.current_grid = GridMap(
            data=data,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )

    def _run_detection(self) -> None:
        if self.current_grid is None or self.last_map_msg is None:
            return

        candidates = self.detector.detect_candidates(self.current_grid)

        target_array_msg = ExplorationTargetArray()
        target_array_msg.header.stamp = self.get_clock().now().to_msg()
        target_array_msg.header.frame_id = self.last_map_msg.header.frame_id or 'map'

        marker_array = MarkerArray()

        for cand in candidates:
            # Exploration target msg
            t_msg = ExplorationTarget()
            t_msg.target_id = cand.target_id
            t_msg.pose.header = target_array_msg.header
            t_msg.pose.pose.position.x = cand.wx
            t_msg.pose.pose.position.y = cand.wy
            t_msg.pose.pose.position.z = 0.0
            t_msg.pose.pose.orientation = yaw_to_quaternion(cand.yaw)
            t_msg.expected_info_gain = cand.expected_info_gain
            t_msg.is_safe = True
            target_array_msg.targets.append(t_msg)

            # RViz Marker (Arrow)
            marker = Marker()
            marker.header = target_array_msg.header
            marker.ns = 'frontier_candidates'
            marker.id = cand.target_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = t_msg.pose.pose
            marker.scale.x = 0.4  # length
            marker.scale.y = 0.08  # width
            marker.scale.z = 0.08  # height
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.9
            marker.color.a = 0.85
            marker_array.markers.append(marker)

        self.pub_targets.publish(target_array_msg)
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()