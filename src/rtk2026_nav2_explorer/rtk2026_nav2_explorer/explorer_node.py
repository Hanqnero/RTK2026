import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math
import numpy as np


FREE = 0
UNKNOWN = -1


class ExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("explore_period_sec", 5.0)
        self.declare_parameter("min_frontier_size", 5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("min_goal_distance_m", 1.0)
        self.declare_parameter("blacklist_radius_m", 0.75)
        self.declare_parameter("prefer_farther_frontiers", True)

        self._map_topic = self.get_parameter("map_topic").value
        self._explore_period = float(self.get_parameter("explore_period_sec").value)
        self._min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._min_goal_distance_m = float(self.get_parameter("min_goal_distance_m").value)
        self._blacklist_radius_m = float(self.get_parameter("blacklist_radius_m").value)
        self._prefer_farther_frontiers = bool(self.get_parameter("prefer_farther_frontiers").value)

        self._map_data = None
        self._visited_frontiers = set()
        self._blacklisted_goals = []
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._idle = True
        self._goal_handle = None
        self._bootstrap_sent = False
        self._last_goal_xy = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._map_cb, map_qos
        )
        self._timer = self.create_timer(
            self._explore_period, self._explore_once
        )

        self.get_logger().info(
            "Explorer started: map=%s, period=%.1fs"
            % (self._map_topic, self._explore_period)
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map_data = msg

    def _robot_pose_in_map(self):
        try:
            when = Time()
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_base_frame,
                when,
                timeout=Duration(seconds=1.5),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except TransformException as e:
            self.get_logger().debug("TF map->base_link: %s" % (e,))
            return None

    def _world_to_map(self, x: float, y: float):
        if self._map_data is None:
            return None
        ox = self._map_data.info.origin.position.x
        oy = self._map_data.info.origin.position.y
        res = self._map_data.info.resolution
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        return (r, c)

    def _map_to_world(self, r: int, c: int):
        if self._map_data is None:
            return (0.0, 0.0)
        ox = self._map_data.info.origin.position.x
        oy = self._map_data.info.origin.position.y
        res = self._map_data.info.resolution
        x = c * res + ox + res * 0.5
        y = r * res + oy + res * 0.5
        return (x, y)

    def _find_frontiers(self, grid: np.ndarray):
        rows, cols = grid.shape
        frontiers = []
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if grid[r, c] != FREE:
                    continue
                neighbors = grid[r - 1 : r + 2, c - 1 : c + 2].flatten()
                if UNKNOWN in neighbors:
                    frontiers.append((r, c))
        return frontiers

    def _cluster_frontiers(self, frontiers):
        if not frontiers:
            return []
        fset = set(frontiers)
        clusters = []
        seen = set()

        def neighbors(rc):
            r, c = rc
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    n = (r + dr, c + dc)
                    if n in fset and n not in seen:
                        yield n

        for start in frontiers:
            if start in seen:
                continue
            stack = [start]
            cluster = []
            while stack:
                rc = stack.pop()
                if rc in seen:
                    continue
                seen.add(rc)
                cluster.append(rc)
                for n in neighbors(rc):
                    stack.append(n)
            if len(cluster) >= self._min_frontier_size:
                clusters.append(cluster)
        return clusters

    def _cluster_center(self, cluster):
        r_avg = sum(r for r, _ in cluster) / len(cluster)
        c_avg = sum(c for _, c in cluster) / len(cluster)
        return (int(round(r_avg)), int(round(c_avg)))

    def _choose_frontier(self, clusters, robot_row, robot_col):
        best = None
        best_score = -float("inf")
        for cluster in clusters:
            r, c = self._cluster_center(cluster)
            key = (r, c)
            if key in self._visited_frontiers:
                continue
            d_cells = float(np.sqrt((robot_row - r) ** 2 + (robot_col - c) ** 2))
            score = d_cells if self._prefer_farther_frontiers else -d_cells
            if score > best_score:
                best_score = score
                best = key
        return best

    def _explore_once(self) -> None:
        if not self._idle:
            return
        if self._map_data is None:
            self.get_logger().info("Waiting for /map from slam_toolbox (move robot or wait for first scan)")
            return
        pose = self._robot_pose_in_map()
        if pose is None:
            self.get_logger().info("Waiting for TF map->base_link (slam_toolbox and odom must be running)")
            return

        robot_x, robot_y, robot_yaw = pose
        rc = self._world_to_map(robot_x, robot_y)
        if rc is None:
            return
        robot_row, robot_col = rc

        grid = np.array(self._map_data.data, dtype=np.int32).reshape(
            (self._map_data.info.height, self._map_data.info.width)
        )
        frontiers = self._find_frontiers(grid)
        if not frontiers:
            if not self._bootstrap_sent:
                free_count = int(np.sum(grid == FREE))
                if free_count < 50:
                    self.get_logger().info("No frontiers yet; sending bootstrap goal 1.5 m ahead")
                    self._bootstrap_sent = True
                    dist = 1.5
                    goal_x = robot_x + dist * math.cos(robot_yaw)
                    goal_y = robot_y + dist * math.sin(robot_yaw)
                    self._send_goal(goal_x, goal_y)
                    return
            self.get_logger().info("No frontiers; exploration may be complete")
            return

        clusters = self._cluster_frontiers(frontiers)
        chosen = self._choose_frontier(clusters, robot_row, robot_col)
        if chosen is None:
            self.get_logger().info("No unvisited frontier cluster")
            return

        goal_x, goal_y = self._map_to_world(chosen[0], chosen[1])
        d_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if d_goal < self._min_goal_distance_m:
            self._visited_frontiers.add(chosen)
            self.get_logger().info("Skipping too-close frontier (%.2fm)" % (d_goal,))
            return
        for bx, by in self._blacklisted_goals:
            if math.hypot(goal_x - bx, goal_y - by) < self._blacklist_radius_m:
                self._visited_frontiers.add(chosen)
                self.get_logger().info("Skipping blacklisted-area frontier")
                return

        self._visited_frontiers.add(chosen)
        self._send_goal(goal_x, goal_y)

    def _send_goal(self, x: float, y: float) -> None:
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self._map_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not ready")
            return

        self._idle = False
        self._last_goal_xy = (x, y)
        self.get_logger().info("Navigating to frontier (%.2f, %.2f)" % (x, y))
        self._send_goal_future = self._nav_client.send_goal_async(
            nav_goal, feedback_callback=self._feedback_cb
        )
        self._send_goal_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        del feedback_msg

    def _goal_response_cb(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            self._idle = True
            return
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        try:
            wrapped = future.result()
            status = wrapped.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation succeeded")
            else:
                self.get_logger().warn("Navigation finished with status=%d" % (status,))
                if self._last_goal_xy is not None:
                    self._blacklisted_goals.append(self._last_goal_xy)
        except Exception as e:
            self.get_logger().warn("Navigation failed: %s" % (e,))
            if self._last_goal_xy is not None:
                self._blacklisted_goals.append(self._last_goal_xy)
        self._idle = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

