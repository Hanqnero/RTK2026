#!/usr/bin/env python3
"""Чистый runtime v3: GlobalPlannerV2 + LocalPlannerPointsV3 + NavigateThroughPoses."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Goals
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rtk2026_interfaces.msg import DrivingDetection
from visualization_msgs.msg import Marker

from rtk2026_graph import GlobalPlannerV2, RoadGraph, load_geojson_path, load_planner_v2_config_path, normalize_lane_mode
from rtk2026_graph.local_planner_points_v3 import LocalPlannerPointsV3


@dataclass
class PlannerStateV3:
    current_vertex: int
    target_vertex: Optional[int]
    active_lane_mode: str
    next_lane_mode: str
    limiter_edges: tuple[tuple[int, int], ...]


@dataclass
class PendingRouteSign:
    command: str
    class_id: str
    confidence: float
    box_area: float


@dataclass
class PendingStopAction:
    action: str
    class_id: str
    confidence: float
    box_area: float
    duration_sec: float


class LaneDecisionManagerV3(Node):
    def __init__(self) -> None:
        super().__init__("lane_decision_manager_v3")

        pkg = Path(get_package_share_directory("rtk2026_route_nav"))
        self.declare_parameter("graph_filepath", str(pkg / "config" / "graph.geojson"))
        self.declare_parameter("planner_v2_config_filepath", str(pkg / "config" / "lane_planner_v2.yaml"))
        self.declare_parameter("sign_direction_topology_filepath", str(pkg / "config" / "sign_direction_topology.yaml"))
        self.declare_parameter("driving_detection_topic", "/perception/driving_detection")
        self.declare_parameter("current_vertex", 5)
        self.declare_parameter("previous_vertex", -1)
        self.declare_parameter("detected_sign_target_vertex", -1)
        self.declare_parameter("active_lane_mode", "lane1")
        self.declare_parameter("direction_mode", "lane1")
        self.declare_parameter("tick_rate_hz", 2.0)
        self.declare_parameter("log_every_n_ticks", 1)
        self.declare_parameter("enable_nav2_action", True)
        self.declare_parameter("nav2_through_poses_action_name", "/navigate_through_poses")
        self.declare_parameter("nav2_goal_topic", "/goal_pose")
        self.declare_parameter("lane_right_offset_m", 0.20)
        # 0.0 => не таймаутить in-flight goal (избегаем лишних preempt/resend).
        self.declare_parameter("goal_result_timeout_sec", 0.0)
        # Если false, один и тот же (current,target,lane) не переотправляется бесконечно после fail.
        self.declare_parameter("retry_same_goal_on_failure", False)
        self.declare_parameter("advance_vertex_distance_threshold_m", 0.35)
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("enable_fallback_visit_balancing", True)
        self.declare_parameter("block_immediate_backtrack", True)

        graph_path = str(self.get_parameter("graph_filepath").value)
        planner_cfg_path = str(self.get_parameter("planner_v2_config_filepath").value)
        sign_topology_path = str(self.get_parameter("sign_direction_topology_filepath").value)
        tick_rate_hz = float(self.get_parameter("tick_rate_hz").value)
        self._log_every_n_ticks = max(1, int(self.get_parameter("log_every_n_ticks").value))

        self._graph: RoadGraph = load_geojson_path(graph_path)
        global_cfg, local_rules = load_planner_v2_config_path(
            planner_cfg_path,
            sign_direction_topology_path=sign_topology_path,
        )
        self._global_planner = GlobalPlannerV2(global_cfg)
        self._local_planner = LocalPlannerPointsV3(self._graph, local_rules)

        active_lane_raw = str(self.get_parameter("active_lane_mode").value)
        direction_lane_raw = str(self.get_parameter("direction_mode").value)
        init_lane = normalize_lane_mode(direction_lane_raw if active_lane_raw == "lane1" else active_lane_raw)
        current_vertex = int(self.get_parameter("current_vertex").value)
        self._state = PlannerStateV3(
            current_vertex=current_vertex,
            target_vertex=None,
            active_lane_mode=init_lane,
            next_lane_mode=init_lane,
            limiter_edges=(),
        )
        self._fallback_choice_counts: dict[int, int] = {}
        for lane_targets in global_cfg.lane_targets.values():
            for targets in lane_targets.values():
                for v in targets:
                    self._fallback_choice_counts[int(v)] = 0

        self._tick_idx = 0
        self._goal_in_flight = False
        self._last_goal_sent_ns = 0
        self._last_goal_signature: Optional[tuple[int, int, str]] = None
        self._last_failed_signature: Optional[tuple[int, int, str]] = None
        self._committed_target_vertex: Optional[int] = None
        self._committed_next_lane_mode: Optional[str] = None
        self._committed_final_goal_xy: Optional[tuple[float, float]] = None
        self._active_goal_handle: Optional[ClientGoalHandle] = None
        self._pending_route_sign: Optional[PendingRouteSign] = None
        self._pending_stop_action: Optional[PendingStopAction] = None
        self._pending_bus_detected = False
        self._pause_until_ns = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._target_marker_pub = self.create_publisher(Marker, "/lane_control_target_marker_v3", 10)
        self._nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, str(self.get_parameter("nav2_through_poses_action_name").value)
        )
        self.create_subscription(
            DrivingDetection,
            str(self.get_parameter("driving_detection_topic").value),
            self._on_driving_detection,
            10,
        )
        self.create_timer(1.0 / max(0.5, tick_rate_hz), self._tick)

        self.get_logger().info(
            "LaneDecisionManagerV3 started. "
            f"graph={graph_path}, planner_v2={planner_cfg_path}, current_vertex={current_vertex}, lane={init_lane}, "
            "local_planner=LocalPlannerPointsV3"
        )

    def _tick(self) -> None:
        self._tick_idx += 1
        current_vertex = int(self.get_parameter("current_vertex").value)
        previous_vertex = int(self.get_parameter("previous_vertex").value)
        sign_target = int(self.get_parameter("detected_sign_target_vertex").value)
        active_lane_mode = normalize_lane_mode(str(self.get_parameter("active_lane_mode").value))
        self._state.current_vertex = current_vertex
        self._state.active_lane_mode = active_lane_mode

        if self._goal_in_flight:
            timeout_s = float(self.get_parameter("goal_result_timeout_sec").value)
            now_ns = self.get_clock().now().nanoseconds
            if timeout_s > 0.0 and self._last_goal_sent_ns > 0 and (now_ns - self._last_goal_sent_ns) > int(timeout_s * 1e9):
                self._goal_in_flight = False
                self._last_goal_signature = None
                self.get_logger().warn("navigate_through_poses timeout; rearm.")
            else:
                self._log_state("waiting_nav2_result", previous_vertex, sign_target, (), "hold")
                return
        now_ns = self.get_clock().now().nanoseconds
        if self._pause_until_ns > now_ns:
            self._log_state("pause_after_stop", previous_vertex, sign_target, (), "hold")
            return

        visit_counts = self._fallback_choice_counts if bool(self.get_parameter("enable_fallback_visit_balancing").value) else {}
        try:
            step = self._global_planner.pick_next(
                current_vertex=current_vertex,
                previous_vertex=previous_vertex,
                active_lane_mode=active_lane_mode,
                sign_target_vertex=sign_target,
                route_sign_command=self._pending_route_sign.command if self._pending_route_sign is not None else None,
                visit_counts=visit_counts,
                block_immediate_backtrack=bool(self.get_parameter("block_immediate_backtrack").value),
            )
        except ValueError:
            self._state.target_vertex = None
            self._state.limiter_edges = ()
            self._log_state("no_target", previous_vertex, sign_target, (), "none")
            return

        target_vertex = int(step.chosen_target)
        segment_lane = normalize_lane_mode(step.active_lane_mode)
        next_lane = normalize_lane_mode(step.next_lane_mode)
        signature = (current_vertex, target_vertex, segment_lane)
        self._state.target_vertex = target_vertex
        self._state.next_lane_mode = next_lane

        if self._last_goal_signature == signature:
            self._log_state("active_hold", previous_vertex, sign_target, tuple(step.allowed_targets), step.pick_source)
            return
        if (
            not bool(self.get_parameter("retry_same_goal_on_failure").value)
            and self._last_failed_signature == signature
        ):
            self._log_state("failed_hold", previous_vertex, sign_target, tuple(step.allowed_targets), step.pick_source)
            return
        if bool(self.get_parameter("enable_nav2_action").value) and not self._nav_through_poses_client.server_is_ready():
            self._log_state("nav_server_wait", previous_vertex, sign_target, tuple(step.allowed_targets), step.pick_source)
            return

        try:
            seq = self._local_planner.build_goal_sequence(
                current_vertex=current_vertex,
                target_vertex=target_vertex,
                lane_mode=segment_lane,
                previous_vertex=previous_vertex,
                lane_right_offset_m=float(self.get_parameter("lane_right_offset_m").value),
            )
        except ValueError as e:
            self.get_logger().warn(str(e))
            self._state.limiter_edges = ()
            self._log_state("no_goal", previous_vertex, sign_target, tuple(step.allowed_targets), step.pick_source)
            return

        self._state.limiter_edges = tuple(seq[0].limiter_edges) if seq else ()
        poses: list[PoseStamped] = []
        for p in seq:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(p.x)
            msg.pose.position.y = float(p.y)
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(0.5 * float(p.yaw))
            msg.pose.orientation.w = math.cos(0.5 * float(p.yaw))
            poses.append(msg)
        self._publish_target_marker(poses[-1].pose.position.x, poses[-1].pose.position.y)
        self.get_logger().info(
            f"nav2_chain_v3 current={current_vertex} target={target_vertex} lane={segment_lane} next_lane={next_lane} "
            f"waypoints={len(poses)} points={[ (round(ps.pose.position.x,3), round(ps.pose.position.y,3)) for ps in poses ]}"
        )

        if not self._send_through_poses(tuple(poses)):
            self._log_state("send_failed", previous_vertex, sign_target, tuple(step.allowed_targets), step.pick_source)
            return

        self._last_goal_signature = signature
        self._last_failed_signature = None
        self._committed_target_vertex = target_vertex
        self._committed_next_lane_mode = next_lane
        self._committed_final_goal_xy = (float(poses[-1].pose.position.x), float(poses[-1].pose.position.y))
        self._log_state("active", previous_vertex, sign_target, tuple(step.allowed_targets), step.pick_source)
        if step.route_sign_applied:
            self._pending_route_sign = None

    def _send_through_poses(self, poses: tuple[PoseStamped, ...]) -> bool:
        if not bool(self.get_parameter("enable_nav2_action").value):
            return True
        if not self._nav_through_poses_client.wait_for_server(timeout_sec=0.2):
            self._last_goal_signature = None
            self.get_logger().warn("navigate_through_poses action server is not available yet.")
            return False
        goal = NavigateThroughPoses.Goal()
        stamped_goals = Goals()
        stamped_goals.header.frame_id = "map"
        stamped_goals.header.stamp = self.get_clock().now().to_msg()
        stamped_goals.goals = list(poses)
        goal.poses = stamped_goals
        future = self._nav_through_poses_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._goal_in_flight = True
        self._last_goal_sent_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"navigate_through_poses_send_v3 poses={len(poses)} "
            f"final=({poses[-1].pose.position.x:.3f},{poses[-1].pose.position.y:.3f})"
        )
        return True

    def _on_goal_response(self, future) -> None:
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._goal_in_flight = False
            # Rejected часто бывает в коротком окне активации bt_navigator/action.
            # Не блокируем сегмент permanently: позволяем повторить отправку.
            self._last_goal_signature = None
            self.get_logger().warn("navigate_through_poses goal rejected.")
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, gh=goal_handle: self._on_goal_result(fut, gh))

    def _on_goal_result(self, future, goal_handle: ClientGoalHandle) -> None:
        if self._active_goal_handle is not None and goal_handle != self._active_goal_handle:
            return
        self._active_goal_handle = None
        self._goal_in_flight = False
        result = future.result()
        if result is None:
            self._last_failed_signature = self._last_goal_signature
            self._last_goal_signature = None
            self.get_logger().warn("navigate_through_poses result is None.")
            return
        if int(result.status) != GoalStatus.STATUS_SUCCEEDED:
            self._last_failed_signature = self._last_goal_signature
            self._last_goal_signature = None
            self.get_logger().warn(f"navigate_through_poses status={int(result.status)}")
            return
        if self._committed_target_vertex is None:
            return
        robot_xy = self._robot_xy_in_map()
        if robot_xy is not None and self._committed_final_goal_xy is not None:
            dist = math.hypot(
                float(robot_xy[0]) - float(self._committed_final_goal_xy[0]),
                float(robot_xy[1]) - float(self._committed_final_goal_xy[1]),
            )
            threshold = float(self.get_parameter("advance_vertex_distance_threshold_m").value)
            if dist > threshold:
                self.get_logger().warn(
                    "navigate_through_poses succeeded with large final-goal delta; "
                    f"advance anyway: dist={dist:.3f} threshold={threshold:.3f}"
                )
        old_current = int(self.get_parameter("current_vertex").value)
        next_current = int(self._committed_target_vertex)
        next_lane = normalize_lane_mode(self._committed_next_lane_mode or str(self.get_parameter("active_lane_mode").value))
        self.set_parameters(
            [
                RclpyParameter("current_vertex", value=next_current),
                RclpyParameter("previous_vertex", value=old_current),
                RclpyParameter("active_lane_mode", value=next_lane),
            ]
        )
        if bool(self.get_parameter("enable_fallback_visit_balancing").value):
            self._fallback_choice_counts[next_current] = self._fallback_choice_counts.get(next_current, 0) + 1
        self._last_failed_signature = None
        self._last_goal_signature = None
        self._committed_target_vertex = None
        self._committed_next_lane_mode = None
        self._committed_final_goal_xy = None
        route_command = self._pending_route_sign.command if self._pending_route_sign is not None else "none"
        stop_action = self._pending_stop_action.action if self._pending_stop_action is not None else "none"
        if self._pending_stop_action is not None:
            duration_sec = max(0.0, float(self._pending_stop_action.duration_sec))
            self._pause_until_ns = self.get_clock().now().nanoseconds + int(duration_sec * 1e9)
        else:
            self._pause_until_ns = 0
        self.get_logger().info(
            "navigate_through_poses succeeded; "
            f"current_vertex={next_current} previous_vertex={old_current} lane={next_lane} "
            f"pending_route={route_command} pending_stop={stop_action} bus={'yes' if self._pending_bus_detected else 'no'}"
        )
        self._pending_stop_action = None
        self._pending_bus_detected = False

    def _on_driving_detection(self, msg: DrivingDetection) -> None:
        updated_route = False
        updated_stop = False
        updated_bus = False
        if msg.route_command:
            candidate = PendingRouteSign(
                command=str(msg.route_command),
                class_id=str(msg.route_class_id),
                confidence=float(msg.route_confidence),
                box_area=float(msg.route_box_area),
            )
            if self._pending_route_sign is None or self._is_better_detection(
                candidate.box_area,
                candidate.confidence,
                self._pending_route_sign.box_area,
                self._pending_route_sign.confidence,
            ):
                self._pending_route_sign = candidate
                updated_route = True

        if msg.stop_action:
            candidate = PendingStopAction(
                action=str(msg.stop_action),
                class_id=str(msg.stop_class_id),
                confidence=float(msg.stop_confidence),
                box_area=float(msg.stop_box_area),
                duration_sec=float(msg.stop_duration_sec),
            )
            if self._pending_stop_action is None or self._is_better_detection(
                candidate.box_area,
                candidate.confidence,
                self._pending_stop_action.box_area,
                self._pending_stop_action.confidence,
            ):
                self._pending_stop_action = candidate
                updated_stop = True

        if bool(msg.bus_detected):
            self._pending_bus_detected = True
            updated_bus = True

        if updated_route or updated_stop or updated_bus:
            self.get_logger().info(
                "driving_detection_applied "
                f"route={self._pending_route_sign.command if self._pending_route_sign is not None else 'none'} "
                f"stop={self._pending_stop_action.action if self._pending_stop_action is not None else 'none'} "
                f"bus={'yes' if self._pending_bus_detected else 'no'}"
            )

    @staticmethod
    def _is_better_detection(
        candidate_box_area: float,
        candidate_confidence: float,
        current_box_area: float,
        current_confidence: float,
    ) -> bool:
        if candidate_box_area > current_box_area:
            return True
        if candidate_box_area == current_box_area and candidate_confidence > current_confidence:
            return True
        return False

    def _robot_xy_in_map(self) -> Optional[tuple[float, float]]:
        base = str(self.get_parameter("robot_base_frame").value)
        try:
            t = self._tf_buffer.lookup_transform("map", base, rclpy.time.Time(), timeout=Duration(seconds=0.5))
            tr = t.transform.translation
            return (float(tr.x), float(tr.y))
        except Exception:
            return None

    def _publish_target_marker(self, x: float, y: float) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "lane_control_target_v3"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.14
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.95
        self._target_marker_pub.publish(marker)

    def _log_state(
        self,
        stage: str,
        previous_vertex: int,
        sign_target_vertex: int,
        allowed_targets: tuple[int, ...],
        pick_source: str,
    ) -> None:
        if self._tick_idx % self._log_every_n_ticks != 0:
            return
        self.get_logger().info(
            f"lane_state_v3 stage={stage} tick={self._tick_idx} current={self._state.current_vertex} "
            f"target={self._state.target_vertex} lane={self._state.active_lane_mode} next_lane={self._state.next_lane_mode} "
            f"pick={pick_source} prev={previous_vertex} sign_target={sign_target_vertex} allowed={allowed_targets} "
            f"limiter={self._state.limiter_edges} route_cmd={self._pending_route_sign.command if self._pending_route_sign else 'none'} "
            f"stop={self._pending_stop_action.action if self._pending_stop_action else 'none'} "
            f"bus={'yes' if self._pending_bus_detected else 'no'}"
        )


def main() -> None:
    rclpy.init()
    node = LaneDecisionManagerV3()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
