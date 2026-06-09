import math
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from bumperbot_active_slam.entropy_utils import euclidean_distance
from bumperbot_active_slam.models import NavigationCandidate, Point2D


class NavigationDispatcher:
    STATE_IDLE = "IDLE"
    STATE_WAITING_FOR_SERVER = "WAITING_FOR_SERVER"
    STATE_NAVIGATING = "NAVIGATING"
    STATE_SUCCEEDED = "SUCCEEDED"
    STATE_FAILED = "FAILED"
    STATE_TIMED_OUT = "TIMED_OUT"

    def __init__(
        self,
        node,
        action_name: str,
        goal_timeout_sec: float,
        blacklist_radius_m: float,
        blacklist_timeout_sec: float,
        max_retries_per_frontier: int,
    ) -> None:
        self.node = node
        self.action_name = action_name
        self.goal_timeout_sec = goal_timeout_sec
        self.blacklist_radius_m = blacklist_radius_m
        self.blacklist_timeout_sec = blacklist_timeout_sec
        self.max_retries_per_frontier = max(1, max_retries_per_frontier)
        self.client = ActionClient(node, NavigateToPose, action_name)

        self.state = self.STATE_IDLE
        self.active_goal_handle = None
        self.active_goal_centroid: Optional[Point2D] = None
        self.active_goal_sent_time = None
        self.goal_response_pending = False
        self.selected_goal_candidate: Optional[NavigationCandidate] = None
        self.last_goal_centroid: Optional[Point2D] = None
        self.last_goal_robot_xy: Optional[Point2D] = None
        self.last_goal_source: str = "none"
        self.last_goal_result: str = "none"
        self.last_result_time = None
        self.blacklist: List[Tuple[Point2D, object]] = []
        self.retry_counts: Dict[Tuple[int, int], int] = {}

    def server_is_ready(self) -> bool:
        return self.client.server_is_ready()

    def is_busy(self) -> bool:
        return self.state == self.STATE_NAVIGATING or self.goal_response_pending

    def send_goal(self, candidate: NavigationCandidate, robot_xy: Point2D, frame_id: str) -> None:
        goal_xy = candidate.point_world
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_xy[0])
        goal_msg.pose.pose.position.y = float(goal_xy[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(
            math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        )

        self.state = self.STATE_WAITING_FOR_SERVER
        self.active_goal_centroid = goal_xy
        self.active_goal_sent_time = self.node.get_clock().now()
        self.last_goal_robot_xy = robot_xy
        self.last_goal_source = candidate.source
        self.last_goal_result = "pending"
        self.goal_response_pending = True
        self.selected_goal_candidate = candidate

        self.node.get_logger().info(
            "Sending NavigateToPose goal: state=%s frame=%s x=%.2f y=%.2f "
            "cluster_id=%d source=%s distance=%.2f score=%.3f cost=%s"
            % (
                self.state,
                frame_id,
                goal_xy[0],
                goal_xy[1],
                candidate.cluster_id,
                candidate.source,
                candidate.distance,
                candidate.score,
                "unknown" if candidate.cost is None else str(candidate.cost),
            )
        )
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def check_active_goal_timeout(self) -> None:
        if self.state != self.STATE_NAVIGATING or self.active_goal_sent_time is None:
            return

        elapsed = (self.node.get_clock().now() - self.active_goal_sent_time).nanoseconds / 1e9
        if elapsed < self.goal_timeout_sec:
            return

        self.state = self.STATE_TIMED_OUT
        self.node.get_logger().warn(
            "NavigateToPose goal timed out after %.1f s; canceling and blacklisting" % elapsed
        )
        self.add_blacklist(self.active_goal_centroid)
        if self.active_goal_handle is not None:
            try:
                cancel_future = self.active_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)
            except Exception as exc:
                self.node.get_logger().warn("Cancel request failed: %s" % exc)
        self._clear_active_goal(keep_state=True)

    def cancel_active_goal(self, add_to_blacklist: bool, reason: str) -> None:
        if self.active_goal_centroid is None:
            return
        self.node.get_logger().warn("Canceling active NavigateToPose goal: reason=%s" % reason)
        self.last_goal_result = "CANCELED"
        self.last_result_time = self.node.get_clock().now()
        if add_to_blacklist:
            self.add_blacklist(self.active_goal_centroid)
        if self.active_goal_handle is not None:
            try:
                cancel_future = self.active_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)
            except Exception as exc:
                self.node.get_logger().warn("Cancel request failed: %s" % exc)
        self.state = self.STATE_FAILED
        self._clear_active_goal(keep_state=True)

    def expire_blacklist(self) -> None:
        if not self.blacklist:
            return
        now = self.node.get_clock().now()
        kept = []
        for goal_xy, stamp in self.blacklist:
            age = (now - stamp).nanoseconds / 1e9
            if age <= self.blacklist_timeout_sec:
                kept.append((goal_xy, stamp))
            else:
                self.node.get_logger().info(
                    "Blacklist expired: goal=(%.2f, %.2f)" % (goal_xy[0], goal_xy[1])
                )
                self.retry_counts.pop(self.goal_key(goal_xy), None)
        self.blacklist = kept

    def add_blacklist(self, goal_xy: Optional[Point2D]) -> None:
        if goal_xy is None:
            return
        key = self.goal_key(goal_xy)
        self.retry_counts[key] = self.retry_counts.get(key, 0) + 1
        if self.retry_counts[key] < self.max_retries_per_frontier:
            self.node.get_logger().warn(
                "Goal failed retry %d/%d at (%.2f, %.2f); not blacklisting yet"
                % (self.retry_counts[key], self.max_retries_per_frontier, goal_xy[0], goal_xy[1])
            )
            return
        self.blacklist.append((goal_xy, self.node.get_clock().now()))
        self.node.get_logger().warn(
            "Blacklist added: goal=(%.2f, %.2f) radius=%.2f timeout=%.1f"
            % (goal_xy[0], goal_xy[1], self.blacklist_radius_m, self.blacklist_timeout_sec)
        )

    def is_blacklisted(self, goal_xy: Point2D) -> bool:
        for blacklisted_xy, _ in self.blacklist:
            if euclidean_distance(goal_xy, blacklisted_xy) <= self.blacklist_radius_m:
                return True
        return False

    def goal_key(self, goal_xy: Point2D) -> Tuple[int, int]:
        return (round(goal_xy[0] / 0.1), round(goal_xy[1] / 0.1))

    def _goal_response_callback(self, future) -> None:
        self.goal_response_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().error("NavigateToPose send_goal failed: %s" % exc)
            self._mark_goal_failed(add_to_blacklist=True)
            return

        if not goal_handle.accepted:
            self.node.get_logger().warn("NavigateToPose goal rejected")
            self._mark_goal_failed(add_to_blacklist=True)
            return

        self.active_goal_handle = goal_handle
        self.state = self.STATE_NAVIGATING
        self.node.get_logger().info(
            "NavigateToPose goal accepted: state=%s goal=(%.2f, %.2f)"
            % (self.state, self.active_goal_centroid[0], self.active_goal_centroid[1])
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.node.get_logger().error("NavigateToPose result failed: %s" % exc)
            self._mark_goal_failed(add_to_blacklist=True)
            return

        if self.active_goal_centroid is None and self.state == self.STATE_TIMED_OUT:
            self.node.get_logger().info("Ignoring stale NavigateToPose result after timeout")
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.state = self.STATE_SUCCEEDED
            self.last_goal_result = "SUCCEEDED"
            self.last_result_time = self.node.get_clock().now()
            self.node.get_logger().info("NavigateToPose result: SUCCEEDED source=%s" % self.last_goal_source)
            if self.active_goal_centroid is not None:
                self.last_goal_centroid = self.active_goal_centroid
            self._clear_active_goal(keep_state=True)
            return

        self.state = self.STATE_FAILED
        self.last_goal_result = "FAILED"
        self.last_result_time = self.node.get_clock().now()
        self.node.get_logger().warn("NavigateToPose result: FAILED status=%d" % wrapped_result.status)
        self._mark_goal_failed(add_to_blacklist=True)

    def last_result_age_sec(self) -> Optional[float]:
        if self.last_result_time is None:
            return None
        return (self.node.get_clock().now() - self.last_result_time).nanoseconds / 1e9

    def _cancel_done_callback(self, future) -> None:
        try:
            future.result()
            self.node.get_logger().info("NavigateToPose cancel request completed")
        except Exception as exc:
            self.node.get_logger().warn("NavigateToPose cancel result failed: %s" % exc)

    def _mark_goal_failed(self, add_to_blacklist: bool) -> None:
        if add_to_blacklist:
            self.add_blacklist(self.active_goal_centroid)
        self._clear_active_goal(keep_state=True)

    def _clear_active_goal(self, keep_state: bool = False) -> None:
        if self.active_goal_centroid is not None:
            self.last_goal_centroid = self.active_goal_centroid
        self.active_goal_handle = None
        self.active_goal_centroid = None
        self.active_goal_sent_time = None
        self.goal_response_pending = False
        self.selected_goal_candidate = None
        if not keep_state:
            self.state = self.STATE_IDLE

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q
