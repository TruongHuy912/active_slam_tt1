from dataclasses import dataclass, replace
from typing import Callable, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient

from bumperbot_active_slam.models import NavigationCandidate, Point2D
from bumperbot_active_slam.path_safety import PathSafetyResult, validate_path_safety
from bumperbot_active_slam.path_entropy import compute_path_entropy_for_nav_path


@dataclass(frozen=True)
class PlannerValidationConfig:
    use_planner_validation: bool
    planner_action_name: str
    planner_id: str
    planner_validation_timeout_sec: float
    min_valid_path_length_m: float
    max_valid_path_length_m: float
    max_path_cost: int
    reject_path_unknown: bool
    path_check_step_m: float
    path_clearance_radius_m: float
    max_planner_validation_candidates: int
    planner_validation_required_for_navigation: bool
    planner_validation_retry_next_best: bool = True
    planner_validation_max_batches: int = 4
    planner_validation_batch_size: int = 20
    fallback_relax_path_clearance: bool = True
    fallback_path_clearance_radius_m: float = 0.15
    high_cost_escape_validation_mode: bool = True
    high_cost_escape_ignore_start_radius_m: float = 0.35
    high_cost_escape_path_clearance_radius_m: float = 0.10
    high_cost_escape_allow_initial_high_cost: bool = True
    enable_efficient_utility: bool = False
    path_entropy_sample_step_m: float = 0.1
    path_clearance_max_near_cost: int = 70
    path_clearance_lethal_cost: int = 90
    allow_low_inflation_near_path: bool = True
    low_inflation_cost_threshold: int = 40
    normal_path_ignore_start_radius_m: float = 0.25


@dataclass
class PlannerValidationStats:
    candidates_before_planner_validation: int = 0
    planner_validated_count: int = 0
    rejected_by_planner_timeout: int = 0
    rejected_by_no_path: int = 0
    rejected_by_path_cost: int = 0
    rejected_by_path_unknown: int = 0
    rejected_by_path_clearance: int = 0
    rejected_by_path_length: int = 0
    rejected_by_server_unavailable: int = 0
    planner_validation_batch_index: int = 0
    candidates_in_batch: int = 0
    accepted_count: int = 0
    strict_validation_failed: bool = False
    trying_relaxed_path_clearance: bool = False
    relaxed_selected: bool = False
    source_label: str = "local"
    selected_path_length: float = 0.0
    selected_candidate_after_planner_validation: bool = False
    done: bool = False
    skip_reason: str = "none"


@dataclass(frozen=True)
class PlannerValidationResult:
    candidate: NavigationCandidate
    path: Path
    path_length_m: float
    max_cost: Optional[int]


class PlannerValidator:
    def __init__(
        self,
        node,
        config: PlannerValidationConfig,
        reject_callback: Optional[Callable[[NavigationCandidate, str], None]] = None,
    ) -> None:
        self.node = node
        self.config = config
        self.reject_callback = reject_callback
        self.client = ActionClient(node, ComputePathToPose, config.planner_action_name)
        self.pending = False
        self._candidates: List[NavigationCandidate] = []
        self._robot_xy: Optional[Point2D] = None
        self._frame_id = "map"
        self._costmap: Optional[OccupancyGrid] = None
        self._grid: Optional[OccupancyGrid] = None
        self._current_index = -1
        self._current_candidate: Optional[NavigationCandidate] = None
        self._current_goal_handle = None
        self._request_start_time = None
        self._result: Optional[PlannerValidationResult] = None
        self._stats = PlannerValidationStats()
        self._last_log_time = None
        self._request_id = 0
        self._source_label = "local"
        self.rejected_candidates: List[NavigationCandidate] = []

    @property
    def stats(self) -> PlannerValidationStats:
        return self._stats

    @property
    def result(self) -> Optional[PlannerValidationResult]:
        return self._result

    def reset_result(self) -> None:
        self._result = None
        self._stats = PlannerValidationStats()

    def start(
        self,
        candidates: List[NavigationCandidate],
        robot_xy: Point2D,
        frame_id: str,
        costmap: Optional[OccupancyGrid],
        source_label: str = "local",
        grid: Optional[OccupancyGrid] = None,
    ) -> bool:
        if not self.config.use_planner_validation:
            return False
        if self.pending:
            return False
        ordered = sorted(candidates, key=lambda candidate: candidate.score, reverse=True)
        batch_capacity = (
            max(1, self.config.planner_validation_batch_size)
            * max(1, self.config.planner_validation_max_batches)
        )
        max_candidates = batch_capacity if self.config.planner_validation_retry_next_best else max(
            1,
            self.config.max_planner_validation_candidates,
        )
        self._candidates = ordered[:max_candidates]
        self._source_label = source_label
        self._robot_xy = robot_xy
        self._frame_id = frame_id
        self._costmap = costmap
        self._grid = grid
        self._current_index = -1
        self._current_candidate = None
        self._current_goal_handle = None
        self._result = None
        self._stats = PlannerValidationStats(
            candidates_before_planner_validation=len(self._candidates),
            source_label=source_label,
        )
        self.rejected_candidates = []
        if not self._candidates:
            self._stats.done = True
            self._stats.skip_reason = "no candidates before planner validation"
            return False
        if not self.client.server_is_ready():
            self._stats.rejected_by_server_unavailable = len(self._candidates)
            self._stats.done = True
            self._stats.skip_reason = (
                "planner action server %s not available" % self.config.planner_action_name
            )
            return False
        self.pending = True
        self.node.get_logger().info(
            "Planner validation started: source=%s action=%s candidates_before_planner_validation=%d "
            "batch_size=%d max_batches=%d"
            % (
                source_label,
                self.config.planner_action_name,
                len(self._candidates),
                self.config.planner_validation_batch_size,
                self.config.planner_validation_max_batches,
            )
        )
        self._start_next_candidate()
        return True

    def check_timeout(self) -> None:
        if not self.pending or self._request_start_time is None:
            return
        elapsed = (self.node.get_clock().now() - self._request_start_time).nanoseconds / 1e9
        if elapsed < self.config.planner_validation_timeout_sec:
            return
        self._stats.rejected_by_planner_timeout += 1
        self.node.get_logger().warn(
            "Planner validation timeout after %.2f s for candidate %d/%d"
            % (elapsed, self._current_index + 1, len(self._candidates))
        )
        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.node.get_logger().warn("ComputePathToPose cancel failed: %s" % exc)
        self._start_next_candidate()

    def take_result(self) -> Optional[PlannerValidationResult]:
        result = self._result
        self._result = None
        return result

    def _start_next_candidate(self) -> None:
        self._current_index += 1
        self._current_goal_handle = None
        self._request_start_time = None
        if self._current_index >= len(self._candidates):
            self.pending = False
            self._stats.done = True
            self._stats.skip_reason = "no planner-validated candidate"
            self.node.get_logger().info(
                "Planner validation finished: source=%s validated=%d rejected_timeout=%d "
                "rejected_no_path=%d rejected_cost=%d rejected_unknown=%d "
                "rejected_clearance=%d selected=false"
                % (
                    self._source_label,
                    self._stats.planner_validated_count,
                    self._stats.rejected_by_planner_timeout,
                    self._stats.rejected_by_no_path,
                    self._stats.rejected_by_path_cost,
                    self._stats.rejected_by_path_unknown,
                    self._stats.rejected_by_path_clearance,
                )
            )
            return

        self._current_candidate = self._candidates[self._current_index]
        batch_index = int(self._current_index / max(1, self.config.planner_validation_batch_size)) + 1
        if batch_index != self._stats.planner_validation_batch_index:
            self._stats.planner_validation_batch_index = batch_index
            batch_start = (batch_index - 1) * max(1, self.config.planner_validation_batch_size)
            batch_end = min(len(self._candidates), batch_start + max(1, self.config.planner_validation_batch_size))
            self._stats.candidates_in_batch = batch_end - batch_start
            self.node.get_logger().info(
                "Planner validation batch: source=%s planner_validation_batch_index=%d candidates_in_batch=%d"
                % (self._source_label, batch_index, self._stats.candidates_in_batch)
            )
        self._request_id += 1
        request_id = self._request_id
        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.planner_id = self.config.planner_id
        goal_msg.start = self._pose_stamped(self._robot_xy)
        goal_msg.goal = self._pose_stamped(self._current_candidate.point_world)

        self._request_start_time = self.node.get_clock().now()
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future: self._goal_response_callback(future, request_id)
        )

    def _goal_response_callback(self, future, request_id: int) -> None:
        if request_id != self._request_id:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().warn("ComputePathToPose send_goal failed: %s" % exc)
            self._stats.rejected_by_no_path += 1
            self._start_next_candidate()
            return

        if not goal_handle.accepted:
            self._stats.rejected_by_no_path += 1
            self._start_next_candidate()
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self._path_result_callback(future, request_id))

    def _path_result_callback(self, future, request_id: int) -> None:
        if request_id != self._request_id:
            return
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.node.get_logger().warn("ComputePathToPose result failed: %s" % exc)
            self._stats.rejected_by_no_path += 1
            self._start_next_candidate()
            return

        candidate = self._current_candidate
        safety = validate_path_safety(
            wrapped_result.result.path,
            self._costmap,
            self.config.min_valid_path_length_m,
            self.config.max_valid_path_length_m,
            self.config.max_path_cost,
            self.config.reject_path_unknown,
            self.config.path_check_step_m,
            self._clearance_radius_for(candidate),
            self._ignore_start_radius_for(candidate),
            self.config.path_clearance_max_near_cost,
            self.config.path_clearance_lethal_cost,
            self.config.allow_low_inflation_near_path,
            self.config.low_inflation_cost_threshold,
        )
        self._stats.planner_validated_count += 1
        used_relaxed = False
        if (
            not safety.safe
            and safety.reject_reason in ("path_clearance", "start_clearance_reject")
            and self.config.fallback_relax_path_clearance
        ):
            self._stats.strict_validation_failed = True
            self._stats.trying_relaxed_path_clearance = True
            relaxed = validate_path_safety(
                wrapped_result.result.path,
                self._costmap,
                self.config.min_valid_path_length_m,
                self.config.max_valid_path_length_m,
                self.config.max_path_cost,
                self.config.reject_path_unknown,
                self.config.path_check_step_m,
                self.config.fallback_path_clearance_radius_m,
                self._ignore_start_radius_for(candidate),
                self.config.path_clearance_max_near_cost,
                self.config.path_clearance_lethal_cost,
                self.config.allow_low_inflation_near_path,
                self.config.low_inflation_cost_threshold,
            )
            if relaxed.safe:
                safety = relaxed
                used_relaxed = True
        if not safety.safe:
            self._record_rejection(safety)
            self._log_rejection(candidate, safety)
            if self.reject_callback is not None and safety.reject_reason in (
                "path_clearance",
                "start_clearance_reject",
                "path_cost",
                "path_unknown",
                "no_path",
                "path_outside_costmap",
            ):
                self.reject_callback(candidate, safety.reject_reason)
            self.rejected_candidates.append(candidate)
            self._start_next_candidate()
            return

        accepted_candidate = candidate
        path_entropy = None
        if self.config.enable_efficient_utility and self._grid is not None:
            path_entropy = compute_path_entropy_for_nav_path(
                self._grid,
                wrapped_result.result.path,
                self.config.path_entropy_sample_step_m,
            )
            accepted_candidate = replace(
                candidate,
                path_entropy=path_entropy.sum_path_entropy,
                mean_path_entropy=path_entropy.mean_path_entropy,
                unknown_ratio_along_path=path_entropy.unknown_ratio_along_path,
            )
        self._result = PlannerValidationResult(
            candidate=accepted_candidate,
            path=wrapped_result.result.path,
            path_length_m=safety.length_m,
            max_cost=safety.max_cost,
        )
        self._stats.selected_path_length = safety.length_m
        self._stats.selected_candidate_after_planner_validation = True
        self._stats.accepted_count += 1
        self._stats.relaxed_selected = used_relaxed
        self._stats.done = True
        self.pending = False
        self.node.get_logger().info(
            "Planner validation accepted: source=%s candidate=(%.2f, %.2f) path_length=%.2f "
            "max_cost=%s validated_count=%d relaxed_selected=%s safety_radius_used=%.2f "
            "post_planner_path_entropy=%.3f unknown_ratio_along_path=%.3f"
            % (
                self._source_label,
                candidate.point_world[0],
                candidate.point_world[1],
                safety.length_m,
                "unknown" if safety.max_cost is None else str(safety.max_cost),
                self._stats.planner_validated_count,
                used_relaxed,
                self.config.fallback_path_clearance_radius_m
                if used_relaxed
                else self._clearance_radius_for(candidate),
                0.0 if path_entropy is None else path_entropy.sum_path_entropy,
                0.0 if path_entropy is None else path_entropy.unknown_ratio_along_path,
            )
        )

    def _record_rejection(self, safety: PathSafetyResult) -> None:
        reason = safety.reject_reason
        if reason in ("no_path", "path_outside_costmap"):
            self._stats.rejected_by_no_path += 1
        elif reason == "path_unknown":
            self._stats.rejected_by_path_unknown += 1
        elif reason in ("path_clearance", "start_clearance_reject"):
            self._stats.rejected_by_path_clearance += 1
        elif reason in ("path_too_short", "path_too_long"):
            self._stats.rejected_by_path_length += 1
        else:
            self._stats.rejected_by_path_cost += 1

    def _log_rejection(self, candidate: NavigationCandidate, safety: PathSafetyResult) -> None:
        if len(self.rejected_candidates) >= 3:
            return
        width = int(self._costmap.info.width) if self._costmap is not None else 0
        height = int(self._costmap.info.height) if self._costmap is not None else 0
        resolution = float(self._costmap.info.resolution) if self._costmap is not None else 0.0
        frame = self._costmap.header.frame_id if self._costmap is not None else "unknown"
        self.node.get_logger().warn(
            "Planner candidate rejected: source=%s cluster_id=%d candidate=(%.2f, %.2f) "
            "path_clearance_radius_m=%.2f fallback_path_clearance_radius_m=%.2f "
            "path_length=%.2f samples_checked=%d first_reject_world=%s "
            "first_reject_map_cell=%s reason=%s detail=%s max_cost_on_path=%s "
            "max_cost_near_path=%s ignore_start_radius_m=%.2f ignored_start_samples=%d "
            "first_checked_pose_after_start=%s robot_cost=%s goal_cost=%s "
            "costmap=%s %.3fm %dx%d path_frame=%s"
            % (
                self._source_label,
                candidate.cluster_id,
                candidate.point_world[0],
                candidate.point_world[1],
                self._clearance_radius_for(candidate),
                self.config.fallback_path_clearance_radius_m,
                safety.length_m,
                safety.samples_checked,
                _point_text(safety.first_reject_world),
                str(safety.first_reject_map_cell),
                safety.reject_reason,
                safety.detail,
                "unknown" if safety.max_cost is None else str(safety.max_cost),
                "unknown" if safety.max_cost_near_path is None else str(safety.max_cost_near_path),
                safety.ignore_start_radius_m,
                safety.ignored_start_samples,
                _point_text(safety.first_checked_world),
                "unknown" if safety.robot_cost is None else str(safety.robot_cost),
                "unknown" if safety.goal_cost is None else str(safety.goal_cost),
                frame,
                resolution,
                width,
                height,
                self._frame_id,
            )
        )

    def _ignore_start_radius_for(self, candidate: Optional[NavigationCandidate]) -> float:
        if (
            candidate is not None
            and candidate.source == "high_cost_escape"
            and self.config.high_cost_escape_validation_mode
            and self.config.high_cost_escape_allow_initial_high_cost
        ):
            return self.config.high_cost_escape_ignore_start_radius_m
        return self.config.normal_path_ignore_start_radius_m

    def _clearance_radius_for(self, candidate: Optional[NavigationCandidate]) -> float:
        if (
            candidate is not None
            and candidate.source == "high_cost_escape"
            and self.config.high_cost_escape_validation_mode
        ):
            return self.config.high_cost_escape_path_clearance_radius_m
        return self.config.path_clearance_radius_m

    def _pose_stamped(self, point_xy: Point2D) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(point_xy[0])
        pose.pose.position.y = float(point_xy[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose


def _point_text(point: Optional[Point2D]) -> str:
    if point is None:
        return "none"
    return "(%.2f, %.2f)" % (point[0], point[1])
