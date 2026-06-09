import math
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from bumperbot_active_slam.costmap_utils import max_cost_in_radius, summarize_costmap_status
from bumperbot_active_slam.entropy_utils import euclidean_distance
from bumperbot_active_slam.frontier_detector import FrontierCluster, detect_frontiers_with_stats
from bumperbot_active_slam.goal_selector import GoalSelector
from bumperbot_active_slam.high_cost_escape import HighCostEscapePolicy
from bumperbot_active_slam.marker_utils import build_active_slam_markers, build_clear_markers
from bumperbot_active_slam.models import (
    CandidateSelectionStats,
    NavigationCandidate,
)
from bumperbot_active_slam.navigation_dispatcher import NavigationDispatcher
from bumperbot_active_slam.node_params import (
    declare_active_slam_parameters,
    make_goal_selector_config,
    make_high_cost_escape_config,
    make_planner_validation_config,
    read_active_slam_parameters,
)
from bumperbot_active_slam.candidate_diversity import diverse_candidates
from bumperbot_active_slam.planner_reject_cache import PlannerRejectCache
from bumperbot_active_slam.planner_validator import PlannerValidator
from bumperbot_active_slam.progress_monitor import ProgressMonitor


class ActiveSlamExplorer(Node):
    """ROS orchestration for frontier detection, debug markers, and optional Nav2 goals."""

    def __init__(self) -> None:
        super().__init__("active_slam_explorer")

        self._declare_parameters()
        self._read_parameters()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self._costmap_callback,
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        selector_config = make_goal_selector_config(self)
        self.navigation = NavigationDispatcher(
            self,
            self.navigate_action_name,
            self.goal_timeout_sec,
            self.blacklist_radius_m,
            self.blacklist_timeout_sec,
            self.max_retries_per_frontier,
        )
        self.stale_frontier_records = {}
        self.stale_frontier_region_cooldowns = {}
        self.stale_frontier_cluster_cooldowns = {}
        self.no_safe_viewpoint_key = None
        self.no_safe_viewpoint_cycles = 0
        self.final_no_safe_viewpoint_stop_active = False
        self.consecutive_high_cost_escape_failures = 0
        self.high_cost_failure_stop_active = False
        self.planner_reject_cache = PlannerRejectCache(
            self,
            self.planner_reject_blacklist_radius_m,
            self.planner_reject_blacklist_timeout_sec,
            self.planner_reject_cluster_fail_threshold,
            self.planner_reject_cluster_timeout_sec,
            self.no_path_blacklist_radius_m,
            self.no_path_blacklist_timeout_sec,
            self.no_path_cluster_fail_threshold,
            self.no_path_cluster_timeout_sec,
            self.planner_reject_reasons_to_cache,
            self.log_individual_blacklist_hits,
            self.max_blacklist_hit_logs_per_cycle,
        )
        self.goal_selector = GoalSelector(
            selector_config,
            self.get_logger(),
            self._is_navigation_or_stale_blacklisted,
            self._is_planner_or_stale_rejected,
        )
        self.planner_validator = PlannerValidator(
            self,
            make_planner_validation_config(self),
            self._on_planner_candidate_rejected,
        )
        self.high_cost_escape = HighCostEscapePolicy(
            make_high_cost_escape_config(self),
            self.get_logger(),
        )

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_costmap: Optional[OccupancyGrid] = None
        self.last_validated_path = None
        self.last_costmap_dimensions: Optional[Tuple[int, int, float, str]] = None
        self.last_map_dimensions: Optional[Tuple[int, int, float, str]] = None
        self.last_log_time = self.get_clock().now()
        self.last_tf_warning_time = self.get_clock().now()
        self.last_nav_log_time = self.get_clock().now()
        self.last_goal_selection_log_time = None
        self.last_goal_attempt_time = None if self.send_goal_on_startup else self.get_clock().now()
        self.progress_monitor = ProgressMonitor(
            self,
            self.progress_check_period_sec,
            self.min_progress_distance_m,
            self.stuck_timeout_sec,
        )

        self.timer = self.create_timer(self.update_period_sec, self._update)

        self.get_logger().info(
            "Active SLAM started: map_topic=%s, global_frame=%s, robot_frame=%s, "
            "marker_topic=%s, map_qos=RELIABLE+TRANSIENT_LOCAL"
            % (self.map_topic, self.global_frame, self.robot_frame, self.marker_topic)
        )
        self.get_logger().info(
            "Navigation dispatch: enable_navigation=%s, action_name=%s, state=%s"
            % (self.enable_navigation, self.navigate_action_name, self.navigation.state)
        )
        if self.enable_efficient_utility and self.scoring_mode == "efficient_entropy_utility":
            self.get_logger().info(
                "Phase5 utility: enable_efficient_utility=True scoring_mode=efficient_entropy_utility"
            )
        else:
            self.get_logger().info("Phase5 utility disabled; using safe_viewpoint baseline.")
        self.get_logger().info(
            "Goal selector: mode=%s efficient_utility=%s global_reposition=%s costmap_filter=%s costmap_topic=%s"
            % (
                self.scoring_mode,
                self.enable_efficient_utility,
                self.enable_global_reposition,
                self.use_costmap_filter,
                self.costmap_topic,
            )
        )

    def _declare_parameters(self) -> None:
        declare_active_slam_parameters(self)

    def _read_parameters(self) -> None:
        read_active_slam_parameters(self)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        dimensions = (
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            msg.header.frame_id,
        )
        if dimensions != self.last_map_dimensions:
            self.last_map_dimensions = dimensions
            self.get_logger().info(
                "Received map: frame=%s size=%dx%d resolution=%.3f origin=(%.2f, %.2f)"
                % (
                    msg.header.frame_id,
                    msg.info.width,
                    msg.info.height,
                    msg.info.resolution,
                    msg.info.origin.position.x,
                    msg.info.origin.position.y,
                )
            )
            if msg.header.frame_id and msg.header.frame_id != self.global_frame:
                self.get_logger().warn(
                    "Map frame '%s' differs from configured global_frame '%s'"
                    % (msg.header.frame_id, self.global_frame)
                )

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self.latest_costmap = msg
        dimensions = (
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            msg.header.frame_id,
        )
        if dimensions != self.last_costmap_dimensions:
            self.last_costmap_dimensions = dimensions
            self.get_logger().info("Received costmap: %s" % summarize_costmap_status(msg))

    def _update(self) -> None:
        self.navigation.expire_blacklist()
        self.planner_reject_cache.expire()
        self.planner_reject_cache.begin_cycle()
        self.navigation.check_active_goal_timeout()
        self.planner_validator.check_timeout()

        if self.latest_map is None:
            self._warn_periodic("Waiting for OccupancyGrid on %s" % self.map_topic)
            self._publish_clear_markers()
            return

        robot_xy = self._lookup_robot_xy()
        if not self._is_valid_grid(self.latest_map):
            self._warn_periodic(
                "Received empty map %dx%d from %s; waiting for SLAM Toolbox to publish "
                "a valid map. Check /scan, /clock, and SLAM launch."
                % (self.latest_map.info.width, self.latest_map.info.height, self.map_topic)
            )
            self._publish_clear_markers()
            self._log_empty_map_summary(robot_xy)
            return

        detection = detect_frontiers_with_stats(
            self.latest_map,
            connectivity=self.frontier_connectivity,
            min_cluster_size=self.min_cluster_size,
        )
        clusters = detection.clusters
        best = self._select_best_candidate(clusters, robot_xy)
        self._update_navigation(clusters, robot_xy, detection.frontier_cell_count)

        if self.enable_debug_markers:
            self.marker_pub.publish(
                build_active_slam_markers(
                    self._marker_frame(),
                    self.get_clock(),
                    self.latest_map,
                    clusters,
                    best,
                    self.navigation.selected_goal_candidate,
                    self.navigation.active_goal_centroid,
                    self.goal_selector.last_valid_candidates[: self.max_candidate_markers],
                    self.goal_selector.last_rejected_candidates[: self.max_rejected_markers],
                    self.navigation.blacklist,
                    self.last_validated_path,
                    self.planner_validator.rejected_candidates[: self.max_planner_rejected_markers],
                    self.planner_reject_cache.entries,
                    self.max_frontier_markers,
                    self.max_candidate_markers,
                    self.max_rejected_markers,
                    self.blacklist_radius_m,
                )
            )

        self._log_frontier_summary(detection.frontier_cell_count, clusters, best, robot_xy)

    def _update_navigation(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Optional[Tuple[float, float]],
        frontier_cell_count: int,
    ) -> None:
        if not self.enable_navigation:
            self._log_navigation_skip("enable_navigation=false")
            return
        self._expire_stale_frontier_cooldowns()

        if self.navigation.state == NavigationDispatcher.STATE_NAVIGATING:
            if self.progress_monitor.update(robot_xy):
                self.get_logger().warn("stuck_detected: canceling current goal")
                if self.cancel_goal_on_stuck:
                    self.navigation.cancel_active_goal(
                        self.blacklist_goal_on_stuck,
                        "stuck_detected",
                    )
                return
            if self.navigation.active_goal_centroid is not None and robot_xy is not None:
                distance = euclidean_distance(self.navigation.active_goal_centroid, robot_xy)
                if distance <= self.goal_reached_distance_m:
                    self._log_navigation_skip(
                        "active goal is within goal_reached_distance_m; waiting for Nav2 result"
                    )
                    return
            self._log_navigation_skip("currently navigating")
            return

        if self.navigation.goal_response_pending:
            self._log_navigation_skip("goal request pending")
            return
        if robot_xy is None:
            self._log_navigation_skip("robot TF unavailable")
            return
        if self.final_no_safe_viewpoint_stop_active:
            self._log_navigation_skip("exploration_complete_or_unreachable_frontiers")
            return
        if self._high_cost_failure_stop_should_pause(robot_xy):
            self._log_navigation_skip("high_cost_failure_stop active")
            self.last_goal_attempt_time = self.get_clock().now()
            return
        if not clusters:
            self._log_navigation_skip("no frontier clusters")
            self._log_goal_selection(CandidateSelectionStats(), None, "no frontier clusters")
            return
        validation_result = self.planner_validator.take_result()
        if validation_result is not None:
            self.last_validated_path = validation_result.path
            self._log_planner_validation_stats()
            if not self.navigation.server_is_ready():
                self.navigation.state = NavigationDispatcher.STATE_WAITING_FOR_SERVER
                self._log_navigation_skip(
                    "NavigateToPose action server %s not available" % self.navigate_action_name
                )
                return
            self.last_goal_attempt_time = self.get_clock().now()
            self.progress_monitor.reset(robot_xy)
            self.navigation.send_goal(validation_result.candidate, robot_xy, self._marker_frame())
            return
        if self.planner_validator.pending:
            self._log_navigation_skip("planner validation pending")
            return
        if self.planner_validator.stats.done and not self.planner_validator.stats.selected_candidate_after_planner_validation:
            if (
                self.planner_validator.stats.source_label == "local"
                and self.enable_medium_reposition
            ):
                self.get_logger().info("local planner validation failed, trying medium_reposition")
                if self._start_medium_reposition_validation(robot_xy):
                    self.last_goal_attempt_time = self.get_clock().now()
                    return
            if (
                self.planner_validator.stats.source_label in ("local", "medium_reposition")
                and self.enable_global_reposition_after_planner_fail
            ):
                self.get_logger().info("local planner validation failed, trying global_reposition")
                if self._start_global_reposition_validation(clusters, robot_xy):
                    self.last_goal_attempt_time = self.get_clock().now()
                    return
            self._log_navigation_skip("planner validation failed; no planner-valid candidate; waiting before retry")
            self.planner_reject_cache.log_cycle_summary()
            self.planner_validator.reset_result()
            self.last_goal_attempt_time = self.get_clock().now()
            return
        if not self._goal_update_period_elapsed():
            self._log_navigation_skip("waiting for goal_update_period_sec")
            return

        selected, stats = self.goal_selector.select(
            clusters,
            robot_xy,
            self.latest_map,
            self.latest_costmap,
            self.navigation.last_goal_centroid,
            self.navigation.last_goal_robot_xy,
            self.navigation.state,
            self.navigation.last_goal_source,
            self.navigation.last_goal_result,
            self.navigation.last_result_age_sec(),
            frontier_cell_count,
        )
        selected = self._maybe_high_cost_escape_candidate(selected, robot_xy)
        if self.high_cost_escape.last_active and selected is None:
            self._record_high_cost_escape_failure("no lower-cost candidate")
            self._log_navigation_skip(self.high_cost_escape.last_skip_reason)
            self.planner_reject_cache.log_cycle_summary()
            self.last_goal_attempt_time = self.get_clock().now()
            return
        skip_reason = stats.skip_reason
        if selected is None:
            if skip_reason == "none":
                if stats.rejected_by_progress > 0:
                    skip_reason = "robot has not progressed enough since previous goal"
                elif stats.sampled_viewpoints > 0 and stats.rejected_by_distance == stats.sampled_viewpoints:
                    skip_reason = "all viewpoints rejected by distance limits"
                else:
                    skip_reason = "no safe viewpoint candidate"
            self._log_navigation_skip(skip_reason)
            self._log_goal_selection(stats, None, skip_reason)
            self._handle_no_safe_viewpoint_cycle(
                clusters,
                robot_xy,
                frontier_cell_count,
                stats,
                skip_reason,
            )
            self.planner_reject_cache.log_cycle_summary()
            self.last_goal_attempt_time = self.get_clock().now()
            return

        self.no_safe_viewpoint_key = None
        self.no_safe_viewpoint_cycles = 0
        self._log_goal_selection(stats, selected, "none")
        if self._stale_frontier_should_skip(selected, frontier_cell_count):
            self._log_navigation_skip("stale frontier suppression active")
            self.planner_reject_cache.log_cycle_summary()
            self.last_goal_attempt_time = self.get_clock().now()
            return
        if self.navigation.is_blacklisted(selected.point_world):
            self._log_navigation_skip("selected candidate is blacklisted")
            self.last_goal_attempt_time = self.get_clock().now()
            return
        if not self.navigation.server_is_ready():
            self.navigation.state = NavigationDispatcher.STATE_WAITING_FOR_SERVER
            self._log_navigation_skip(
                "NavigateToPose action server %s not available" % self.navigate_action_name
            )
            return

        selected = self._planner_validate_before_dispatch(selected, robot_xy)
        if selected is None:
            return

        self.last_goal_attempt_time = self.get_clock().now()
        self.progress_monitor.reset(robot_xy)
        self.navigation.send_goal(selected, robot_xy, self._marker_frame())

    def _start_medium_reposition_validation(self, robot_xy: Tuple[float, float]) -> bool:
        selected, stats = self.goal_selector.select_medium_reposition(
            robot_xy,
            self.latest_map,
            self.latest_costmap,
            self.medium_reposition_min_distance_m,
            self.medium_reposition_max_distance_m,
            self.medium_reposition_sample_count,
        )
        if selected is None:
            self._log_goal_selection(stats, None, "medium reposition produced no candidate")
            return False
        self._log_goal_selection(stats, selected, "planner fallback medium_reposition")
        started = self.planner_validator.start(
            self._planner_validation_candidates(selected),
            robot_xy,
            self._marker_frame(),
            self.latest_costmap,
            source_label="medium_reposition",
            grid=self.latest_map,
        )
        if started:
            self._log_navigation_skip("local planner validation failed, trying medium_reposition")
        return started

    def _maybe_high_cost_escape_candidate(
        self,
        selected: Optional[NavigationCandidate],
        robot_xy: Tuple[float, float],
    ) -> Optional[NavigationCandidate]:
        escape = self.high_cost_escape.select(robot_xy, self.latest_map, self.latest_costmap)
        if not self.high_cost_escape.last_active:
            self.consecutive_high_cost_escape_failures = 0
            return selected
        if self.enable_efficient_utility and self.scoring_mode == "efficient_entropy_utility":
            self.get_logger().info("Efficient utility skipped: high_cost_escape priority")
        if self.recovery_clear_reject_cache_when_robot_high_cost:
            self.planner_reject_cache.clear()
        self.goal_selector.last_valid_candidates = self.high_cost_escape.last_candidates[:]
        self.goal_selector.last_rejected_candidates = self.high_cost_escape.last_rejected[:]
        if escape is not None:
            self.consecutive_high_cost_escape_failures = 0
        return escape

    def _is_navigation_or_stale_blacklisted(self, point_xy: Tuple[float, float]) -> bool:
        return self.navigation.is_blacklisted(point_xy) or self._stale_frontier_region_on_cooldown(point_xy)

    def _is_planner_or_stale_rejected(self, point_xy: Tuple[float, float], cluster_id: int) -> bool:
        return (
            self.planner_reject_cache.is_rejected(point_xy, cluster_id)
            or self._stale_frontier_region_on_cooldown(point_xy)
            or self._stale_frontier_cluster_on_cooldown(cluster_id)
        )

    def _frontier_region_bucket(self, point_xy: Tuple[float, float]) -> Tuple[int, int]:
        size = max(0.1, self.stale_frontier_region_size_m)
        return (int(math.floor(point_xy[0] / size)), int(math.floor(point_xy[1] / size)))

    def _expire_stale_frontier_cooldowns(self) -> None:
        if not self.stale_frontier_region_cooldowns:
            region_cooldowns = {}
        else:
            region_cooldowns = self.stale_frontier_region_cooldowns
        now = self.get_clock().now()
        kept = {}
        for region, expiry in region_cooldowns.items():
            if (expiry - now).nanoseconds > 0:
                kept[region] = expiry
            else:
                self.get_logger().info(
                    "stale_frontier_cooldown_expired: selected_region_bucket=%s" % (region,)
                )
        self.stale_frontier_region_cooldowns = kept
        kept_clusters = {}
        for cluster_id, expiry in self.stale_frontier_cluster_cooldowns.items():
            if (expiry - now).nanoseconds > 0:
                kept_clusters[cluster_id] = expiry
            else:
                self.get_logger().info(
                    "stale_frontier_cluster_cooldown_expired: selected_cluster_id=%d" % cluster_id
                )
        self.stale_frontier_cluster_cooldowns = kept_clusters

    def _stale_frontier_region_on_cooldown(self, point_xy: Tuple[float, float]) -> bool:
        if not self.enable_stale_frontier_suppression and not self.enable_unreachable_frontier_cooldown:
            return False
        self._expire_stale_frontier_cooldowns()
        return self._frontier_region_bucket(point_xy) in self.stale_frontier_region_cooldowns

    def _stale_frontier_cluster_on_cooldown(self, cluster_id: int) -> bool:
        if not self.enable_stale_frontier_suppression and not self.enable_unreachable_frontier_cooldown:
            return False
        self._expire_stale_frontier_cooldowns()
        return cluster_id in self.stale_frontier_cluster_cooldowns

    def _handle_no_safe_viewpoint_cycle(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Tuple[float, float],
        frontier_cell_count: int,
        stats: CandidateSelectionStats,
        skip_reason: str,
    ) -> None:
        if skip_reason != "no safe viewpoint candidate":
            self.no_safe_viewpoint_key = None
            self.no_safe_viewpoint_cycles = 0
            return
        if stats.local_candidates != 0 or stats.global_candidates != 0:
            return

        best = self._select_best_candidate(clusters, robot_xy)
        if best is None:
            return
        best_distance = euclidean_distance(best.centroid_world, robot_xy)
        robot_cost = (
            max_cost_in_radius(robot_xy[0], robot_xy[1], self.latest_costmap, 0.20)
            if self.latest_costmap is not None
            else None
        )
        near_final_state = (
            frontier_cell_count >= self.near_frontier_min_frontier_cells
            and best.size >= self.near_frontier_min_best_cluster_size
            and best_distance <= self.near_frontier_max_best_distance_m
            and robot_cost is not None
            and robot_cost < self.high_cost_robot_threshold
        )
        if not near_final_state:
            self.no_safe_viewpoint_key = None
            self.no_safe_viewpoint_cycles = 0
            return

        region = self._frontier_region_bucket(best.centroid_world)
        key = (best.id, region)
        if key == self.no_safe_viewpoint_key:
            self.no_safe_viewpoint_cycles += 1
        else:
            self.no_safe_viewpoint_key = key
            self.no_safe_viewpoint_cycles = 1

        self.get_logger().info(
            "near_frontier_no_safe_viewpoint_status: selected_cluster_id=%d "
            "selected_region_bucket=%s no_safe_cycles=%d frontier_cells=%d "
            "best_size=%d best_distance=%.2f local_candidates=%d global_candidates=%d "
            "rejected_by_clearance=%d rejected_by_distance=%d robot_cost=%s"
            % (
                best.id,
                region,
                self.no_safe_viewpoint_cycles,
                frontier_cell_count,
                best.size,
                best_distance,
                stats.local_candidates,
                stats.global_candidates,
                stats.rejected_by_clearance,
                stats.rejected_by_distance,
                "unknown" if robot_cost is None else str(robot_cost),
            )
        )

        if (
            self.enable_unreachable_frontier_cooldown
            and self.no_safe_viewpoint_cycles >= self.unreachable_frontier_no_safe_cycles
        ):
            now = self.get_clock().now()
            self.stale_frontier_cluster_cooldowns[best.id] = now + Duration(
                seconds=self.unreachable_frontier_cooldown_sec
            )
            self.stale_frontier_region_cooldowns[region] = now + Duration(
                seconds=self.unreachable_frontier_cooldown_sec
            )
            self.get_logger().warn(
                "unreachable_frontier_cooldown_applied: selected_cluster_id=%d "
                "selected_region_bucket=%s no_safe_cycles=%d cooldown_sec=%.1f "
                "reason=no_safe_viewpoint_after_retries"
                % (
                    best.id,
                    region,
                    self.no_safe_viewpoint_cycles,
                    self.unreachable_frontier_cooldown_sec,
                )
            )

        if (
            self.enable_final_no_safe_viewpoint_stop
            and self.no_safe_viewpoint_cycles >= self.final_no_safe_viewpoint_cycles
        ):
            self.final_no_safe_viewpoint_stop_active = True
            self.get_logger().error(
                "final_no_safe_viewpoint_stop: no_safe_cycles=%d threshold=%d "
                "frontier_cells=%d best_id=%d best_size=%d best_distance=%.2f "
                "robot_cost=%s"
                % (
                    self.no_safe_viewpoint_cycles,
                    self.final_no_safe_viewpoint_cycles,
                    frontier_cell_count,
                    best.id,
                    best.size,
                    best_distance,
                    "unknown" if robot_cost is None else str(robot_cost),
                )
            )
            self.get_logger().error(
                "exploration_complete_or_unreachable_frontiers: "
                "reason=no_safe_viewpoint_after_retries"
            )

    def _stale_frontier_should_skip(
        self,
        selected: NavigationCandidate,
        frontier_cell_count: int,
    ) -> bool:
        if not self.enable_stale_frontier_suppression or selected.source != "frontier_cell":
            return False

        region = self._frontier_region_bucket(selected.point_world)
        keys = (("cluster", selected.cluster_id), ("region", region))
        max_repeat = 1
        applied = False
        reasons = []
        now = self.get_clock().now()

        for key in keys:
            previous = self.stale_frontier_records.get(key)
            if previous is None:
                repeat_count = 1
                reason = "first_seen"
            else:
                frontier_reduction = previous["frontier_cells"] - frontier_cell_count
                cluster_reduction = previous["cluster_size"] - selected.cluster_size
                distance_change = abs(previous["distance"] - selected.distance)
                stale = (
                    frontier_reduction < self.stale_frontier_min_frontier_reduction
                    and (
                        cluster_reduction < self.stale_frontier_min_frontier_reduction
                        or distance_change < self.stale_frontier_min_distance_change_m
                    )
                )
                repeat_count = previous["repeat_count"] + 1 if stale else 1
                reason = (
                    "stale_progress frontier_reduction=%d cluster_reduction=%d distance_change=%.2f"
                    % (frontier_reduction, cluster_reduction, distance_change)
                    if stale
                    else "progress_observed"
                )

            self.stale_frontier_records[key] = {
                "repeat_count": repeat_count,
                "frontier_cells": frontier_cell_count,
                "cluster_size": selected.cluster_size,
                "distance": selected.distance,
            }
            max_repeat = max(max_repeat, repeat_count)
            reasons.append("%s:%s" % (key[0], reason))
            if repeat_count >= self.stale_frontier_max_repeats:
                self.stale_frontier_cluster_cooldowns[selected.cluster_id] = now + Duration(
                    seconds=self.stale_frontier_cooldown_sec
                )
                self.stale_frontier_region_cooldowns[region] = now + Duration(
                    seconds=self.stale_frontier_cooldown_sec
                )
                applied = True

        self.get_logger().warn(
            "stale_frontier_status: selected_cluster_id=%d selected_region_bucket=%s "
            "repeat_count=%d cooldown_applied=%s reason=%s"
            % (
                selected.cluster_id,
                region,
                max_repeat,
                applied,
                ";".join(reasons),
            )
        )
        return applied

    def _record_high_cost_escape_failure(self, reason: str) -> None:
        if not self.enable_high_cost_failure_stop:
            return
        self.consecutive_high_cost_escape_failures += 1
        if self.consecutive_high_cost_escape_failures < self.max_consecutive_high_cost_escape_failures:
            return
        self.high_cost_failure_stop_active = True
        self.get_logger().error(
            "high_cost_failure_stop: consecutive_failures=%d threshold=%d reason=%s"
            % (
                self.consecutive_high_cost_escape_failures,
                self.max_consecutive_high_cost_escape_failures,
                reason,
            )
        )

    def _high_cost_failure_stop_should_pause(self, robot_xy: Tuple[float, float]) -> bool:
        if not self.enable_high_cost_failure_stop or not self.high_cost_failure_stop_active:
            return False
        robot_cost = (
            max_cost_in_radius(robot_xy[0], robot_xy[1], self.latest_costmap, 0.20)
            if self.latest_costmap is not None
            else None
        )
        if robot_cost is not None and robot_cost < self.high_cost_robot_threshold:
            self.get_logger().info(
                "high_cost_failure_stop_reset: global_max_cost_near_robot=%d threshold=%d"
                % (robot_cost, self.high_cost_robot_threshold)
            )
            self.high_cost_failure_stop_active = False
            self.consecutive_high_cost_escape_failures = 0
            return False
        self.get_logger().error(
            "high_cost_failure_stop: pause_exploration=True consecutive_failures=%d "
            "global_max_cost_near_robot=%s threshold=%d"
            % (
                self.consecutive_high_cost_escape_failures,
                "unknown" if robot_cost is None else str(robot_cost),
                self.high_cost_robot_threshold,
            )
        )
        return True

    def _start_global_reposition_validation(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Tuple[float, float],
    ) -> bool:
        selected, stats = self.goal_selector.select_global_reposition(
            clusters,
            robot_xy,
            self.latest_map,
            self.latest_costmap,
            self.navigation.last_goal_centroid,
        )
        if selected is None:
            self._log_goal_selection(stats, None, "global reposition planner fallback produced no candidate")
            return False
        self._log_goal_selection(stats, selected, "planner fallback global_reposition")
        candidates = self._planner_validation_candidates(selected)
        started = self.planner_validator.start(
            candidates,
            robot_xy,
            self._marker_frame(),
            self.latest_costmap,
            source_label="global_reposition",
            grid=self.latest_map,
        )
        if started:
            self._log_navigation_skip("local planner validation failed, trying global_reposition")
        return started

    def _planner_validate_before_dispatch(
        self,
        selected: NavigationCandidate,
        robot_xy: Tuple[float, float],
    ) -> Optional[NavigationCandidate]:
        requires_bridge_validation = (
            selected.source == "frontier_bridge"
            and self.frontier_bridge_require_planner_validation
        )
        if not self.use_planner_validation and not requires_bridge_validation:
            return selected

        validation_result = self.planner_validator.take_result()
        if validation_result is not None:
            self.last_validated_path = validation_result.path
            self._log_planner_validation_stats()
            return validation_result.candidate

        if self.planner_validator.pending:
            self._log_navigation_skip("planner validation pending")
            return None

        candidates = self._planner_validation_candidates(selected)
        started = self.planner_validator.start(
            candidates,
            robot_xy,
            self._marker_frame(),
            self.latest_costmap,
            source_label=selected.source
            if selected.source in ("high_cost_escape", "frontier_bridge")
            else "local",
            grid=self.latest_map,
        )
        self.last_goal_attempt_time = self.get_clock().now()
        if started:
            self._log_navigation_skip("planner validation started")
            return None

        self._log_planner_validation_stats()
        if self.planner_validation_required_for_navigation:
            self._log_navigation_skip("no planner-validated candidate")
            return None
        self.get_logger().warn(
            "Planner validation did not produce a valid path; dispatching anyway because "
            "planner_validation_required_for_navigation=false"
        )
        return selected

    def _planner_validation_candidates(
        self,
        selected: NavigationCandidate,
    ) -> List[NavigationCandidate]:
        candidates = list(self.goal_selector.last_valid_candidates)
        candidates.insert(0, selected)
        unique = []
        seen = set()
        for candidate in candidates:
            key = (round(candidate.point_world[0], 2), round(candidate.point_world[1], 2))
            if key in seen:
                continue
            seen.add(key)
            unique.append(candidate)
        unique = diverse_candidates(
            unique,
            self.planner_validation_batch_size * self.planner_validation_max_batches
            if self.planner_validation_retry_next_best
            else self.max_planner_validation_candidates,
            self.max_candidates_per_cluster_for_validation,
            self.candidate_spatial_separation_m,
        )
        return unique

    def _on_planner_candidate_rejected(self, candidate: NavigationCandidate, reason: str) -> None:
        if not self.blacklist_on_planner_reject:
            return
        if candidate.source == "high_cost_escape":
            self.get_logger().warn(
                "high_cost_escape_failed: no lower-cost planner-valid escape reason=%s point=(%.2f, %.2f)"
                % (reason, candidate.point_world[0], candidate.point_world[1])
            )
            self._record_high_cost_escape_failure("planner_reject:%s" % reason)
            return
        if reason == "start_clearance_reject":
            self.get_logger().warn(
                "start clearance rejected near robot; not blacklisting frontier cluster: "
                "cluster_id=%d point=(%.2f, %.2f)"
                % (candidate.cluster_id, candidate.point_world[0], candidate.point_world[1])
            )
            return
        self.planner_reject_cache.add(candidate, reason)

    def _log_planner_validation_stats(self) -> None:
        stats = self.planner_validator.stats
        self.get_logger().info(
            "Planner validation summary: candidates_before_planner_validation=%d "
            "planner_validation_batch_index=%d candidates_in_batch=%d accepted_count=%d "
            "planner_validated_count=%d rejected_by_planner_timeout=%d "
            "rejected_by_no_path=%d rejected_by_path_cost=%d rejected_by_path_unknown=%d "
            "rejected_by_path_clearance=%d strict_validation_failed=%s "
            "trying_relaxed_path_clearance=%s relaxed_selected=%s selected_path_length=%.2f "
            "selected_candidate_after_planner_validation=%s source=%s skip_reason=%s"
            % (
                stats.candidates_before_planner_validation,
                stats.planner_validation_batch_index,
                stats.candidates_in_batch,
                stats.accepted_count,
                stats.planner_validated_count,
                stats.rejected_by_planner_timeout,
                stats.rejected_by_no_path,
                stats.rejected_by_path_cost,
                stats.rejected_by_path_unknown,
                stats.rejected_by_path_clearance,
                stats.strict_validation_failed,
                stats.trying_relaxed_path_clearance,
                stats.relaxed_selected,
                stats.selected_path_length,
                stats.selected_candidate_after_planner_validation,
                stats.source_label,
                stats.skip_reason,
            )
        )
        self.planner_reject_cache.log_cycle_summary()

    def _goal_update_period_elapsed(self) -> bool:
        if self.last_goal_attempt_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_goal_attempt_time).nanoseconds / 1e9
        return elapsed >= self.goal_update_period_sec

    def _lookup_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self._warn_periodic(
                "TF lookup failed %s -> %s: %s"
                % (self.global_frame, self.robot_frame, str(exc))
            )
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def _select_best_candidate(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Optional[Tuple[float, float]],
    ) -> Optional[FrontierCluster]:
        if not clusters:
            return None
        if robot_xy is None:
            return max(clusters, key=lambda cluster: cluster.size)

        def score(cluster: FrontierCluster) -> float:
            distance = euclidean_distance(cluster.centroid_world, robot_xy)
            return float(cluster.size) / max(distance, 0.1)

        return max(clusters, key=score)

    def _publish_clear_markers(self) -> None:
        if not self.enable_debug_markers or not self.publish_empty_markers:
            return
        self.marker_pub.publish(build_clear_markers(self._marker_frame(), self.get_clock()))

    def _marker_frame(self) -> str:
        if self.latest_map is not None and self.latest_map.header.frame_id:
            return self.latest_map.header.frame_id
        return self.global_frame

    def _is_valid_grid(self, grid: OccupancyGrid) -> bool:
        width = int(grid.info.width)
        height = int(grid.info.height)
        return width > 0 and height > 0 and len(grid.data) >= width * height

    def _log_navigation_skip(self, reason: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_nav_log_time).nanoseconds < int(self.log_period_sec * 1e9):
            return
        self.last_nav_log_time = now
        self.get_logger().info(
            "Navigation skip: state=%s enable_navigation=%s reason=%s"
            % (self.navigation.state, self.enable_navigation, reason)
        )

    def _log_goal_selection(
        self,
        stats: CandidateSelectionStats,
        selected: Optional[NavigationCandidate],
        skip_reason: str,
    ) -> None:
        now = self.get_clock().now()
        if self.last_goal_selection_log_time is not None and (
            now - self.last_goal_selection_log_time
        ).nanoseconds < int(self.goal_update_period_sec * 1e9):
            return
        self.last_goal_selection_log_time = now

        if selected is None:
            self.get_logger().info(
                "Goal selection: mode=%s efficient_utility=%s local_candidates=%d "
                "utility_candidates=%d global_attempted=%s global_candidates=%d "
                "total_frontier_clusters=%d sampled_frontier_cells=%d "
                "sampled_viewpoints=%d rejected_by_distance=%d rejected_by_costmap=%d "
                "rejected_by_clearance=%d rejected_by_blacklist=%d rejected_by_progress=%d "
                "selected=none skip_reason=%s"
                % (
                    self.scoring_mode,
                    self.enable_efficient_utility,
                    stats.local_candidates,
                    stats.utility_candidates,
                    stats.global_reposition_attempted,
                    stats.global_candidates,
                    stats.total_frontier_clusters,
                    stats.sampled_frontier_cells,
                    stats.sampled_viewpoints,
                    stats.rejected_by_distance,
                    stats.rejected_by_costmap,
                    stats.rejected_by_clearance,
                    stats.rejected_by_blacklist,
                    stats.rejected_by_progress,
                    skip_reason,
                )
            )
            return

        self.get_logger().info(
            "Goal selection: mode=%s efficient_utility=%s selected_mode=%s fallback_mode=%s relaxed_clearance=%s "
            "local_candidates=%d utility_candidates=%d global_attempted=%s global_candidates=%d "
            "total_frontier_clusters=%d sampled_frontier_cells=%d sampled_viewpoints=%d "
            "rejected_by_distance=%d rejected_by_costmap=%d rejected_by_clearance=%d "
            "rejected_by_blacklist=%d selected_cluster_id=%d selected_source=%s "
            "selected_distance=%.2f selected_score=%.3f selected_cost=%s "
            "selected_world=(%.2f, %.2f) selected_map=(%.1f, %.1f) information_gain=%.3f "
            "safety_radius=%.2f skip_reason=%s"
            % (
                self.scoring_mode,
                self.enable_efficient_utility,
                stats.selected_mode,
                stats.fallback_mode,
                selected.relaxed_clearance,
                stats.local_candidates,
                stats.utility_candidates,
                stats.global_reposition_attempted,
                stats.global_candidates,
                stats.total_frontier_clusters,
                stats.sampled_frontier_cells,
                stats.sampled_viewpoints,
                stats.rejected_by_distance,
                stats.rejected_by_costmap,
                stats.rejected_by_clearance,
                stats.rejected_by_blacklist,
                selected.cluster_id,
                selected.source,
                selected.distance,
                selected.score,
                "unknown" if selected.cost is None else str(selected.cost),
                selected.point_world[0],
                selected.point_world[1],
                selected.point_map[0],
                selected.point_map[1],
                selected.information_gain,
                selected.safety_radius_used,
                skip_reason,
            )
        )

    def _log_frontier_summary(
        self,
        frontier_cell_count: int,
        clusters: List[FrontierCluster],
        best: Optional[FrontierCluster],
        robot_xy: Optional[Tuple[float, float]],
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < int(self.log_period_sec * 1e9):
            return
        self.last_log_time = now

        map_info = self.latest_map.info
        robot_text = "unknown" if robot_xy is None else "(%.2f, %.2f)" % robot_xy
        if best is None:
            self.get_logger().info(
                "Runtime: map=%dx%d res=%.3f robot=%s frontier_cells=%d "
                "frontier_clusters=0 best=none; no frontier clusters found"
                % (map_info.width, map_info.height, map_info.resolution, robot_text, frontier_cell_count)
            )
            return

        distance_text = "unknown"
        if robot_xy is not None:
            distance_text = "%.2f m" % euclidean_distance(best.centroid_world, robot_xy)
        self.get_logger().info(
            "Runtime: map=%dx%d res=%.3f robot=%s frontier_cells=%d frontier_clusters=%d "
            "best_id=%d best_size=%d best_centroid_map=(%.1f, %.1f) "
            "best_centroid_world=(%.2f, %.2f) best_distance=%s"
            % (
                map_info.width,
                map_info.height,
                map_info.resolution,
                robot_text,
                frontier_cell_count,
                len(clusters),
                best.id,
                best.size,
                best.centroid_map[0],
                best.centroid_map[1],
                best.centroid_world[0],
                best.centroid_world[1],
                distance_text,
            )
        )

    def _log_empty_map_summary(self, robot_xy: Optional[Tuple[float, float]]) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < int(self.log_period_sec * 1e9):
            return
        self.last_log_time = now
        robot_text = "unknown" if robot_xy is None else "(%.2f, %.2f)" % robot_xy
        self.get_logger().info(
            "Runtime: map=%dx%d robot=%s frontier_cells=0 frontier_clusters=0 best=none"
            % (self.latest_map.info.width, self.latest_map.info.height, robot_text)
        )

    def _warn_periodic(self, message: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_tf_warning_time).nanoseconds < int(self.log_period_sec * 1e9):
            return
        self.last_tf_warning_time = now
        self.get_logger().warn(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActiveSlamExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.get_default_context().ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
