import math
import time
from dataclasses import replace
from typing import Callable, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from bumperbot_active_slam.costmap_utils import (
    get_cost_at_world,
    is_pose_safe,
    map_pose_is_free,
    max_cost_in_radius,
)
from bumperbot_active_slam.efficient_active_slam_utility import EfficientActiveSlamUtility
from bumperbot_active_slam.entropy_utils import euclidean_distance, map_to_world, world_to_map
from bumperbot_active_slam.frontier_detector import FrontierCluster
from bumperbot_active_slam.models import (
    CandidateSelectionStats,
    GoalSelectorConfig,
    NavigationCandidate,
    Point2D,
)
from bumperbot_active_slam.viewpoint_sampler import (
    sample_cluster_cells,
    sample_frontier_bridge_points,
    sample_global_reposition_points,
    sample_viewpoints_around_frontier,
)


class GoalSelector:
    def __init__(
        self,
        config: GoalSelectorConfig,
        logger,
        is_blacklisted: Callable[[Point2D], bool],
        is_planner_rejected: Optional[Callable[[Point2D, int], bool]] = None,
    ) -> None:
        self.config = config
        self.logger = logger
        self.is_blacklisted = is_blacklisted
        self.is_planner_rejected = is_planner_rejected
        self.last_valid_candidates: List[NavigationCandidate] = []
        self.last_rejected_candidates: List[Point2D] = []
        self.utility = EfficientActiveSlamUtility(config, logger)
        self.initial_robot_xy: Optional[Point2D] = None
        self.cycles_without_goal_candidate = 0
        self.progress_gate_started_time: Optional[float] = None
        self.progress_gate_skip_count = 0
        self.post_escape_force_cycles_remaining = 0
        self.recent_goals: List[Tuple[Point2D, str, str, float]] = []
        self.recorded_result_key: Optional[Tuple[str, str, int]] = None
        self.consecutive_global_reposition_count = 0
        self.post_global_reposition_prefer_frontier_remaining = 0
        self.global_reposition_success_time: Optional[float] = None

    def select(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
        last_goal_robot_xy: Optional[Point2D],
        nav_state: str,
        previous_goal_source: str = "none",
        previous_goal_result: str = "none",
        previous_result_age_sec: Optional[float] = None,
        frontier_cell_count: Optional[int] = None,
    ) -> Tuple[Optional[NavigationCandidate], CandidateSelectionStats]:
        self.update_goal_memory(last_goal_centroid, previous_goal_source, previous_goal_result)
        if self.initial_robot_xy is None:
            self.initial_robot_xy = robot_xy
        gate_allows, gate_stats = self._progress_gate_allows_selection(
            clusters,
            robot_xy,
            last_goal_robot_xy,
            nav_state,
            previous_goal_source,
            previous_goal_result,
            previous_result_age_sec,
        )
        if not gate_allows:
            return None, gate_stats
        local_selected, local_stats, local_candidates, rejected_points = self._select_local_candidate(
            clusters,
            robot_xy,
            grid,
            costmap,
            last_goal_centroid,
            use_utility=True,
        )
        self.last_valid_candidates = local_candidates
        self.last_rejected_candidates = rejected_points
        if local_selected is not None:
            self.cycles_without_goal_candidate = 0
            return local_selected, replace(local_stats, local_candidates=len(local_candidates), skip_reason="none")

        if (
            self._efficient_mode()
            and self.config.enable_utility_fallback_to_baseline
            and self.config.utility_fallback_when_no_candidates
        ):
            self.logger.info("Efficient utility fallback: no safe candidates, trying baseline selector")
            baseline_selected, baseline_stats, baseline_candidates, baseline_rejected = self._select_local_candidate(
                clusters,
                robot_xy,
                grid,
                costmap,
                last_goal_centroid,
                use_utility=False,
            )
            if baseline_selected is not None:
                self.cycles_without_goal_candidate = 0
                self.last_valid_candidates = baseline_candidates
                self.last_rejected_candidates = baseline_rejected
                return baseline_selected, replace(
                    baseline_stats,
                    selected_mode="baseline_fallback",
                    fallback_mode="baseline_fallback",
                    local_candidates=len(baseline_candidates),
                    skip_reason="none",
                )

        if not self.config.enable_global_reposition:
            bootstrap = self._select_bootstrap_if_needed(
                clusters,
                robot_xy,
                grid,
                costmap,
                last_goal_centroid,
                local_stats,
            )
            if bootstrap[0] is not None:
                return bootstrap
            self.cycles_without_goal_candidate += 1
            return None, replace(local_stats, local_candidates=len(local_candidates), skip_reason="no safe viewpoint candidate")

        global_selected, global_stats, global_candidates, global_rejected = (
            self._select_global_reposition_candidate(
                clusters,
                robot_xy,
                grid,
                costmap,
                last_goal_centroid,
                local_stats,
            )
        )
        self.last_valid_candidates = (local_candidates + global_candidates)[:200]
        self.last_rejected_candidates = (rejected_points + global_rejected)[:200]
        if global_selected is not None:
            self.cycles_without_goal_candidate = 0
            return global_selected, global_stats
        bridge_selected, bridge_stats, bridge_candidates, bridge_rejected = (
            self._select_frontier_bridge_candidate(
                clusters,
                robot_xy,
                grid,
                costmap,
                last_goal_centroid,
                global_stats,
                frontier_cell_count,
            )
        )
        self.last_valid_candidates = (local_candidates + global_candidates + bridge_candidates)[:200]
        self.last_rejected_candidates = (rejected_points + global_rejected + bridge_rejected)[:200]
        if bridge_selected is not None:
            self.cycles_without_goal_candidate = 0
            return bridge_selected, bridge_stats
        bootstrap = self._select_bootstrap_if_needed(
            clusters,
            robot_xy,
            grid,
            costmap,
            last_goal_centroid,
            global_stats,
        )
        if bootstrap[0] is not None:
            return bootstrap
        self.cycles_without_goal_candidate += 1
        return None, replace(global_stats, skip_reason="no safe viewpoint candidate")

    def select_global_reposition(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
    ) -> Tuple[Optional[NavigationCandidate], CandidateSelectionStats]:
        base_stats = CandidateSelectionStats(total_frontier_clusters=len(clusters))
        if not self._global_reposition_allowed_by_source_policy():
            return None, replace(
                base_stats,
                global_reposition_attempted=True,
                fallback_mode="global_reposition",
                skip_reason="global reposition cooldown/source policy active",
            )
        selected, stats, candidates, rejected = self._select_global_reposition_candidate(
            clusters,
            robot_xy,
            grid,
            costmap,
            last_goal_centroid,
            base_stats,
        )
        self.last_valid_candidates = candidates[:200]
        self.last_rejected_candidates = rejected[:200]
        if selected is None:
            return None, replace(stats, skip_reason="no global reposition candidate")
        return selected, replace(stats, skip_reason="none")

    def select_medium_reposition(
        self,
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        min_distance_m: float,
        max_distance_m: float,
        sample_count: int,
    ) -> Tuple[Optional[NavigationCandidate], CandidateSelectionStats]:
        candidates: List[NavigationCandidate] = []
        rejected: List[Point2D] = []
        radii = [min_distance_m, 0.5 * (min_distance_m + max_distance_m), max_distance_m]
        for radius in radii:
            for index in range(max(1, sample_count)):
                angle = 2.0 * math.pi * float(index) / float(max(1, sample_count))
                point = (
                    robot_xy[0] + math.cos(angle) * radius,
                    robot_xy[1] + math.sin(angle) * radius,
                )
                if self.is_planner_rejected is not None and self.is_planner_rejected(point, -3):
                    rejected.append(point)
                    continue
                if not map_pose_is_free(point[0], point[1], grid, safety_radius_m=min(self.config.safety_radius_m, 0.10)):
                    rejected.append(point)
                    continue
                safe, cost, _ = self._is_viewpoint_cost_safe(
                    point,
                    costmap,
                    self.config.safety_radius_m,
                    self.config.reject_unknown_cost,
                )
                if not safe:
                    rejected.append(point)
                    continue
                info_gain = self._local_unknown_gain(point, grid)
                candidate = NavigationCandidate(
                    cluster_id=-3,
                    cluster_size=1,
                    point_world=point,
                    point_map=self._world_point_to_map_float(point, grid),
                    distance=euclidean_distance(point, robot_xy),
                    source="medium_reposition",
                    frontier_world=point,
                    score=0.5 + info_gain - self._cost_penalty(cost),
                    information_gain=info_gain,
                    cost=cost,
                    cost_penalty=self._cost_penalty(cost),
                    safety_radius_used=self.config.safety_radius_m,
                )
                candidates.append(candidate)
        self.last_valid_candidates = candidates[:200]
        self.last_rejected_candidates = rejected[:200]
        stats = CandidateSelectionStats(
            sampled_viewpoints=len(candidates) + len(rejected),
            local_candidates=len(candidates),
            selected_mode="medium_reposition" if candidates else "none",
            skip_reason="none" if candidates else "no medium reposition candidate",
        )
        if not candidates:
            return None, stats
        return max(candidates, key=lambda item: item.score), stats

    def _select_local_candidate(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
        use_utility: bool,
    ) -> Tuple[
        Optional[NavigationCandidate],
        CandidateSelectionStats,
        List[NavigationCandidate],
        List[Point2D],
    ]:
        sampled_frontier_cells = 0
        sampled_viewpoints = 0
        rejected_by_distance = 0
        rejected_by_costmap = 0
        rejected_by_clearance = 0
        rejected_by_blacklist = 0
        rejected_by_cluster_size = 0
        candidates: List[NavigationCandidate] = []
        rejected_markers: List[Point2D] = []
        max_cluster_size = max((cluster.size for cluster in clusters), default=1)

        for cluster in clusters:
            if cluster.size < self.config.min_cluster_size_for_navigation:
                rejected_by_cluster_size += 1
                continue
            for frontier_map, frontier_world, source in self._candidate_points_for_cluster(cluster, grid):
                sampled_frontier_cells += 1
                frontier_distance = euclidean_distance(frontier_world, robot_xy)
                if (
                    frontier_distance < self.config.min_candidate_distance_m
                    or frontier_distance > self.config.max_candidate_distance_m
                ):
                    rejected_by_distance += 1
                    rejected_markers.append(frontier_world)
                    continue
                viewpoints = sample_viewpoints_around_frontier(
                    frontier_world,
                    robot_xy,
                    self.config.viewpoint_sample_radius_m,
                    self.config.viewpoint_num_samples,
                    self._safe_viewpoint_sampling_enabled(),
                )
                for viewpoint_world in viewpoints:
                    sampled_viewpoints += 1
                    candidate = self._evaluate_point(
                        cluster,
                        robot_xy,
                        frontier_world,
                        viewpoint_world,
                        source,
                        grid,
                        costmap,
                        last_goal_centroid,
                        self.config.safety_radius_m,
                        self.config.reject_unknown_cost,
                        max_cluster_size,
                    )
                    if candidate is not None:
                        if candidate.information_gain >= self.config.min_information_gain_for_goal:
                            candidates.append(candidate)
                        else:
                            rejected_by_costmap += 1
                            rejected_markers.append(viewpoint_world)
                        continue
                    reason = self._last_reject_reason
                    if reason == "distance":
                        rejected_by_distance += 1
                    elif reason == "blacklist":
                        rejected_by_blacklist += 1
                    elif reason == "clearance":
                        rejected_by_clearance += 1
                    else:
                        rejected_by_costmap += 1
                    rejected_markers.append(viewpoint_world)

        stats = CandidateSelectionStats(
            total_frontier_clusters=len(clusters),
            sampled_frontier_cells=sampled_frontier_cells,
            sampled_viewpoints=sampled_viewpoints,
            rejected_by_distance=rejected_by_distance,
            rejected_by_costmap=rejected_by_costmap,
            rejected_by_clearance=rejected_by_clearance,
            rejected_by_blacklist=rejected_by_blacklist,
            rejected_by_cluster_size=rejected_by_cluster_size,
            local_candidates=len(candidates),
        )
        if not candidates:
            if use_utility and self._efficient_mode():
                self.logger.info("Efficient utility skipped: no safe candidates")
            return None, stats, candidates, rejected_markers
        selected, candidates, selected_mode = self._rank_and_select_candidates(
            candidates,
            grid,
            robot_xy,
            last_goal_centroid,
            rejected_markers,
            use_utility,
            "local_safe_viewpoint",
        )
        stats = replace(
            stats,
            utility_candidates=len(candidates) if use_utility and self._efficient_mode() else 0,
            selected_mode=selected_mode,
        )
        return selected, stats, candidates, rejected_markers

    def _select_global_reposition_candidate(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
        base_stats: CandidateSelectionStats,
    ) -> Tuple[
        Optional[NavigationCandidate],
        CandidateSelectionStats,
        List[NavigationCandidate],
        List[Point2D],
    ]:
        candidates: List[NavigationCandidate] = []
        rejected: List[Point2D] = []
        rejected_recent_region = 0
        rejected_zero_gain = 0
        rejected_pingpong = 0
        if not self._global_reposition_allowed_by_source_policy():
            self.logger.info(
                "global_reposition_rejected: reason=cooldown/source_policy "
                "consecutive_global_reposition_count=%d recent_goal_regions=%d"
                % (self.consecutive_global_reposition_count, len(self.recent_goals))
            )
            return None, replace(
                base_stats,
                global_reposition_attempted=True,
                fallback_mode="global_reposition",
                skip_reason="global reposition cooldown/source policy active",
            ), candidates, rejected
        max_cluster_size = max((cluster.size for cluster in clusters), default=1)

        far_clusters = []
        for cluster in clusters:
            if cluster.size < self.config.min_cluster_size_for_navigation:
                continue
            distance = euclidean_distance(cluster.centroid_world, robot_xy)
            if (
                distance < self.config.global_reposition_min_distance_m
                or distance > self.config.global_reposition_max_distance_m
            ):
                continue
            gain = self._local_unknown_gain(cluster.centroid_world, grid)
            far_clusters.append((cluster, distance, gain))

        far_clusters.sort(
            key=lambda item: (
                item[2],
                float(item[0].size) / float(max_cluster_size),
                item[1] / max(self.config.global_reposition_max_distance_m, 0.1),
            ),
            reverse=True,
        )

        for cluster, _, _ in far_clusters[: max(1, min(12, len(far_clusters)))]:
            target_world = cluster.centroid_world
            points = sample_global_reposition_points(
                robot_xy,
                target_world,
                self.config.global_reposition_step_m,
                self.config.global_reposition_sample_count,
            )
            for point in points:
                candidate = self._evaluate_point(
                    cluster,
                    robot_xy,
                    target_world,
                    point,
                    "global_reposition",
                    grid,
                    costmap,
                    last_goal_centroid,
                    self.config.safety_radius_m,
                    self.config.reject_unknown_cost,
                    max_cluster_size,
                )
                if candidate is None and self.config.fallback_relax_clearance:
                    candidate = self._evaluate_point(
                        cluster,
                        robot_xy,
                        target_world,
                        point,
                        "global_reposition_relaxed",
                        grid,
                        costmap,
                        last_goal_centroid,
                        self.config.fallback_safety_radius_m,
                        self.config.fallback_reject_unknown_cost,
                        max_cluster_size,
                    )
                    if candidate is not None:
                        candidate = replace(candidate, relaxed_clearance=True)
                if candidate is None:
                    rejected.append(point)
                    continue
                reject_reason = self._global_reposition_reject_reason(candidate)
                if reject_reason is not None:
                    rejected.append(point)
                    if reject_reason == "recent_region":
                        rejected_recent_region += 1
                    elif reject_reason == "zero_gain":
                        rejected_zero_gain += 1
                    elif reject_reason == "pingpong":
                        rejected_pingpong += 1
                    self.logger.info(
                        "global_reposition_rejected: reason=%s point=(%.2f, %.2f) "
                        "information_gain=%.3f recent_goal_regions=%d "
                        "consecutive_global_reposition_count=%d"
                        % (
                            reject_reason,
                            candidate.point_world[0],
                            candidate.point_world[1],
                            candidate.information_gain,
                            len(self.recent_goals),
                            self.consecutive_global_reposition_count,
                        )
                    )
                    continue
                candidate = self._penalize_global_reposition_candidate(candidate)
                candidates.append(candidate)

        stats = replace(
            base_stats,
            global_reposition_attempted=True,
            global_candidates=len(candidates),
            selected_mode="global_reposition" if candidates else "none",
            fallback_mode="global_reposition",
            relaxed_clearance=any(candidate.relaxed_clearance for candidate in candidates),
            safety_radius_used=(
                self.config.fallback_safety_radius_m
                if any(candidate.relaxed_clearance for candidate in candidates)
                else self.config.safety_radius_m
            ),
        )
        if not candidates:
            self.logger.info(
                "Global reposition summary: recent_goal_regions=%d consecutive_global_reposition_count=%d "
                "global_reposition_cooldown_active=%s pingpong_detected=%s "
                "candidate_rejected_recent_region=%d candidate_rejected_zero_gain=%d "
                "candidate_rejected_pingpong=%d selected_source=none selected_mode=none "
                "selected_information_gain=0.000"
                % (
                    len(self.recent_goals),
                    self.consecutive_global_reposition_count,
                    self._global_reposition_cooldown_active(),
                    self._pingpong_detected(),
                    rejected_recent_region,
                    rejected_zero_gain,
                    rejected_pingpong,
                )
            )
            return None, stats, candidates, rejected
        selected, candidates, selected_mode = self._rank_and_select_candidates(
            candidates,
            grid,
            robot_xy,
            last_goal_centroid,
            rejected,
            True,
            "global_reposition",
        )
        self.logger.info(
            "Global reposition summary: recent_goal_regions=%d consecutive_global_reposition_count=%d "
            "global_reposition_cooldown_active=%s pingpong_detected=%s "
            "candidate_rejected_recent_region=%d candidate_rejected_zero_gain=%d "
            "candidate_rejected_pingpong=%d selected_source=%s selected_mode=%s "
            "selected_information_gain=%.3f"
            % (
                len(self.recent_goals),
                self.consecutive_global_reposition_count,
                self._global_reposition_cooldown_active(),
                self._pingpong_detected(),
                rejected_recent_region,
                rejected_zero_gain,
                rejected_pingpong,
                selected.source,
                selected_mode,
                selected.information_gain,
            )
        )
        return selected, replace(stats, selected_mode=selected_mode), candidates, rejected

    def _select_frontier_bridge_candidate(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
        base_stats: CandidateSelectionStats,
        frontier_cell_count: Optional[int],
    ) -> Tuple[
        Optional[NavigationCandidate],
        CandidateSelectionStats,
        List[NavigationCandidate],
        List[Point2D],
    ]:
        candidates: List[NavigationCandidate] = []
        rejected: List[Point2D] = []
        if not self.config.enable_frontier_bridge_reposition:
            return None, base_stats, candidates, rejected

        counted_frontier_cells = (
            int(frontier_cell_count)
            if frontier_cell_count is not None
            else sum(cluster.size for cluster in clusters)
        )
        if counted_frontier_cells < self.config.frontier_bridge_min_frontier_cells:
            self.logger.info(
                "frontier_bridge_rejected: reason=frontier_cells_below_threshold "
                "frontier_cells=%d threshold=%d"
                % (counted_frontier_cells, self.config.frontier_bridge_min_frontier_cells)
            )
            return None, base_stats, candidates, rejected

        navigable_clusters = [
            cluster for cluster in clusters
            if cluster.size >= self.config.min_cluster_size_for_navigation
        ]
        if not navigable_clusters:
            return None, base_stats, candidates, rejected
        best_cluster = max(navigable_clusters, key=lambda cluster: cluster.size)
        best_distance = euclidean_distance(best_cluster.centroid_world, robot_xy)
        if best_cluster.size < self.config.frontier_bridge_min_best_cluster_size:
            self.logger.info(
                "frontier_bridge_rejected: reason=best_cluster_size_below_threshold "
                "best_size=%d threshold=%d"
                % (best_cluster.size, self.config.frontier_bridge_min_best_cluster_size)
            )
            return None, base_stats, candidates, rejected
        if best_distance < self.config.frontier_bridge_min_best_distance_m:
            self.logger.info(
                "frontier_bridge_rejected: reason=best_distance_below_threshold "
                "best_distance=%.2f threshold=%.2f"
                % (best_distance, self.config.frontier_bridge_min_best_distance_m)
            )
            return None, base_stats, candidates, rejected

        robot_near_cost = self._max_cost_near_point(robot_xy, costmap, self.config.safety_radius_m)
        if robot_near_cost is None and self.config.use_costmap_filter:
            self.logger.info("frontier_bridge_rejected: reason=robot_cost_unknown")
            return None, base_stats, candidates, rejected
        if robot_near_cost is not None and robot_near_cost > self.config.frontier_bridge_max_near_cost:
            self.logger.info(
                "frontier_bridge_rejected: reason=robot_high_cost robot_near_cost=%d threshold=%d"
                % (robot_near_cost, self.config.frontier_bridge_max_near_cost)
            )
            return None, base_stats, candidates, rejected

        bridge_points = sample_frontier_bridge_points(
            robot_xy,
            best_cluster.centroid_world,
            list(self.config.frontier_bridge_step_distances_m),
            list(self.config.frontier_bridge_lateral_offsets_m),
        )
        self.logger.info(
            "frontier_bridge_triggered: frontier_cells=%d best_cluster_id=%d best_size=%d "
            "best_distance=%.2f best_centroid_world=(%.2f, %.2f) robot=(%.2f, %.2f) "
            "candidate_points=%d robot_near_cost=%s"
            % (
                counted_frontier_cells,
                best_cluster.id,
                best_cluster.size,
                best_distance,
                best_cluster.centroid_world[0],
                best_cluster.centroid_world[1],
                robot_xy[0],
                robot_xy[1],
                len(bridge_points),
                "unknown" if robot_near_cost is None else str(robot_near_cost),
            )
        )

        max_cluster_size = max((cluster.size for cluster in clusters), default=1)
        for point, step_m, lateral_m in bridge_points:
            reject_reason = self._frontier_bridge_reject_reason(
                point,
                -6,
                grid,
                costmap,
                last_goal_centroid,
            )
            if reject_reason is not None:
                rejected.append(point)
                self.logger.info(
                    "frontier_bridge_rejected: reason=%s point=(%.2f, %.2f) "
                    "step=%.2f lateral=%.2f"
                    % (reject_reason, point[0], point[1], step_m, lateral_m)
                )
                continue

            cost = None if costmap is None else get_cost_at_world(point[0], point[1], costmap)
            info_gain = self._local_unknown_gain(point, grid)
            cost_penalty = self._cost_penalty(cost)
            candidate = NavigationCandidate(
                cluster_id=-6,
                cluster_size=best_cluster.size,
                point_world=point,
                point_map=self._world_point_to_map_float(point, grid),
                distance=euclidean_distance(point, robot_xy),
                source="frontier_bridge",
                frontier_world=best_cluster.centroid_world,
                information_gain=info_gain,
                cost=cost,
                cost_penalty=cost_penalty,
                safety_radius_used=self.config.safety_radius_m,
            )
            candidate = replace(candidate, score=self._score_candidate(candidate, max_cluster_size))
            candidates.append(candidate)
            self.logger.info(
                "frontier_bridge_candidate: point=(%.2f, %.2f) step=%.2f lateral=%.2f "
                "distance=%.2f cost=%s information_gain=%.3f score=%.3f"
                % (
                    point[0],
                    point[1],
                    step_m,
                    lateral_m,
                    candidate.distance,
                    "unknown" if cost is None else str(cost),
                    candidate.information_gain,
                    candidate.score,
                )
            )

        stats = replace(
            base_stats,
            sampled_viewpoints=base_stats.sampled_viewpoints + len(bridge_points),
            selected_mode="frontier_bridge" if candidates else "none",
            fallback_mode="frontier_bridge",
        )
        if not candidates:
            self.logger.info(
                "frontier_bridge_rejected: reason=no_safe_bridge_candidate "
                "candidate_points=%d rejected=%d"
                % (len(bridge_points), len(rejected))
            )
            return None, stats, candidates, rejected

        selected, candidates, selected_mode = self._rank_and_select_candidates(
            candidates,
            grid,
            robot_xy,
            last_goal_centroid,
            rejected,
            False,
            "frontier_bridge",
        )
        self.logger.info(
            "frontier_bridge_selected: point=(%.2f, %.2f) distance=%.2f cost=%s "
            "information_gain=%.3f score=%.3f source=%s selected_mode=%s "
            "planner_validation_required=%s"
            % (
                selected.point_world[0],
                selected.point_world[1],
                selected.distance,
                "unknown" if selected.cost is None else str(selected.cost),
                selected.information_gain,
                selected.score,
                selected.source,
                selected_mode,
                self.config.frontier_bridge_require_planner_validation,
            )
        )
        return selected, replace(stats, selected_mode=selected_mode, skip_reason="none"), candidates, rejected

    def _frontier_bridge_reject_reason(
        self,
        point_xy: Point2D,
        cluster_id: int,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
    ) -> Optional[str]:
        if not math.isfinite(point_xy[0]) or not math.isfinite(point_xy[1]):
            return "invalid_point"
        if self.is_blacklisted(point_xy):
            return "blacklist"
        if self.is_planner_rejected is not None and self.is_planner_rejected(point_xy, cluster_id):
            return "planner_reject_cache"
        if last_goal_centroid is not None and euclidean_distance(
            point_xy,
            last_goal_centroid,
        ) < self.config.min_goal_separation_m:
            return "goal_separation"
        if not map_pose_is_free(
            point_xy[0],
            point_xy[1],
            grid,
            safety_radius_m=min(self.config.safety_radius_m, 0.10),
        ):
            return "map_clearance"
        if costmap is None or not self._is_valid_grid(costmap):
            return None if not self.config.use_costmap_filter else "costmap_unavailable"
        goal_cost = get_cost_at_world(point_xy[0], point_xy[1], costmap)
        if goal_cost is None:
            return "goal_cost_unknown"
        if goal_cost < 0:
            return "goal_cost_unknown"
        if goal_cost > self.config.frontier_bridge_max_goal_cost:
            return "goal_cost"
        near_cost = self._max_cost_near_point(point_xy, costmap, self.config.safety_radius_m)
        if near_cost is None:
            return "near_cost_unknown"
        if near_cost > self.config.frontier_bridge_max_near_cost:
            return "near_cost"
        if not is_pose_safe(
            point_xy[0],
            point_xy[1],
            costmap,
            self.config.frontier_bridge_max_near_cost,
            True,
            self.config.safety_radius_m,
        ):
            return "clearance"
        return None

    def _max_cost_near_point(
        self,
        point_xy: Point2D,
        costmap: Optional[OccupancyGrid],
        radius_m: float,
    ) -> Optional[int]:
        if costmap is None or not self._is_valid_grid(costmap):
            return None
        return max_cost_in_radius(point_xy[0], point_xy[1], costmap, radius_m)

    def update_goal_memory(
        self,
        goal_xy: Optional[Point2D],
        source: str,
        result: str,
    ) -> None:
        if goal_xy is None or result not in ("SUCCEEDED", "FAILED", "CANCELED"):
            return
        key = (source, result, round(goal_xy[0] * 10), round(goal_xy[1] * 10))
        if key == self.recorded_result_key:
            return
        self.recorded_result_key = key
        now = time.monotonic()
        self._expire_recent_goals(now)
        self.recent_goals.append((goal_xy, source, result, now))
        self.recent_goals = self.recent_goals[-max(2, self.config.global_reposition_pingpong_window):]
        if source == "global_reposition" and result == "SUCCEEDED":
            self.consecutive_global_reposition_count += 1
            self.global_reposition_success_time = now
            self.post_global_reposition_prefer_frontier_remaining = (
                self.config.post_global_reposition_prefer_frontier_cycles
            )
        elif source != "global_reposition" and result == "SUCCEEDED":
            self.consecutive_global_reposition_count = 0
        if self._pingpong_detected():
            self.logger.warn("pingpong_detected: suppressing global_reposition regions for cooldown")

    def _expire_recent_goals(self, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        timeout = max(
            self.config.recent_goal_region_timeout_sec,
            self.config.global_reposition_blacklist_after_success_sec,
        )
        self.recent_goals = [
            item for item in self.recent_goals
            if now - item[3] <= timeout
        ]

    def _global_reposition_allowed_by_source_policy(self) -> bool:
        if self.post_global_reposition_prefer_frontier_remaining > 0:
            self.post_global_reposition_prefer_frontier_remaining -= 1
            self.logger.info(
                "global_reposition_rejected: reason=prefer_frontier_after_success remaining_cycles=%d"
                % self.post_global_reposition_prefer_frontier_remaining
            )
            return False
        if (
            self.config.global_reposition_max_consecutive_goals > 0
            and self.consecutive_global_reposition_count >= self.config.global_reposition_max_consecutive_goals
        ):
            self.logger.info(
                "global_reposition_rejected: reason=max_consecutive count=%d"
                % self.consecutive_global_reposition_count
            )
            return False
        return True

    def _global_reposition_reject_reason(self, candidate: NavigationCandidate) -> Optional[str]:
        if self._recent_region_hit(candidate.point_world, self.config.global_reposition_recent_goal_radius_m):
            return "recent_region"
        if self._pingpong_candidate_hit(candidate.point_world):
            return "pingpong"
        if (
            self.config.enable_goal_usefulness_gate
            and candidate.information_gain < self.config.global_reposition_min_information_gain
        ):
            if not self.config.global_reposition_allow_zero_gain_only_if_no_alternative:
                return "zero_gain"
            if self._recent_region_hit(candidate.point_world, self.config.min_goal_frontier_distance_from_recent_m):
                return "zero_gain"
        return None

    def _penalize_global_reposition_candidate(self, candidate: NavigationCandidate) -> NavigationCandidate:
        penalty = 0.0
        if self._recent_region_hit(candidate.point_world, self.config.recent_goal_region_radius_m):
            penalty += self.config.global_reposition_recent_region_penalty
        if (
            self.config.enable_goal_usefulness_gate
            and candidate.information_gain < self.config.min_goal_information_gain
            and not self.config.allow_low_gain_recovery_goal
        ):
            penalty += 1.0
        if penalty <= 0.0:
            return candidate
        return replace(candidate, score=candidate.score - penalty)

    def _recent_region_hit(self, point_xy: Point2D, radius_m: float) -> bool:
        if radius_m <= 0.0:
            return False
        self._expire_recent_goals()
        for goal_xy, source, result, stamp in self.recent_goals:
            if source == "high_cost_escape":
                continue
            if euclidean_distance(point_xy, goal_xy) <= radius_m:
                return True
        return False

    def _global_reposition_cooldown_active(self) -> bool:
        if self.global_reposition_success_time is None:
            return False
        return time.monotonic() - self.global_reposition_success_time <= self.config.global_reposition_cooldown_sec

    def _pingpong_detected(self) -> bool:
        goals = [
            item for item in self.recent_goals[-self.config.global_reposition_pingpong_window:]
            if item[1] == "global_reposition" and item[2] == "SUCCEEDED"
        ]
        if len(goals) < 4:
            return False
        a = goals[-1][0]
        b = goals[-2][0]
        for point, _, _, _ in goals[-4:-2]:
            if euclidean_distance(point, a) <= self.config.global_reposition_pingpong_radius_m:
                return True
            if euclidean_distance(point, b) <= self.config.global_reposition_pingpong_radius_m:
                return True
        return False

    def _pingpong_candidate_hit(self, point_xy: Point2D) -> bool:
        if not self._pingpong_detected():
            return False
        for goal_xy, source, result, _ in self.recent_goals[-self.config.global_reposition_pingpong_window:]:
            if source == "global_reposition" and euclidean_distance(
                point_xy,
                goal_xy,
            ) <= self.config.global_reposition_pingpong_radius_m:
                return True
        return False

    def _rank_and_select_candidates(
        self,
        candidates: List[NavigationCandidate],
        grid: OccupancyGrid,
        robot_xy: Point2D,
        last_goal_centroid: Optional[Point2D],
        rejected_points: List[Point2D],
        use_utility: bool,
        baseline_mode: str,
    ) -> Tuple[NavigationCandidate, List[NavigationCandidate], str]:
        if use_utility and self._efficient_mode():
            if len(candidates) >= self.config.utility_min_safe_candidates_before_ranking:
                ranked = self.utility.rank_candidates(
                    candidates,
                    grid,
                    robot_xy,
                    last_goal_centroid,
                    rejected_points,
                )
                return ranked[0], ranked, "efficient_utility"
            self.logger.info(
                "Efficient utility fallback: using baseline safe_viewpoint safe_candidates=%d "
                "min_required=%d"
                % (len(candidates), self.config.utility_min_safe_candidates_before_ranking)
            )
            return max(candidates, key=lambda candidate: candidate.score), candidates, "baseline_fallback"
        return max(candidates, key=lambda candidate: candidate.score), candidates, baseline_mode

    def _select_bootstrap_if_needed(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
        base_stats: CandidateSelectionStats,
    ) -> Tuple[Optional[NavigationCandidate], CandidateSelectionStats]:
        if not self._should_bootstrap(clusters, robot_xy):
            return None, base_stats
        selected, candidates, rejected = self._select_bootstrap_candidate(
            robot_xy,
            grid,
            costmap,
            last_goal_centroid,
        )
        self.last_valid_candidates = candidates[:200]
        self.last_rejected_candidates = rejected[:200]
        if selected is None:
            return None, replace(
                base_stats,
                selected_mode="none",
                fallback_mode="bootstrap",
                sampled_viewpoints=base_stats.sampled_viewpoints + len(candidates) + len(rejected),
                skip_reason="bootstrap exploration produced no candidate",
            )
        self.cycles_without_goal_candidate = 0
        self.logger.info(
            "Bootstrap exploration: selected candidate=(%.2f, %.2f) distance=%.2f "
            "reason=initial_no_safe_candidates"
            % (selected.point_world[0], selected.point_world[1], selected.distance)
        )
        return selected, replace(
            base_stats,
            selected_mode="bootstrap",
            fallback_mode="bootstrap",
            sampled_viewpoints=base_stats.sampled_viewpoints + len(candidates) + len(rejected),
            local_candidates=base_stats.local_candidates + len(candidates),
            skip_reason="none",
        )

    def _select_bootstrap_candidate(
        self,
        robot_xy: Point2D,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
    ) -> Tuple[Optional[NavigationCandidate], List[NavigationCandidate], List[Point2D]]:
        candidates: List[NavigationCandidate] = []
        rejected: List[Point2D] = []
        sample_count = max(12, self.config.viewpoint_num_samples)
        radii = [
            self.config.bootstrap_min_goal_distance_m,
            0.5 * (self.config.bootstrap_min_goal_distance_m + self.config.bootstrap_max_goal_distance_m),
            self.config.bootstrap_max_goal_distance_m,
        ]
        for radius in radii:
            for index in range(sample_count):
                angle = 2.0 * math.pi * float(index) / float(sample_count)
                point = (
                    robot_xy[0] + math.cos(angle) * radius,
                    robot_xy[1] + math.sin(angle) * radius,
                )
                if self.is_planner_rejected is not None and self.is_planner_rejected(point, -5):
                    rejected.append(point)
                    continue
                if last_goal_centroid is not None and euclidean_distance(point, last_goal_centroid) < self.config.min_goal_separation_m:
                    rejected.append(point)
                    continue
                if self.config.bootstrap_use_known_free_space and not map_pose_is_free(
                    point[0],
                    point[1],
                    grid,
                    safety_radius_m=min(self.config.bootstrap_safety_radius_m, 0.10),
                ):
                    rejected.append(point)
                    continue
                safe, cost, _ = self._is_viewpoint_cost_safe(
                    point,
                    costmap,
                    self.config.bootstrap_safety_radius_m,
                    self.config.reject_unknown_cost,
                )
                if not safe and self.config.bootstrap_allow_relaxed_clearance:
                    safe, cost, _ = self._is_viewpoint_cost_safe(
                        point,
                        costmap,
                        min(self.config.bootstrap_safety_radius_m, 0.10),
                        False,
                    )
                if not safe:
                    rejected.append(point)
                    continue
                info_gain = self._local_unknown_gain(point, grid)
                cost_penalty = self._cost_penalty(cost)
                candidate = NavigationCandidate(
                    cluster_id=-5,
                    cluster_size=1,
                    point_world=point,
                    point_map=self._world_point_to_map_float(point, grid),
                    distance=euclidean_distance(point, robot_xy),
                    source="bootstrap",
                    frontier_world=point,
                    score=0.75 + info_gain - cost_penalty,
                    information_gain=info_gain,
                    cost=cost,
                    cost_penalty=cost_penalty,
                    safety_radius_used=self.config.bootstrap_safety_radius_m,
                    relaxed_clearance=self.config.bootstrap_allow_relaxed_clearance,
                )
                candidates.append(candidate)
        if not candidates:
            return None, candidates, rejected
        return max(candidates, key=lambda candidate: candidate.score), candidates, rejected

    def _should_bootstrap(self, clusters: List[FrontierCluster], robot_xy: Point2D) -> bool:
        if not self.config.enable_bootstrap_exploration:
            return False
        if not self._efficient_mode():
            return False
        if self.initial_robot_xy is None:
            return True
        travel = euclidean_distance(robot_xy, self.initial_robot_xy)
        if travel < self.config.utility_bootstrap_min_robot_travel_m:
            return True
        if len(clusters) < self.config.utility_bootstrap_min_frontier_clusters:
            return True
        return self.cycles_without_goal_candidate >= self.config.bootstrap_max_cycles_without_goal

    def _progress_gate_allows_selection(
        self,
        clusters: List[FrontierCluster],
        robot_xy: Point2D,
        last_goal_robot_xy: Optional[Point2D],
        nav_state: str,
        previous_goal_source: str,
        previous_goal_result: str,
        previous_result_age_sec: Optional[float],
    ) -> Tuple[bool, CandidateSelectionStats]:
        if not self.config.progress_gate_enabled:
            self._reset_progress_gate()
            return True, CandidateSelectionStats()
        if nav_state != "SUCCEEDED" or last_goal_robot_xy is None:
            self._reset_progress_gate()
            return True, CandidateSelectionStats()

        moved = euclidean_distance(robot_xy, last_goal_robot_xy)
        threshold = self.config.progress_gate_min_distance_m
        if moved >= threshold:
            self._reset_progress_gate()
            return True, CandidateSelectionStats()

        if (
            self.config.progress_gate_disable_after_escape
            and previous_goal_source == "high_cost_escape"
            and previous_goal_result == "SUCCEEDED"
        ):
            age = 0.0 if previous_result_age_sec is None else previous_result_age_sec
            if age >= self.config.post_escape_resume_delay_sec:
                self.post_escape_force_cycles_remaining = max(
                    self.post_escape_force_cycles_remaining,
                    self.config.post_escape_force_selection_cycles,
                )
                self._reset_progress_gate()
                self.logger.info(
                    "post_escape_resume: high_cost_escape succeeded, progress gate relaxed "
                    "age=%.2f force_selection_cycles=%d"
                    % (age, self.post_escape_force_cycles_remaining)
                )
                return True, CandidateSelectionStats()

        if self.post_escape_force_cycles_remaining > 0:
            self.post_escape_force_cycles_remaining -= 1
            self._reset_progress_gate()
            self.logger.info(
                "post_escape_resume: forcing selection despite progress gate remaining_cycles=%d"
                % self.post_escape_force_cycles_remaining
            )
            return True, CandidateSelectionStats()

        now = time.monotonic()
        if self.progress_gate_started_time is None:
            self.progress_gate_started_time = now
            self.progress_gate_skip_count = 0
        age = now - self.progress_gate_started_time
        self.progress_gate_skip_count += 1
        if age >= self.config.progress_gate_timeout_sec:
            self.logger.info(
                "progress_gate_timeout: allowing new goal selection age=%.2f moved=%.3f threshold=%.3f"
                % (age, moved, threshold)
            )
            self._reset_progress_gate()
            return True, CandidateSelectionStats()
        if self.progress_gate_skip_count > self.config.progress_gate_max_skip_cycles:
            self.logger.info(
                "progress_gate_max_skip_cycles reached; resuming exploration "
                "skip_count=%d moved=%.3f threshold=%.3f"
                % (self.progress_gate_skip_count, moved, threshold)
            )
            self._reset_progress_gate()
            return True, CandidateSelectionStats()

        skip_reason = (
            "robot has not progressed enough since previous goal "
            "progress_gate_age_sec=%.2f progress_gate_skip_count=%d "
            "previous_goal_source=%s previous_goal_result=%s "
            "robot_moved_since_previous_goal=%.3f threshold=%.3f"
            % (
                age,
                self.progress_gate_skip_count,
                previous_goal_source,
                previous_goal_result,
                moved,
                threshold,
            )
        )
        return False, CandidateSelectionStats(
            total_frontier_clusters=len(clusters),
            rejected_by_progress=1,
            skip_reason=skip_reason,
        )

    def _reset_progress_gate(self) -> None:
        self.progress_gate_started_time = None
        self.progress_gate_skip_count = 0

    def _efficient_mode(self) -> bool:
        return self.config.enable_efficient_utility and self.config.scoring_mode == "efficient_entropy_utility"

    def _safe_viewpoint_sampling_enabled(self) -> bool:
        return self.config.scoring_mode in ("safe_viewpoint", "efficient_entropy_utility")

    def _evaluate_point(
        self,
        cluster: FrontierCluster,
        robot_xy: Point2D,
        frontier_world: Point2D,
        point_world: Point2D,
        source: str,
        grid: OccupancyGrid,
        costmap: Optional[OccupancyGrid],
        last_goal_centroid: Optional[Point2D],
        safety_radius_m: float,
        reject_unknown_cost: bool,
        max_cluster_size: int,
    ) -> Optional[NavigationCandidate]:
        distance = euclidean_distance(point_world, robot_xy)
        min_distance = (
            self.config.global_reposition_step_m * 0.5
            if source.startswith("global_reposition")
            else self.config.min_viewpoint_distance_m
        )
        max_distance = (
            self.config.global_reposition_step_m * 1.6
            if source.startswith("global_reposition")
            else self.config.max_viewpoint_distance_m
        )
        if distance < min_distance or distance > max_distance:
            self._last_reject_reason = "distance"
            return None
        if self.is_blacklisted(point_world):
            self._last_reject_reason = "blacklist"
            return None
        if self.is_planner_rejected is not None and self.is_planner_rejected(point_world, cluster.id):
            self._last_reject_reason = "blacklist"
            return None
        if last_goal_centroid is not None:
            separation = euclidean_distance(point_world, last_goal_centroid)
            if separation < self.config.min_goal_separation_m:
                self._last_reject_reason = "blacklist"
                return None
        if not map_pose_is_free(point_world[0], point_world[1], grid, safety_radius_m=min(safety_radius_m, 0.10)):
            self._last_reject_reason = "clearance"
            return None

        safe, cost, rejected_reason = self._is_viewpoint_cost_safe(
            point_world,
            costmap,
            safety_radius_m,
            reject_unknown_cost,
        )
        if not safe:
            self._last_reject_reason = rejected_reason
            return None

        info_gain = self._local_unknown_gain(point_world, grid)
        cost_penalty = self._cost_penalty(cost)
        candidate = NavigationCandidate(
            cluster_id=cluster.id,
            cluster_size=cluster.size,
            point_world=point_world,
            point_map=self._world_point_to_map_float(point_world, grid),
            distance=distance,
            source=source,
            frontier_world=frontier_world,
            information_gain=info_gain,
            cost=cost,
            cost_penalty=cost_penalty,
            safety_radius_used=safety_radius_m,
            relaxed_clearance=source.endswith("relaxed"),
        )
        return replace(candidate, score=self._score_candidate(candidate, max_cluster_size))

    def _candidate_points_for_cluster(
        self,
        cluster: FrontierCluster,
        grid: OccupancyGrid,
    ) -> List[Tuple[Tuple[float, float], Point2D, str]]:
        if self.config.goal_candidate_mode == "centroid":
            return [(cluster.centroid_map, cluster.centroid_world, "centroid")]

        sampled_cells = sample_cluster_cells(cluster.cells, self.config.max_cells_sampled_per_cluster)
        return [
            ((float(mx), float(my)), map_to_world(mx, my, grid.info), "frontier_cell")
            for mx, my in sampled_cells
        ]

    def _is_viewpoint_cost_safe(
        self,
        point_xy: Point2D,
        costmap: Optional[OccupancyGrid],
        safety_radius_m: float,
        reject_unknown_cost: bool,
    ) -> Tuple[bool, Optional[int], str]:
        if not self.config.use_costmap_filter:
            return True, None, "disabled"
        if costmap is None or not self._is_valid_grid(costmap):
            return True, None, "fallback"

        cost = get_cost_at_world(point_xy[0], point_xy[1], costmap)
        if cost is None:
            return False, cost, "costmap"
        if not is_pose_safe(
            point_xy[0],
            point_xy[1],
            costmap,
            self.config.max_allowed_cost,
            reject_unknown_cost,
            safety_radius_m,
        ):
            return False, cost, "clearance"
        return True, cost, "safe"

    def _world_point_to_map_float(self, point_xy: Point2D, grid: OccupancyGrid) -> Tuple[float, float]:
        cell = world_to_map(point_xy[0], point_xy[1], grid.info)
        if cell is None:
            return -1.0, -1.0
        return float(cell[0]), float(cell[1])

    def _local_unknown_gain(self, point_xy: Point2D, grid: OccupancyGrid) -> float:
        if not self._is_valid_grid(grid):
            return 0.0
        center = world_to_map(point_xy[0], point_xy[1], grid.info)
        if center is None:
            return 0.0

        resolution = float(grid.info.resolution)
        radius_cells = max(1, int(math.ceil(self.config.information_radius_m / resolution)))
        width = int(grid.info.width)
        height = int(grid.info.height)
        cx, cy = center
        unknown = 0
        total = 0
        for my in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
            for mx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
                if euclidean_distance((mx, my), (cx, cy)) * resolution > self.config.information_radius_m:
                    continue
                total += 1
                if int(grid.data[my * width + mx]) < 0:
                    unknown += 1
        if total == 0:
            return 0.0
        return float(unknown) / float(total)

    def _cost_penalty(self, cost: Optional[int]) -> float:
        if cost is None or cost < 0:
            return 0.0
        return min(1.0, float(cost) / max(1.0, float(self.config.max_allowed_cost)))

    def _score_candidate(self, candidate: NavigationCandidate, max_cluster_size: int) -> float:
        normalized_cluster_size = float(candidate.cluster_size) / max(1.0, float(max_cluster_size))
        target_distance = 0.5 * (
            self.config.min_viewpoint_distance_m + self.config.max_viewpoint_distance_m
        )
        half_range = max(
            0.1,
            0.5 * (self.config.max_viewpoint_distance_m - self.config.min_viewpoint_distance_m),
        )
        distance_score = 1.0 - min(1.0, abs(candidate.distance - target_distance) / half_range)
        if self.config.prefer_farther_than_current:
            distance_score = max(
                distance_score,
                min(1.0, candidate.distance / self.config.max_viewpoint_distance_m),
            )
        if candidate.source.startswith("global_reposition"):
            distance_score = 1.0

        goal_switch_penalty = 0.0
        return (
            self.config.w_cluster_size * normalized_cluster_size
            + self.config.w_distance * distance_score
            + self.config.w_information_gain * candidate.information_gain
            - self.config.w_cost_penalty * candidate.cost_penalty
            - self.config.w_goal_switching * goal_switch_penalty
        )

    def _is_valid_grid(self, grid: OccupancyGrid) -> bool:
        width = int(grid.info.width)
        height = int(grid.info.height)
        return width > 0 and height > 0 and len(grid.data) >= width * height
