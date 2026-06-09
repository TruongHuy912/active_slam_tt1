from dataclasses import dataclass, replace
from typing import Iterable, List, Optional

from nav_msgs.msg import OccupancyGrid

from bumperbot_active_slam.frontier_detector import FrontierCluster
from bumperbot_active_slam.information_gain import compute_local_information_gain
from bumperbot_active_slam.models import GoalSelectorConfig, NavigationCandidate, Point2D
from bumperbot_active_slam.path_entropy import estimate_straight_line_path_entropy
from bumperbot_active_slam.region_diversity import limit_candidates_by_region, region_diversity_score
from bumperbot_active_slam.uncertainty_utils import compute_uncertainty_proxy


@dataclass
class EfficientUtilityStats:
    utility_candidates_before_limit: int = 0
    utility_candidates_after_limit: int = 0
    top_cluster_count: int = 0
    selected_before_planner_validation: Optional[NavigationCandidate] = None
    top_path_entropy: float = 0.0
    top_information_gain: float = 0.0
    top_uncertainty_proxy: float = 0.0
    top_region_diversity: float = 0.0
    top_final_utility: float = 0.0


class EfficientActiveSlamUtility:
    """Single-robot adaptation of the efficient frontier utility layer."""

    def __init__(self, config: GoalSelectorConfig, logger) -> None:
        self.config = config
        self.logger = logger
        self.last_stats = EfficientUtilityStats()

    def filter_clusters(
        self,
        clusters: List[FrontierCluster],
        grid: OccupancyGrid,
        robot_xy: Point2D,
    ) -> List[FrontierCluster]:
        if not self.config.enable_efficient_utility:
            return clusters
        ranked = []
        max_size = max((cluster.size for cluster in clusters), default=1)
        for cluster in clusters:
            gain = compute_local_information_gain(
                grid,
                cluster.centroid_world,
                self.config.information_radius_m,
            )
            size_score = float(cluster.size) / float(max_size)
            distance_score = 1.0 / (1.0 + max(0.0, _distance(robot_xy, cluster.centroid_world)))
            ranked.append((cluster, gain.unknown_ratio + size_score + distance_score))
        ranked.sort(key=lambda item: item[1], reverse=True)
        limited = [cluster for cluster, _ in ranked[: max(1, self.config.max_utility_frontier_clusters)]]
        self.last_stats.top_cluster_count = len(limited)
        return limited

    def rank_candidates(
        self,
        candidates: List[NavigationCandidate],
        grid: OccupancyGrid,
        robot_xy: Point2D,
        last_goal: Optional[Point2D],
        rejected_points: Iterable[Point2D],
    ) -> List[NavigationCandidate]:
        self.last_stats = EfficientUtilityStats(
            utility_candidates_before_limit=len(candidates),
            top_cluster_count=self.last_stats.top_cluster_count,
        )
        if not self.config.enable_efficient_utility or self.config.scoring_mode != "efficient_entropy_utility":
            return candidates
        if not candidates:
            return candidates

        ordered = sorted(candidates, key=lambda candidate: candidate.score, reverse=True)
        if self.config.enable_region_diversity:
            ordered = limit_candidates_by_region(
                ordered,
                self.config.max_candidates_per_region,
                self.config.region_grid_size_m,
            )
        ordered = ordered[: max(1, self.config.max_utility_candidates)]
        scored = [self._score_candidate(candidate, grid, robot_xy, last_goal, rejected_points) for candidate in ordered]
        scored.sort(key=lambda candidate: candidate.utility_score, reverse=True)
        self.last_stats.utility_candidates_after_limit = len(scored)
        if scored:
            selected = scored[0]
            self.last_stats.selected_before_planner_validation = selected
            self.last_stats.top_path_entropy = selected.path_entropy
            self.last_stats.top_information_gain = selected.information_gain
            self.last_stats.top_uncertainty_proxy = selected.uncertainty_proxy
            self.last_stats.top_region_diversity = selected.region_diversity
            self.last_stats.top_final_utility = selected.utility_score
            self.logger.info(
                "Efficient utility: utility_candidates_before_limit=%d "
                "utility_candidates_after_limit=%d top_cluster_count=%d "
                "best_utility=%.3f best_candidate=(%.2f, %.2f) "
                "path_entropy=%.3f information_gain=%.3f uncertainty_proxy=%.3f "
                "region_diversity=%.3f final_utility=%.3f selected_before_planner_validation=(%.2f, %.2f)"
                % (
                    self.last_stats.utility_candidates_before_limit,
                    self.last_stats.utility_candidates_after_limit,
                    self.last_stats.top_cluster_count,
                    selected.utility_score,
                    selected.point_world[0],
                    selected.point_world[1],
                    selected.path_entropy,
                    selected.information_gain,
                    selected.uncertainty_proxy,
                    selected.region_diversity,
                    selected.utility_score,
                    selected.point_world[0],
                    selected.point_world[1],
                )
            )
        return scored

    def _score_candidate(
        self,
        candidate: NavigationCandidate,
        grid: OccupancyGrid,
        robot_xy: Point2D,
        last_goal: Optional[Point2D],
        rejected_points: Iterable[Point2D],
    ) -> NavigationCandidate:
        info = compute_local_information_gain(grid, candidate.point_world, self.config.information_radius_m)
        path = estimate_straight_line_path_entropy(grid, robot_xy, candidate.point_world)
        diversity = region_diversity_score(
            candidate,
            self.config.region_grid_size_m,
            last_goal,
            rejected_points,
            self.config.recent_goal_region_penalty,
            self.config.rejected_region_penalty,
        )
        uncertainty = compute_uncertainty_proxy(
            candidate,
            max(self.config.max_viewpoint_distance_m, self.config.global_reposition_max_distance_m),
            info.unknown_ratio,
            None,
        )
        path_length_norm = min(
            1.0,
            candidate.distance / max(0.1, self.config.global_reposition_max_distance_m),
        )
        path_entropy_norm = _clamp01(path.mean_path_entropy)
        information_norm = _clamp01(info.unknown_ratio + 0.5 * info.local_entropy_mean)
        cost_norm = _clamp01(candidate.cost_penalty)
        utility = (
            self.config.w_information_gain * information_norm
            + self.config.w_path_entropy * path_entropy_norm
            + self.config.w_region_diversity * diversity
            - self.config.w_path_length_penalty * path_length_norm
            - self.config.w_cost_penalty * cost_norm
            - self.config.w_uncertainty_penalty * uncertainty.value
            - self.config.w_goal_switching * 0.0
        )
        return replace(
            candidate,
            score=utility,
            utility_score=utility,
            path_entropy=path.sum_path_entropy,
            mean_path_entropy=path.mean_path_entropy,
            unknown_ratio_along_path=path.unknown_ratio_along_path,
            information_gain=information_norm,
            local_entropy=info.local_entropy_sum,
            uncertainty_proxy=uncertainty.value,
            region_diversity=diversity,
        )


def _distance(a: Point2D, b: Point2D) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return (dx * dx + dy * dy) ** 0.5


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))
