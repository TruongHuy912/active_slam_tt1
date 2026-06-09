import math
from typing import List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from bumperbot_active_slam.costmap_utils import (
    get_cost_at_world,
    is_pose_safe,
    map_pose_is_free,
    max_cost_in_radius,
)
from bumperbot_active_slam.entropy_utils import euclidean_distance
from bumperbot_active_slam.models import HighCostEscapeConfig, NavigationCandidate, Point2D


class HighCostEscapePolicy:
    """Generate short lower-cost escape candidates when the robot starts in inflated cost."""

    def __init__(self, config: HighCostEscapeConfig, logger) -> None:
        self.config = config
        self.logger = logger
        self.last_robot_cost: Optional[int] = None
        self.last_goal_cost: Optional[int] = None
        self.last_active = False
        self.last_candidates: List[NavigationCandidate] = []
        self.last_rejected: List[Point2D] = []
        self.last_skip_reason = "none"

    def select(
        self,
        robot_xy: Point2D,
        grid: Optional[OccupancyGrid],
        costmap: Optional[OccupancyGrid],
    ) -> Optional[NavigationCandidate]:
        self.last_active = False
        self.last_candidates = []
        self.last_rejected = []
        self.last_skip_reason = "none"
        self.last_robot_cost = None
        self.last_goal_cost = None

        if not self.config.enabled or costmap is None:
            return None

        robot_cost = max_cost_in_radius(robot_xy[0], robot_xy[1], costmap, 0.20)
        self.last_robot_cost = robot_cost
        self.logger.info(
            "costmap_robot_status: global_max_cost_near_robot=%s threshold=%d"
            % ("unknown" if robot_cost is None else str(robot_cost), self.config.robot_cost_threshold)
        )
        if robot_cost is None or robot_cost < self.config.robot_cost_threshold:
            return None

        self.last_active = True
        candidates = self._sample_escape_candidates(robot_xy, grid, costmap, robot_cost)
        self.last_candidates = candidates
        if not candidates:
            self.last_skip_reason = "high_cost_escape_failed: no lower-cost candidate"
            self.logger.warn(
                "high_cost_escape_failed: no lower-cost candidate robot_cost=%d rejected=%d"
                % (robot_cost, len(self.last_rejected))
            )
            return None

        selected = max(candidates, key=lambda item: item.score)
        self.last_goal_cost = selected.cost
        self.logger.warn(
            "high_cost_escape: robot_cost=%d goal_cost=%s selected_escape=(%.2f, %.2f) "
            "distance=%.2f candidates=%d"
            % (
                robot_cost,
                "unknown" if selected.cost is None else str(selected.cost),
                selected.point_world[0],
                selected.point_world[1],
                selected.distance,
                len(candidates),
            )
        )
        return selected

    def _sample_escape_candidates(
        self,
        robot_xy: Point2D,
        grid: Optional[OccupancyGrid],
        costmap: OccupancyGrid,
        robot_cost: int,
    ) -> List[NavigationCandidate]:
        candidates: List[NavigationCandidate] = []
        max_distance = max(0.2, self.config.max_goal_distance_m)
        radii = sorted(set([
            min(max_distance, max(0.3, self.config.sample_radius_m * 0.5)),
            min(max_distance, max(0.5, self.config.sample_radius_m)),
            max_distance,
        ]))
        attempts = max(1, min(self.config.sample_count, self.config.max_attempts_per_cycle))
        angle_slots = max(1, int(math.ceil(float(attempts) / float(max(1, len(radii))))))
        for attempt in range(attempts):
            radius = radii[attempt % len(radii)]
            angle_index = int(attempt / max(1, len(radii)))
            angle = 2.0 * math.pi * float(angle_index) / float(angle_slots)
            point = (
                robot_xy[0] + math.cos(angle) * radius,
                robot_xy[1] + math.sin(angle) * radius,
            )
            distance = euclidean_distance(robot_xy, point)
            if distance > max_distance:
                self.last_rejected.append(point)
                continue
            if grid is not None and not map_pose_is_free(point[0], point[1], grid, safety_radius_m=0.05):
                self.last_rejected.append(point)
                continue
            goal_cost = get_cost_at_world(point[0], point[1], costmap)
            if goal_cost is None:
                self.last_rejected.append(point)
                continue
            if self.config.require_cost_decrease and goal_cost > robot_cost - self.config.min_cost_drop:
                self.last_rejected.append(point)
                continue
            if not is_pose_safe(
                point[0],
                point[1],
                costmap,
                self.config.max_allowed_cost,
                self.config.reject_unknown_cost,
                self.config.safety_radius_m,
            ):
                self.last_rejected.append(point)
                continue
            cost_drop = max(0, robot_cost - goal_cost)
            candidates.append(
                NavigationCandidate(
                    cluster_id=-2,
                    cluster_size=1,
                    point_world=point,
                    point_map=(-1.0, -1.0),
                    distance=distance,
                    source="high_cost_escape",
                    frontier_world=point,
                    score=1000.0 + float(cost_drop) - distance,
                    information_gain=0.0,
                    cost=goal_cost,
                    cost_penalty=0.0,
                    safety_radius_used=self.config.safety_radius_m,
                )
            )
        return candidates
