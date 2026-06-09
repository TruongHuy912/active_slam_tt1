import math
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Tuple

from bumperbot_active_slam.models import NavigationCandidate, Point2D


RegionKey = Tuple[int, int]


def region_key(point_xy: Point2D, region_grid_size_m: float) -> RegionKey:
    size = max(0.1, region_grid_size_m)
    return int(math.floor(point_xy[0] / size)), int(math.floor(point_xy[1] / size))


def limit_candidates_by_region(
    candidates: Iterable[NavigationCandidate],
    max_candidates_per_region: int,
    region_grid_size_m: float,
) -> List[NavigationCandidate]:
    per_region: Dict[RegionKey, int] = defaultdict(int)
    limited: List[NavigationCandidate] = []
    for candidate in candidates:
        key = region_key(candidate.point_world, region_grid_size_m)
        if per_region[key] >= max(1, max_candidates_per_region):
            continue
        per_region[key] += 1
        limited.append(candidate)
    return limited


def region_diversity_score(
    candidate: NavigationCandidate,
    region_grid_size_m: float,
    recent_goal: Optional[Point2D],
    rejected_points: Iterable[Point2D],
    recent_goal_region_penalty: float,
    rejected_region_penalty: float,
) -> float:
    score = 1.0
    key = region_key(candidate.point_world, region_grid_size_m)
    if recent_goal is not None and region_key(recent_goal, region_grid_size_m) == key:
        score -= max(0.0, min(1.0, recent_goal_region_penalty))
    rejected_hits = 0
    for point in rejected_points:
        if region_key(point, region_grid_size_m) == key:
            rejected_hits += 1
    if rejected_hits:
        score -= max(0.0, min(1.0, rejected_region_penalty)) * min(1.0, rejected_hits / 3.0)
    return max(0.0, min(1.0, score))
