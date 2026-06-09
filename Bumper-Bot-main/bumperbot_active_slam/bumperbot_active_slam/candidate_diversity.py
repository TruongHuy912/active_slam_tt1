from collections import defaultdict
from typing import List

from bumperbot_active_slam.entropy_utils import euclidean_distance
from bumperbot_active_slam.models import NavigationCandidate


def diverse_candidates(
    candidates: List[NavigationCandidate],
    max_total: int,
    max_per_cluster: int,
    spatial_separation_m: float,
) -> List[NavigationCandidate]:
    selected: List[NavigationCandidate] = []
    per_cluster = defaultdict(int)
    for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
        if len(selected) >= max_total:
            break
        if per_cluster[candidate.cluster_id] >= max_per_cluster:
            continue
        if any(
            euclidean_distance(candidate.point_world, existing.point_world) < spatial_separation_m
            for existing in selected
        ):
            continue
        selected.append(candidate)
        per_cluster[candidate.cluster_id] += 1
    return selected
