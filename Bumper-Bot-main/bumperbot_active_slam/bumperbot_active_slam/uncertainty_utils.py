from dataclasses import dataclass
from typing import Optional

from bumperbot_active_slam.models import NavigationCandidate


@dataclass(frozen=True)
class UncertaintyProxyResult:
    value: float = 0.0
    distance_component: float = 0.0
    unknown_component: float = 0.0
    map_staleness_component: float = 0.0


def compute_uncertainty_proxy(
    candidate: NavigationCandidate,
    max_distance_m: float,
    near_unknown_ratio: float,
    map_age_sec: Optional[float] = None,
) -> UncertaintyProxyResult:
    distance_component = min(1.0, candidate.distance / max(0.1, max_distance_m))
    unknown_component = max(0.0, min(1.0, near_unknown_ratio))
    map_staleness_component = 0.0
    if map_age_sec is not None:
        map_staleness_component = min(1.0, max(0.0, map_age_sec) / 10.0)
    value = min(
        1.0,
        0.45 * distance_component
        + 0.45 * unknown_component
        + 0.10 * map_staleness_component,
    )
    return UncertaintyProxyResult(
        value=value,
        distance_component=distance_component,
        unknown_component=unknown_component,
        map_staleness_component=map_staleness_component,
    )
