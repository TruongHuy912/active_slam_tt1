import math
from dataclasses import dataclass

from nav_msgs.msg import OccupancyGrid

from bumperbot_active_slam.entropy_utils import (
    cell_entropy,
    euclidean_distance,
    occupancy_probability,
    world_to_map,
)
from bumperbot_active_slam.models import Point2D


@dataclass(frozen=True)
class InformationGainResult:
    unknown_count: int = 0
    unknown_ratio: float = 0.0
    local_entropy_sum: float = 0.0
    local_entropy_mean: float = 0.0
    sample_count: int = 0


def compute_local_information_gain(
    grid: OccupancyGrid,
    point_xy: Point2D,
    radius_m: float,
) -> InformationGainResult:
    width = int(grid.info.width)
    height = int(grid.info.height)
    if width <= 0 or height <= 0 or len(grid.data) < width * height:
        return InformationGainResult()

    center = world_to_map(point_xy[0], point_xy[1], grid.info)
    if center is None:
        return InformationGainResult()

    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return InformationGainResult()

    radius_cells = max(1, int(math.ceil(radius_m / resolution)))
    cx, cy = center
    unknown_count = 0
    entropy_sum = 0.0
    sample_count = 0
    for my in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
        for mx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
            if euclidean_distance((mx, my), (cx, cy)) * resolution > radius_m:
                continue
            value = int(grid.data[my * width + mx])
            sample_count += 1
            if value < 0:
                unknown_count += 1
            entropy_sum += cell_entropy(occupancy_probability(value))

    if sample_count <= 0:
        return InformationGainResult()
    return InformationGainResult(
        unknown_count=unknown_count,
        unknown_ratio=float(unknown_count) / float(sample_count),
        local_entropy_sum=entropy_sum,
        local_entropy_mean=entropy_sum / float(sample_count),
        sample_count=sample_count,
    )
