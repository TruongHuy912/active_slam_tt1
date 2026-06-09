import math
from dataclasses import dataclass
from typing import Iterable, List, Optional

from nav_msgs.msg import OccupancyGrid, Path

from bumperbot_active_slam.entropy_utils import (
    bresenham,
    cell_entropy,
    euclidean_distance,
    occupancy_probability,
    world_to_map,
)
from bumperbot_active_slam.models import Point2D


@dataclass(frozen=True)
class PathEntropyResult:
    sum_path_entropy: float = 0.0
    mean_path_entropy: float = 0.0
    unknown_ratio_along_path: float = 0.0
    entropy_sample_count: int = 0


def estimate_straight_line_path_entropy(
    grid: OccupancyGrid,
    start_xy: Point2D,
    goal_xy: Point2D,
) -> PathEntropyResult:
    start = world_to_map(start_xy[0], start_xy[1], grid.info)
    goal = world_to_map(goal_xy[0], goal_xy[1], grid.info)
    if start is None or goal is None:
        return PathEntropyResult()
    return _entropy_for_cells(grid, bresenham(start, goal))


def compute_path_entropy_for_nav_path(
    grid: OccupancyGrid,
    path: Path,
    sample_step_m: float,
) -> PathEntropyResult:
    if path is None or not path.poses:
        return PathEntropyResult()
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    cells = []
    last_xy: Optional[Point2D] = None
    step = max(0.01, sample_step_m)
    for point in points:
        if last_xy is None:
            _append_world_cell(grid, cells, point)
            last_xy = point
            continue
        distance = euclidean_distance(last_xy, point)
        sample_count = max(1, int(math.ceil(distance / step)))
        for index in range(1, sample_count + 1):
            ratio = float(index) / float(sample_count)
            sample = (
                last_xy[0] + (point[0] - last_xy[0]) * ratio,
                last_xy[1] + (point[1] - last_xy[1]) * ratio,
            )
            _append_world_cell(grid, cells, sample)
        last_xy = point
    return _entropy_for_cells(grid, cells)


def _append_world_cell(grid: OccupancyGrid, cells: List[tuple], point_xy: Point2D) -> None:
    cell = world_to_map(point_xy[0], point_xy[1], grid.info)
    if cell is not None and (not cells or cells[-1] != cell):
        cells.append(cell)


def _entropy_for_cells(grid: OccupancyGrid, cells: Iterable[tuple]) -> PathEntropyResult:
    width = int(grid.info.width)
    height = int(grid.info.height)
    if width <= 0 or height <= 0 or len(grid.data) < width * height:
        return PathEntropyResult()
    entropy_sum = 0.0
    unknown_count = 0
    sample_count = 0
    for mx, my in cells:
        if mx < 0 or my < 0 or mx >= width or my >= height:
            continue
        value = int(grid.data[my * width + mx])
        sample_count += 1
        if value < 0:
            unknown_count += 1
        entropy_sum += cell_entropy(occupancy_probability(value))
    if sample_count <= 0:
        return PathEntropyResult()
    return PathEntropyResult(
        sum_path_entropy=entropy_sum,
        mean_path_entropy=entropy_sum / float(sample_count),
        unknown_ratio_along_path=float(unknown_count) / float(sample_count),
        entropy_sample_count=sample_count,
    )
