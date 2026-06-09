import math
from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from bumperbot_active_slam.entropy_utils import euclidean_distance, world_to_map


CostmapCell = Tuple[int, int]


def world_to_costmap(x: float, y: float, costmap: OccupancyGrid) -> Optional[CostmapCell]:
    return world_to_map(x, y, costmap.info)


def get_cost_at_world(x: float, y: float, costmap: OccupancyGrid) -> Optional[int]:
    cell = world_to_costmap(x, y, costmap)
    if cell is None:
        return None
    mx, my = cell
    index = my * int(costmap.info.width) + mx
    if index < 0 or index >= len(costmap.data):
        return None
    return int(costmap.data[index])


def is_cost_safe(cost: Optional[int], max_allowed_cost: int, reject_unknown_cost: bool) -> bool:
    if cost is None:
        return False
    if cost < 0:
        return not reject_unknown_cost
    return cost <= max_allowed_cost


def check_clearance_radius(
    x: float,
    y: float,
    costmap: OccupancyGrid,
    radius_m: float,
    max_allowed_cost: int,
    reject_unknown_cost: bool,
) -> bool:
    center = world_to_costmap(x, y, costmap)
    if center is None:
        return False

    resolution = float(costmap.info.resolution)
    if resolution <= 0.0:
        return False

    width = int(costmap.info.width)
    height = int(costmap.info.height)
    radius_cells = max(0, int(math.ceil(radius_m / resolution)))
    cx, cy = center

    for my in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
        for mx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
            wx = (mx - cx) * resolution
            wy = (my - cy) * resolution
            if math.hypot(wx, wy) > radius_m:
                continue
            index = my * width + mx
            if index < 0 or index >= len(costmap.data):
                return False
            if not is_cost_safe(int(costmap.data[index]), max_allowed_cost, reject_unknown_cost):
                return False

    return True


def is_pose_safe(
    x: float,
    y: float,
    costmap: OccupancyGrid,
    max_allowed_cost: int,
    reject_unknown_cost: bool,
    safety_radius_m: float,
) -> bool:
    cost = get_cost_at_world(x, y, costmap)
    if not is_cost_safe(cost, max_allowed_cost, reject_unknown_cost):
        return False
    return check_clearance_radius(
        x,
        y,
        costmap,
        safety_radius_m,
        max_allowed_cost,
        reject_unknown_cost,
    )


def summarize_costmap_status(costmap: Optional[OccupancyGrid]) -> str:
    if costmap is None:
        return "missing"
    width = int(costmap.info.width)
    height = int(costmap.info.height)
    if width <= 0 or height <= 0 or len(costmap.data) < width * height:
        return "invalid %dx%d len=%d" % (width, height, len(costmap.data))
    return "frame=%s size=%dx%d res=%.3f" % (
        costmap.header.frame_id,
        width,
        height,
        costmap.info.resolution,
    )


def map_pose_is_free(x: float, y: float, grid: OccupancyGrid, safety_radius_m: float = 0.0) -> bool:
    cell = world_to_map(x, y, grid.info)
    if cell is None:
        return False

    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return False

    width = int(grid.info.width)
    height = int(grid.info.height)
    radius_cells = max(0, int(math.ceil(safety_radius_m / resolution)))
    cx, cy = cell

    for my in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
        for mx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
            if euclidean_distance((mx, my), (cx, cy)) * resolution > safety_radius_m:
                continue
            index = my * width + mx
            if index < 0 or index >= len(grid.data):
                return False
            if int(grid.data[index]) != 0:
                return False
    return True


def max_cost_in_radius(
    x: float,
    y: float,
    costmap: OccupancyGrid,
    radius_m: float,
) -> Optional[int]:
    center = world_to_costmap(x, y, costmap)
    if center is None:
        return None
    resolution = float(costmap.info.resolution)
    if resolution <= 0.0:
        return None
    width = int(costmap.info.width)
    height = int(costmap.info.height)
    radius_cells = max(0, int(math.ceil(radius_m / resolution)))
    cx, cy = center
    max_cost = None
    for my in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
        for mx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
            if euclidean_distance((mx, my), (cx, cy)) * resolution > radius_m:
                continue
            index = my * width + mx
            if index < 0 or index >= len(costmap.data):
                continue
            cost = int(costmap.data[index])
            max_cost = cost if max_cost is None else max(max_cost, cost)
    return max_cost


def nearest_safe_pose_sample(
    x: float,
    y: float,
    costmap: OccupancyGrid,
    radius_m: float,
    sample_count: int,
    max_allowed_cost: int,
    reject_unknown_cost: bool,
    safety_radius_m: float,
) -> Optional[Tuple[float, float]]:
    best = None
    best_cost = None
    for index in range(max(1, sample_count)):
        angle = 2.0 * math.pi * float(index) / float(max(1, sample_count))
        px = x + math.cos(angle) * radius_m
        py = y + math.sin(angle) * radius_m
        if not is_pose_safe(px, py, costmap, max_allowed_cost, reject_unknown_cost, safety_radius_m):
            continue
        cost = get_cost_at_world(px, py, costmap)
        if best is None or (cost is not None and (best_cost is None or cost < best_cost)):
            best = (px, py)
            best_cost = cost
    return best
