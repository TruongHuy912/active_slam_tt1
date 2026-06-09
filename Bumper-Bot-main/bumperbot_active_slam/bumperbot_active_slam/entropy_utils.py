import math
from typing import Iterable, List, Optional, Sequence, Tuple


MapCell = Tuple[int, int]
WorldPoint = Tuple[float, float]


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_map(x: float, y: float, map_info) -> Optional[MapCell]:
    """Convert world coordinates to an OccupancyGrid cell index."""
    resolution = map_info.resolution
    origin = map_info.origin.position
    yaw = _yaw_from_quaternion(map_info.origin.orientation)

    dx = x - origin.x
    dy = y - origin.y
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    local_x = (dx * cos_yaw) - (dy * sin_yaw)
    local_y = (dx * sin_yaw) + (dy * cos_yaw)

    mx = int(math.floor(local_x / resolution))
    my = int(math.floor(local_y / resolution))
    if mx < 0 or my < 0 or mx >= map_info.width or my >= map_info.height:
        return None
    return mx, my


def map_to_world(mx: int, my: int, map_info) -> WorldPoint:
    """Convert an OccupancyGrid cell index to the world coordinate at cell center."""
    resolution = map_info.resolution
    origin = map_info.origin.position
    yaw = _yaw_from_quaternion(map_info.origin.orientation)

    local_x = (float(mx) + 0.5) * resolution
    local_y = (float(my) + 0.5) * resolution
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = origin.x + (local_x * cos_yaw) - (local_y * sin_yaw)
    world_y = origin.y + (local_x * sin_yaw) + (local_y * cos_yaw)
    return world_x, world_y


def bresenham(start: MapCell, end: MapCell) -> List[MapCell]:
    """Return inclusive integer grid cells on a line from start to end."""
    x0, y0 = start
    x1, y1 = end
    cells: List[MapCell] = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return cells


def occupancy_probability(value: int, unknown_probability: float = 0.5) -> float:
    """Map OccupancyGrid values to occupancy probability."""
    if value < 0:
        return _clamp_probability(unknown_probability)
    return _clamp_probability(float(value) / 100.0)


def cell_entropy(probability: float) -> float:
    """Binary entropy in bits for a single occupancy probability."""
    p = _clamp_probability(probability)
    if p <= 0.0 or p >= 1.0:
        return 0.0
    return -p * math.log2(p) - (1.0 - p) * math.log2(1.0 - p)


def compute_path_entropy(
    data: Sequence[int],
    width: int,
    height: int,
    start: MapCell,
    end: MapCell,
    unknown_probability: float = 0.5,
) -> float:
    """Sum cell entropy along a grid line."""
    entropy = 0.0
    for mx, my in bresenham(start, end):
        if mx < 0 or my < 0 or mx >= width or my >= height:
            continue
        index = my * width + mx
        entropy += cell_entropy(occupancy_probability(data[index], unknown_probability))
    return entropy


def euclidean_distance(a: Iterable[float], b: Iterable[float]) -> float:
    ax, ay = list(a)[:2]
    bx, by = list(b)[:2]
    return math.hypot(ax - bx, ay - by)


def _clamp_probability(value: float) -> float:
    return max(0.0, min(1.0, value))
