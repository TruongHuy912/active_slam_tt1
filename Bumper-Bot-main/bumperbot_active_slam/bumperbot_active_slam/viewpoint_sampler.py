import math
from typing import List, Tuple


Point2D = Tuple[float, float]
Cell = Tuple[int, int]


def sample_cluster_cells(cells: List[Cell], max_cells: int) -> List[Cell]:
    if len(cells) <= max_cells:
        return list(cells)
    if max_cells <= 1:
        return [cells[len(cells) // 2]]

    last_index = len(cells) - 1
    sampled = []
    seen = set()
    for sample_index in range(max_cells):
        cell_index = round(sample_index * last_index / (max_cells - 1))
        cell = cells[int(cell_index)]
        if cell in seen:
            continue
        seen.add(cell)
        sampled.append(cell)
    return sampled


def sample_viewpoints_around_frontier(
    frontier_world: Point2D,
    robot_xy: Point2D,
    radius_m: float,
    num_samples: int,
    safe_viewpoint_mode: bool,
) -> List[Point2D]:
    if not safe_viewpoint_mode:
        return [frontier_world]

    viewpoints = []
    fx, fy = frontier_world
    rx, ry = robot_xy
    angle_to_robot = math.atan2(ry - fy, rx - fx)
    viewpoints.append((
        fx + math.cos(angle_to_robot) * radius_m,
        fy + math.sin(angle_to_robot) * radius_m,
    ))
    for index in range(max(1, num_samples)):
        angle = 2.0 * math.pi * float(index) / float(max(1, num_samples))
        viewpoints.append((fx + math.cos(angle) * radius_m, fy + math.sin(angle) * radius_m))
    return viewpoints


def sample_global_reposition_points(
    robot_xy: Point2D,
    target_world: Point2D,
    step_m: float,
    sample_count: int,
) -> List[Point2D]:
    rx, ry = robot_xy
    tx, ty = target_world
    dx = tx - rx
    dy = ty - ry
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        return []

    base_angle = math.atan2(dy, dx)
    base_step = min(max(step_m, 0.1), distance)
    points = [(rx + math.cos(base_angle) * base_step, ry + math.sin(base_angle) * base_step)]

    # A small angular fan lets Nav2 pick a nearby safe corridor without jumping
    # straight into unknown frontier cells.
    total = max(1, sample_count)
    fan_width = math.radians(75.0)
    for index in range(total):
        if total == 1:
            angle = base_angle
        else:
            ratio = float(index) / float(total - 1)
            angle = base_angle - 0.5 * fan_width + ratio * fan_width
        points.append((rx + math.cos(angle) * base_step, ry + math.sin(angle) * base_step))
    return points


def sample_frontier_bridge_points(
    robot_xy: Point2D,
    target_world: Point2D,
    step_distances_m: List[float],
    lateral_offsets_m: List[float],
) -> List[Tuple[Point2D, float, float]]:
    rx, ry = robot_xy
    tx, ty = target_world
    dx = tx - rx
    dy = ty - ry
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        return []

    ux = dx / distance
    uy = dy / distance
    px = -uy
    py = ux
    points = []
    seen = set()
    for step_m in step_distances_m:
        step = min(max(float(step_m), 0.1), distance)
        for lateral_m in lateral_offsets_m:
            lateral = float(lateral_m)
            point = (
                rx + ux * step + px * lateral,
                ry + uy * step + py * lateral,
            )
            key = (round(point[0], 3), round(point[1], 3))
            if key in seen:
                continue
            seen.add(key)
            points.append((point, step, lateral))
    return points
