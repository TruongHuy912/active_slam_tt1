import math
from dataclasses import dataclass
from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid, Path

from bumperbot_active_slam.costmap_utils import get_cost_at_world, is_cost_safe


@dataclass(frozen=True)
class PathSafetyResult:
    safe: bool
    length_m: float = 0.0
    max_cost: Optional[int] = None
    reject_reason: str = "none"
    samples_checked: int = 0
    first_reject_world: Optional[Tuple[float, float]] = None
    first_reject_map_cell: Optional[Tuple[int, int]] = None
    max_cost_near_path: Optional[int] = None
    detail: str = "none"
    ignore_start_radius_m: float = 0.0
    ignored_start_samples: int = 0
    first_checked_world: Optional[Tuple[float, float]] = None
    robot_cost: Optional[int] = None
    goal_cost: Optional[int] = None


def compute_path_length(path: Path) -> float:
    if len(path.poses) < 2:
        return 0.0
    length = 0.0
    last = path.poses[0].pose.position
    for pose_stamped in path.poses[1:]:
        point = pose_stamped.pose.position
        length += math.hypot(point.x - last.x, point.y - last.y)
        last = point
    return length


def validate_path_safety(
    path: Path,
    costmap: Optional[OccupancyGrid],
    min_length_m: float,
    max_length_m: float,
    max_path_cost: int,
    reject_unknown: bool,
    path_check_step_m: float,
    clearance_radius_m: float,
    ignore_start_radius_m: float = 0.0,
    path_clearance_max_near_cost: int = 70,
    path_clearance_lethal_cost: int = 90,
    allow_low_inflation_near_path: bool = True,
    low_inflation_cost_threshold: int = 40,
) -> PathSafetyResult:
    if path is None or not path.poses:
        return PathSafetyResult(False, reject_reason="no_path")

    length = compute_path_length(path)
    if length < min_length_m:
        return PathSafetyResult(False, length_m=length, reject_reason="path_too_short")
    if length > max_length_m:
        return PathSafetyResult(False, length_m=length, reject_reason="path_too_long")
    if costmap is None or int(costmap.info.width) <= 0 or int(costmap.info.height) <= 0:
        return PathSafetyResult(True, length_m=length, reject_reason="costmap_unavailable")

    max_seen_cost: Optional[int] = None
    samples_checked = 0
    ignored_start_samples = 0
    first_checked_world: Optional[Tuple[float, float]] = None
    start_point = path.poses[0].pose.position
    goal_point = path.poses[-1].pose.position
    robot_cost = get_cost_at_world(start_point.x, start_point.y, costmap)
    goal_cost = get_cost_at_world(goal_point.x, goal_point.y, costmap)
    step = max(0.01, path_check_step_m)
    for index, pose_stamped in enumerate(path.poses):
        point = pose_stamped.pose.position
        if _within_ignore_start_radius(point.x, point.y, start_point.x, start_point.y, ignore_start_radius_m):
            ignored_start_samples += 1
            continue
        if first_checked_world is None:
            first_checked_world = (point.x, point.y)
        result = _validate_sample(
            point.x,
            point.y,
            costmap,
            max_path_cost,
            reject_unknown,
            clearance_radius_m,
            path_clearance_max_near_cost,
            path_clearance_lethal_cost,
            allow_low_inflation_near_path,
            low_inflation_cost_threshold,
            start_point.x,
            start_point.y,
            ignore_start_radius_m,
        )
        samples_checked += 1
        if not result.safe:
            return PathSafetyResult(
                False,
                length_m=length,
                max_cost=result.max_cost,
                reject_reason=result.reject_reason,
                samples_checked=samples_checked,
                first_reject_world=result.first_reject_world,
                first_reject_map_cell=result.first_reject_map_cell,
                max_cost_near_path=result.max_cost_near_path,
                detail=result.detail,
                ignore_start_radius_m=ignore_start_radius_m,
                ignored_start_samples=ignored_start_samples,
                first_checked_world=first_checked_world,
                robot_cost=robot_cost,
                goal_cost=goal_cost,
            )
        max_seen_cost = _combine_cost(max_seen_cost, result.max_cost)

        if index == 0:
            continue
        last = path.poses[index - 1].pose.position
        segment_length = math.hypot(point.x - last.x, point.y - last.y)
        if segment_length <= step:
            continue
        sample_count = int(math.ceil(segment_length / step))
        for sample_index in range(1, sample_count):
            ratio = float(sample_index) / float(sample_count)
            x = last.x + (point.x - last.x) * ratio
            y = last.y + (point.y - last.y) * ratio
            if _within_ignore_start_radius(x, y, start_point.x, start_point.y, ignore_start_radius_m):
                ignored_start_samples += 1
                continue
            if first_checked_world is None:
                first_checked_world = (x, y)
            result = _validate_sample(
                x,
                y,
                costmap,
                max_path_cost,
                reject_unknown,
                clearance_radius_m,
                path_clearance_max_near_cost,
                path_clearance_lethal_cost,
                allow_low_inflation_near_path,
                low_inflation_cost_threshold,
                start_point.x,
                start_point.y,
                ignore_start_radius_m,
            )
            samples_checked += 1
            if not result.safe:
                return PathSafetyResult(
                    False,
                    length_m=length,
                    max_cost=result.max_cost,
                    reject_reason=result.reject_reason,
                    samples_checked=samples_checked,
                    first_reject_world=result.first_reject_world,
                    first_reject_map_cell=result.first_reject_map_cell,
                    max_cost_near_path=result.max_cost_near_path,
                    detail=result.detail,
                    ignore_start_radius_m=ignore_start_radius_m,
                    ignored_start_samples=ignored_start_samples,
                    first_checked_world=first_checked_world,
                    robot_cost=robot_cost,
                    goal_cost=goal_cost,
                )
            max_seen_cost = _combine_cost(max_seen_cost, result.max_cost)

    return PathSafetyResult(
        True,
        length_m=length,
        max_cost=max_seen_cost,
        samples_checked=samples_checked,
        ignore_start_radius_m=ignore_start_radius_m,
        ignored_start_samples=ignored_start_samples,
        first_checked_world=first_checked_world,
        robot_cost=robot_cost,
        goal_cost=goal_cost,
    )


def _within_ignore_start_radius(
    x: float,
    y: float,
    start_x: float,
    start_y: float,
    ignore_start_radius_m: float,
) -> bool:
    return ignore_start_radius_m > 0.0 and math.hypot(x - start_x, y - start_y) <= ignore_start_radius_m


def _validate_sample(
    x: float,
    y: float,
    costmap: OccupancyGrid,
    max_path_cost: int,
    reject_unknown: bool,
    clearance_radius_m: float,
    path_clearance_max_near_cost: int,
    path_clearance_lethal_cost: int,
    allow_low_inflation_near_path: bool,
    low_inflation_cost_threshold: int,
    start_x: float,
    start_y: float,
    ignore_start_radius_m: float,
) -> PathSafetyResult:
    cost = get_cost_at_world(x, y, costmap)
    cell = _world_to_cell(x, y, costmap)
    if cost is None:
        return PathSafetyResult(
            False,
            max_cost=cost,
            reject_reason="path_outside_costmap",
            first_reject_world=(x, y),
            first_reject_map_cell=cell,
            detail="out_of_bounds",
        )
    if cost < 0 and reject_unknown:
        return PathSafetyResult(
            False,
            max_cost=cost,
            reject_reason="path_unknown",
            first_reject_world=(x, y),
            first_reject_map_cell=cell,
            detail="unknown",
        )
    if not is_cost_safe(cost, max_path_cost, reject_unknown):
        detail = "obstacle_cost" if cost >= 100 else "inflated_cost"
        return PathSafetyResult(
            False,
            max_cost=cost,
            reject_reason="path_cost",
            first_reject_world=(x, y),
            first_reject_map_cell=cell,
            detail=detail,
        )
    clearance = _clearance_details(
        x,
        y,
        costmap,
        clearance_radius_m,
    )
    if clearance.max_cost is not None and clearance.max_cost >= path_clearance_lethal_cost:
        reject_reason = _clearance_reject_reason(x, y, start_x, start_y, ignore_start_radius_m, clearance_radius_m)
        return PathSafetyResult(
            False,
            max_cost=cost,
            reject_reason=reject_reason,
            first_reject_world=(x, y),
            first_reject_map_cell=cell,
            max_cost_near_path=clearance.max_cost,
            detail="lethal_near_path",
        )
    if clearance.max_cost is not None and clearance.max_cost > path_clearance_max_near_cost:
        reject_reason = _clearance_reject_reason(x, y, start_x, start_y, ignore_start_radius_m, clearance_radius_m)
        return PathSafetyResult(
            False,
            max_cost=cost,
            reject_reason=reject_reason,
            first_reject_world=(x, y),
            first_reject_map_cell=cell,
            max_cost_near_path=clearance.max_cost,
            detail="clearance_radius_hit",
        )
    if (
        not allow_low_inflation_near_path
        and clearance.max_cost is not None
        and clearance.max_cost > low_inflation_cost_threshold
    ):
        reject_reason = _clearance_reject_reason(x, y, start_x, start_y, ignore_start_radius_m, clearance_radius_m)
        return PathSafetyResult(
            False,
            max_cost=cost,
            reject_reason=reject_reason,
            first_reject_world=(x, y),
            first_reject_map_cell=cell,
            max_cost_near_path=clearance.max_cost,
            detail="inflation_near_path",
        )
    return PathSafetyResult(True, max_cost=cost, max_cost_near_path=clearance.max_cost)


@dataclass(frozen=True)
class _ClearanceDetails:
    max_cost: Optional[int]


def _clearance_details(
    x: float,
    y: float,
    costmap: OccupancyGrid,
    radius_m: float,
) -> _ClearanceDetails:
    max_cost = _max_cost_near(x, y, costmap, radius_m)
    return _ClearanceDetails(max_cost=max_cost)


def _clearance_reject_reason(
    x: float,
    y: float,
    start_x: float,
    start_y: float,
    ignore_start_radius_m: float,
    clearance_radius_m: float,
) -> str:
    start_radius = max(ignore_start_radius_m, clearance_radius_m)
    if start_radius > 0.0 and math.hypot(x - start_x, y - start_y) <= start_radius:
        return "start_clearance_reject"
    return "path_clearance"


def _combine_cost(current: Optional[int], new_cost: Optional[int]) -> Optional[int]:
    if new_cost is None:
        return current
    if current is None:
        return new_cost
    return max(current, new_cost)


def _world_to_cell(x: float, y: float, costmap: OccupancyGrid) -> Optional[Tuple[int, int]]:
    resolution = float(costmap.info.resolution)
    if resolution <= 0.0:
        return None
    origin = costmap.info.origin.position
    mx = int((x - origin.x) / resolution)
    my = int((y - origin.y) / resolution)
    if mx < 0 or my < 0 or mx >= int(costmap.info.width) or my >= int(costmap.info.height):
        return None
    return mx, my


def _max_cost_near(x: float, y: float, costmap: OccupancyGrid, radius_m: float) -> Optional[int]:
    center = _world_to_cell(x, y, costmap)
    if center is None:
        return None
    resolution = float(costmap.info.resolution)
    radius_cells = max(0, int(math.ceil(radius_m / resolution)))
    width = int(costmap.info.width)
    height = int(costmap.info.height)
    cx, cy = center
    max_cost = None
    for my in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
        for mx in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
            if math.hypot((mx - cx) * resolution, (my - cy) * resolution) > radius_m:
                continue
            index = my * width + mx
            if index < 0 or index >= len(costmap.data):
                continue
            cost = int(costmap.data[index])
            max_cost = cost if max_cost is None else max(max_cost, cost)
    return max_cost
