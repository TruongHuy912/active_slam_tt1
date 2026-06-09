from collections import deque
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Set, Tuple

from nav_msgs.msg import OccupancyGrid

from bumperbot_active_slam.entropy_utils import map_to_world


MapCell = Tuple[int, int]


@dataclass(frozen=True)
class FrontierCluster:
    id: int
    cells: List[MapCell]
    size: int
    centroid_map: Tuple[float, float]
    centroid_world: Tuple[float, float]


@dataclass(frozen=True)
class FrontierDetection:
    frontier_cell_count: int
    clusters: List[FrontierCluster]


def detect_frontiers(
    grid: OccupancyGrid,
    connectivity: int = 8,
    min_cluster_size: int = 3,
) -> List[FrontierCluster]:
    """Detect frontier clusters in an OccupancyGrid.

    A frontier cell is a free cell adjacent to at least one unknown cell.
    """
    return detect_frontiers_with_stats(grid, connectivity, min_cluster_size).clusters


def detect_frontiers_with_stats(
    grid: OccupancyGrid,
    connectivity: int = 8,
    min_cluster_size: int = 3,
) -> FrontierDetection:
    """Detect frontier clusters and return raw frontier-cell statistics."""
    if connectivity not in (4, 8):
        raise ValueError("connectivity must be 4 or 8")

    width = int(grid.info.width)
    height = int(grid.info.height)
    data = grid.data
    if width <= 0 or height <= 0 or len(data) < width * height:
        return FrontierDetection(frontier_cell_count=0, clusters=[])

    frontier_cells = _find_frontier_cells(data, width, height, connectivity)
    clusters = _cluster_frontier_cells(
        frontier_cells,
        width,
        height,
        connectivity,
        max(1, int(min_cluster_size)),
        grid.info,
    )
    return FrontierDetection(frontier_cell_count=len(frontier_cells), clusters=clusters)


def _find_frontier_cells(
    data: Sequence[int],
    width: int,
    height: int,
    connectivity: int,
) -> Set[MapCell]:
    frontier_cells: Set[MapCell] = set()
    for my in range(height):
        for mx in range(width):
            if not _is_free(data, width, mx, my):
                continue
            if any(_is_unknown(data, width, nx, ny) for nx, ny in _neighbors(mx, my, width, height, connectivity)):
                frontier_cells.add((mx, my))
    return frontier_cells


def _cluster_frontier_cells(
    frontier_cells: Set[MapCell],
    width: int,
    height: int,
    connectivity: int,
    min_cluster_size: int,
    map_info,
) -> List[FrontierCluster]:
    clusters: List[FrontierCluster] = []
    visited: Set[MapCell] = set()

    for start in sorted(frontier_cells, key=lambda cell: (cell[1], cell[0])):
        if start in visited:
            continue

        cells: List[MapCell] = []
        queue = deque([start])
        visited.add(start)

        while queue:
            cell = queue.popleft()
            cells.append(cell)
            for neighbor in _neighbors(cell[0], cell[1], width, height, connectivity):
                if neighbor not in frontier_cells or neighbor in visited:
                    continue
                visited.add(neighbor)
                queue.append(neighbor)

        if len(cells) < min_cluster_size:
            continue

        centroid_map = _centroid(cells)
        centroid_world = _centroid_world(cells, map_info)
        clusters.append(
            FrontierCluster(
                id=len(clusters),
                cells=cells,
                size=len(cells),
                centroid_map=centroid_map,
                centroid_world=centroid_world,
            )
        )

    return clusters


def _neighbors(
    mx: int,
    my: int,
    width: int,
    height: int,
    connectivity: int,
) -> Iterable[MapCell]:
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if connectivity == 8:
        offsets.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

    for dx, dy in offsets:
        nx = mx + dx
        ny = my + dy
        if 0 <= nx < width and 0 <= ny < height:
            yield nx, ny


def _is_free(data: Sequence[int], width: int, mx: int, my: int) -> bool:
    return data[my * width + mx] == 0


def _is_unknown(data: Sequence[int], width: int, mx: int, my: int) -> bool:
    return data[my * width + mx] < 0


def _centroid(cells: Sequence[MapCell]) -> Tuple[float, float]:
    count = float(len(cells))
    return (
        sum(cell[0] for cell in cells) / count,
        sum(cell[1] for cell in cells) / count,
    )


def _centroid_world(cells: Sequence[MapCell], map_info) -> Tuple[float, float]:
    count = float(len(cells))
    world_points = [map_to_world(mx, my, map_info) for mx, my in cells]
    return (
        sum(point[0] for point in world_points) / count,
        sum(point[1] for point in world_points) / count,
    )
