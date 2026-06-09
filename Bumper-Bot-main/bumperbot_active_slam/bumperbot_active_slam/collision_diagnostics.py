from typing import Optional

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def summarize_scan(scan: Optional[LaserScan]) -> str:
    if scan is None:
        return "scan=missing"
    finite_ranges = [value for value in scan.ranges if scan.range_min <= value <= scan.range_max]
    if not finite_ranges:
        return "scan frame=%s finite_ranges=0" % scan.header.frame_id
    return "scan frame=%s finite_ranges=%d min_range=%.2f max_range=%.2f" % (
        scan.header.frame_id,
        len(finite_ranges),
        min(finite_ranges),
        max(finite_ranges),
    )


def summarize_occupancy_grid(name: str, grid: Optional[OccupancyGrid]) -> str:
    if grid is None:
        return "%s=missing" % name
    width = int(grid.info.width)
    height = int(grid.info.height)
    if width <= 0 or height <= 0 or len(grid.data) < width * height:
        return "%s=invalid %dx%d len=%d" % (name, width, height, len(grid.data))
    occupied = 0
    unknown = 0
    for value in grid.data:
        cell = int(value)
        if cell < 0:
            unknown += 1
        elif cell >= 70:
            occupied += 1
    return "%s frame=%s size=%dx%d occupied_or_high=%d unknown=%d" % (
        name,
        grid.header.frame_id,
        width,
        height,
        occupied,
        unknown,
    )
