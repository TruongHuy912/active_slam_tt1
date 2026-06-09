from typing import Optional, Tuple

from bumperbot_active_slam.entropy_utils import euclidean_distance


class ProgressMonitor:
    def __init__(
        self,
        node,
        check_period_sec: float,
        min_progress_distance_m: float,
        stuck_timeout_sec: float,
    ) -> None:
        self.node = node
        self.check_period_sec = check_period_sec
        self.min_progress_distance_m = min_progress_distance_m
        self.stuck_timeout_sec = stuck_timeout_sec
        self._last_pose: Optional[Tuple[float, float]] = None
        self._last_check_time = None
        self._stuck_start_time = None

    def reset(self, robot_xy: Optional[Tuple[float, float]] = None) -> None:
        self._last_pose = robot_xy
        self._last_check_time = self.node.get_clock().now()
        self._stuck_start_time = None

    def update(self, robot_xy: Optional[Tuple[float, float]]) -> bool:
        if robot_xy is None:
            return False
        now = self.node.get_clock().now()
        if self._last_pose is None or self._last_check_time is None:
            self.reset(robot_xy)
            return False
        elapsed = (now - self._last_check_time).nanoseconds / 1e9
        if elapsed < self.check_period_sec:
            return False
        moved = euclidean_distance(robot_xy, self._last_pose)
        self.node.get_logger().info(
            "progress_monitor: moved=%.3f in %.1f sec" % (moved, elapsed)
        )
        self._last_pose = robot_xy
        self._last_check_time = now
        if moved >= self.min_progress_distance_m:
            self._stuck_start_time = None
            return False
        if self._stuck_start_time is None:
            self._stuck_start_time = now
            return False
        stuck_elapsed = (now - self._stuck_start_time).nanoseconds / 1e9
        return stuck_elapsed >= self.stuck_timeout_sec
