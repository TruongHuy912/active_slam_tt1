from collections import Counter, defaultdict
from typing import Dict, List, Tuple

from bumperbot_active_slam.entropy_utils import euclidean_distance
from bumperbot_active_slam.models import NavigationCandidate, Point2D


class PlannerRejectCache:
    def __init__(
        self,
        node,
        radius_m: float,
        timeout_sec: float,
        cluster_fail_threshold: int,
        cluster_timeout_sec: float,
        no_path_radius_m: float = 1.0,
        no_path_timeout_sec: float = 60.0,
        no_path_cluster_fail_threshold: int = 2,
        no_path_cluster_timeout_sec: float = 90.0,
        cache_reasons=None,
        log_individual_hits: bool = False,
        max_hit_logs_per_cycle: int = 5,
    ) -> None:
        self.node = node
        self.radius_m = radius_m
        self.timeout_sec = timeout_sec
        self.cluster_fail_threshold = max(1, cluster_fail_threshold)
        self.cluster_timeout_sec = cluster_timeout_sec
        self.no_path_radius_m = no_path_radius_m
        self.no_path_timeout_sec = no_path_timeout_sec
        self.no_path_cluster_fail_threshold = max(1, no_path_cluster_fail_threshold)
        self.no_path_cluster_timeout_sec = no_path_cluster_timeout_sec
        self.cache_reasons = set(cache_reasons or ["clearance", "no_path", "cost", "unknown"])
        self.log_individual_hits = log_individual_hits
        self.max_hit_logs_per_cycle = max_hit_logs_per_cycle
        self._entries: List[Tuple[Point2D, int, object, str]] = []
        self._cluster_fail_counts: Dict[int, int] = {}
        self._cluster_blacklist: Dict[int, object] = {}
        self._cluster_blacklist_reason: Dict[int, str] = {}
        self._hit_counts = Counter()
        self._hit_clusters = Counter()
        self._hit_logs = 0

    @property
    def entries(self) -> List[Tuple[Point2D, int, object, str]]:
        return self._entries

    def add(self, candidate: NavigationCandidate, reason: str) -> None:
        normalized = _normalize_reason(reason)
        if normalized not in self.cache_reasons:
            return
        self._entries.append((candidate.point_world, candidate.cluster_id, self.node.get_clock().now(), reason))
        self.node.get_logger().warn(
            "planner_reject_cache_added: reason=%s cluster_id=%d point=(%.2f, %.2f) radius=%.2f"
            % (normalized, candidate.cluster_id, candidate.point_world[0], candidate.point_world[1], self._radius_for(normalized))
        )
        key = (candidate.cluster_id, normalized)
        count = self._cluster_fail_counts.get(key, 0) + 1
        self._cluster_fail_counts[key] = count
        threshold = self.no_path_cluster_fail_threshold if normalized == "no_path" else self.cluster_fail_threshold
        if count >= threshold:
            self._cluster_blacklist[candidate.cluster_id] = self.node.get_clock().now()
            self._cluster_blacklist_reason[candidate.cluster_id] = normalized
            self.node.get_logger().warn(
                "planner_reject_cluster_blacklisted: reason=%s cluster_id=%d failures=%d timeout=%.1f"
                % (normalized, candidate.cluster_id, count, self._cluster_timeout_for(normalized))
            )

    def is_rejected(self, point_xy: Point2D, cluster_id: int) -> bool:
        self.expire()
        if cluster_id in self._cluster_blacklist:
            reason = self._cluster_blacklist_reason.get(cluster_id, "cluster")
            self._record_hit(cluster_id, reason, point_xy)
            return True
        for cached_xy, cached_cluster_id, _, reason in self._entries:
            normalized = _normalize_reason(reason)
            if cached_cluster_id == cluster_id and euclidean_distance(point_xy, cached_xy) <= self._radius_for(normalized):
                self._record_hit(cluster_id, normalized, point_xy)
                return True
        return False

    def begin_cycle(self) -> None:
        self._hit_counts = Counter()
        self._hit_clusters = Counter()
        self._hit_logs = 0

    def clear(self) -> None:
        self._entries = []
        self._cluster_blacklist = {}
        self._cluster_blacklist_reason = {}
        self._cluster_fail_counts = {}
        self.node.get_logger().info("planner_reject_cache_cleared: reason=robot_high_cost_recovery")

    def log_cycle_summary(self) -> None:
        total = sum(self._hit_counts.values())
        if total <= 0:
            return
        top_clusters = dict(self._hit_clusters.most_common(5))
        self.node.get_logger().info(
            "planner_reject_cache_summary: blacklist_hits_total=%d hits_by_reason=%s top_blacklisted_clusters=%s"
            % (total, dict(self._hit_counts), top_clusters)
        )

    def expire(self) -> None:
        now = self.node.get_clock().now()
        kept = []
        for point_xy, cluster_id, stamp, reason in self._entries:
            age = (now - stamp).nanoseconds / 1e9
            normalized = _normalize_reason(reason)
            if age <= self._timeout_for(normalized):
                kept.append((point_xy, cluster_id, stamp, reason))
            else:
                self.node.get_logger().info(
                    "expired planner reject blacklist: cluster_id=%d point=(%.2f, %.2f)"
                    % (cluster_id, point_xy[0], point_xy[1])
                )
        self._entries = kept

        kept_clusters: Dict[int, object] = {}
        for cluster_id, stamp in self._cluster_blacklist.items():
            age = (now - stamp).nanoseconds / 1e9
            reason = self._cluster_blacklist_reason.get(cluster_id, "clearance")
            if age <= self._cluster_timeout_for(reason):
                kept_clusters[cluster_id] = stamp
            else:
                self.node.get_logger().info("expired planner reject cluster blacklist: cluster_id=%d" % cluster_id)
                self._cluster_blacklist_reason.pop(cluster_id, None)
        self._cluster_blacklist = kept_clusters

    def _record_hit(self, cluster_id: int, reason: str, point_xy: Point2D) -> None:
        self._hit_counts[reason] += 1
        self._hit_clusters[cluster_id] += 1
        if self.log_individual_hits and self._hit_logs < self.max_hit_logs_per_cycle:
            self._hit_logs += 1
            self.node.get_logger().info(
                "planner_reject_blacklist_hit: reason=%s cluster_id=%d point=(%.2f, %.2f)"
                % (reason, cluster_id, point_xy[0], point_xy[1])
            )

    def _radius_for(self, reason: str) -> float:
        return self.no_path_radius_m if reason == "no_path" else self.radius_m

    def _timeout_for(self, reason: str) -> float:
        return self.no_path_timeout_sec if reason == "no_path" else self.timeout_sec

    def _cluster_timeout_for(self, reason: str) -> float:
        return self.no_path_cluster_timeout_sec if reason == "no_path" else self.cluster_timeout_sec


def _normalize_reason(reason: str) -> str:
    if reason in ("path_clearance", "clearance"):
        return "clearance"
    if reason in ("path_unknown", "unknown"):
        return "unknown"
    if reason in ("path_cost", "cost"):
        return "cost"
    if reason in ("no_path", "path_outside_costmap"):
        return "no_path"
    return reason
