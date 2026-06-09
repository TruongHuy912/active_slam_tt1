from typing import List, Optional, Tuple

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.clock import Clock
from visualization_msgs.msg import Marker, MarkerArray

from bumperbot_active_slam.frontier_detector import FrontierCluster
from bumperbot_active_slam.models import NavigationCandidate, Point2D


def build_clear_markers(frame_id: str, clock: Clock) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.append(_delete_all_marker(frame_id, clock))
    return markers


def build_active_slam_markers(
    frame_id: str,
    clock: Clock,
    grid: OccupancyGrid,
    clusters: List[FrontierCluster],
    best: Optional[FrontierCluster],
    selected_goal_candidate: Optional[NavigationCandidate],
    active_goal_centroid: Optional[Point2D],
    valid_candidates: List[NavigationCandidate],
    rejected_candidates: List[Point2D],
    blacklist: List[Tuple[Point2D, object]],
    validated_path: Optional[Path],
    planner_rejected_candidates: List[NavigationCandidate],
    planner_reject_entries: List[Tuple[Point2D, int, object, str]],
    max_frontier_markers: int,
    max_candidate_markers: int,
    max_rejected_markers: int,
    blacklist_radius_m: float,
) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.append(_delete_all_marker(frame_id, clock))

    displayed_clusters = clusters[: max(0, max_frontier_markers)]
    for marker_id, cluster in enumerate(displayed_clusters, start=1):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = clock.now().to_msg()
        marker.ns = "frontier_clusters"
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = max(grid.info.resolution, 0.03)
        marker.scale.y = max(grid.info.resolution, 0.03)
        marker.color.r = 0.1
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 0.85
        marker.points = [_point_from_world(cluster.centroid_world)]
        markers.markers.append(marker)

        centroid_marker = Marker()
        centroid_marker.header.frame_id = frame_id
        centroid_marker.header.stamp = clock.now().to_msg()
        centroid_marker.ns = "cluster_centroid"
        centroid_marker.id = marker_id
        centroid_marker.type = Marker.SPHERE
        centroid_marker.action = Marker.ADD
        centroid_marker.pose.position = _point_from_world(cluster.centroid_world)
        centroid_marker.pose.orientation.w = 1.0
        centroid_marker.scale.x = 0.10
        centroid_marker.scale.y = 0.10
        centroid_marker.scale.z = 0.10
        centroid_marker.color.r = 0.75
        centroid_marker.color.g = 0.75
        centroid_marker.color.b = 0.75
        centroid_marker.color.a = 0.65
        markers.markers.append(centroid_marker)

    if best is not None:
        best_marker = Marker()
        best_marker.header.frame_id = frame_id
        best_marker.header.stamp = clock.now().to_msg()
        best_marker.ns = "best_frontier"
        best_marker.id = 100000
        best_marker.type = Marker.SPHERE
        best_marker.action = Marker.ADD
        best_marker.pose.position = _point_from_world(best.centroid_world)
        best_marker.pose.orientation.w = 1.0
        best_marker.scale.x = 0.22
        best_marker.scale.y = 0.22
        best_marker.scale.z = 0.22
        best_marker.color.r = 0.0
        best_marker.color.g = 1.0
        best_marker.color.b = 0.25
        best_marker.color.a = 0.95
        markers.markers.append(best_marker)

    selected_goal_xy = None
    if selected_goal_candidate is not None:
        selected_goal_xy = selected_goal_candidate.point_world
    elif active_goal_centroid is not None:
        selected_goal_xy = active_goal_centroid

    if selected_goal_xy is not None:
        selected_marker = Marker()
        selected_marker.header.frame_id = frame_id
        selected_marker.header.stamp = clock.now().to_msg()
        selected_marker.ns = "selected_goal"
        selected_marker.id = 100001
        selected_marker.type = Marker.CYLINDER
        selected_marker.action = Marker.ADD
        selected_marker.pose.position = _point_from_world(selected_goal_xy)
        selected_marker.pose.orientation.w = 1.0
        selected_marker.scale.x = 0.30
        selected_marker.scale.y = 0.30
        selected_marker.scale.z = 0.05
        selected_marker.color.r = 1.0
        selected_marker.color.g = 0.65
        selected_marker.color.b = 0.0
        selected_marker.color.a = 0.90
        markers.markers.append(selected_marker)

    for marker_id, candidate in enumerate(valid_candidates[: max_candidate_markers], start=120000):
        candidate_marker = Marker()
        candidate_marker.header.frame_id = frame_id
        candidate_marker.header.stamp = clock.now().to_msg()
        candidate_marker.ns = "utility_candidates" if candidate.utility_score != 0.0 else "safe_viewpoint_candidates"
        candidate_marker.id = marker_id
        candidate_marker.type = Marker.SPHERE
        candidate_marker.action = Marker.ADD
        candidate_marker.pose.position = _point_from_world(candidate.point_world)
        candidate_marker.pose.orientation.w = 1.0
        candidate_marker.scale.x = 0.08
        candidate_marker.scale.y = 0.08
        candidate_marker.scale.z = 0.08
        candidate_marker.color.r = 0.8 if candidate.utility_score != 0.0 else 0.2
        candidate_marker.color.g = 0.4 if candidate.utility_score != 0.0 else 0.9
        candidate_marker.color.b = 1.0 if candidate.utility_score != 0.0 else 0.9
        candidate_marker.color.a = 0.45
        markers.markers.append(candidate_marker)

    for marker_id, point_xy in enumerate(rejected_candidates[: max_rejected_markers], start=140000):
        rejected_marker = Marker()
        rejected_marker.header.frame_id = frame_id
        rejected_marker.header.stamp = clock.now().to_msg()
        rejected_marker.ns = "rejected_viewpoint_candidates"
        rejected_marker.id = marker_id
        rejected_marker.type = Marker.CUBE
        rejected_marker.action = Marker.ADD
        rejected_marker.pose.position = _point_from_world(point_xy)
        rejected_marker.pose.orientation.w = 1.0
        rejected_marker.scale.x = 0.06
        rejected_marker.scale.y = 0.06
        rejected_marker.scale.z = 0.06
        rejected_marker.color.r = 1.0
        rejected_marker.color.g = 0.2
        rejected_marker.color.b = 0.0
        rejected_marker.color.a = 0.35
        markers.markers.append(rejected_marker)

    for marker_id, (goal_xy, _) in enumerate(blacklist[:20], start=200000):
        blacklist_marker = Marker()
        blacklist_marker.header.frame_id = frame_id
        blacklist_marker.header.stamp = clock.now().to_msg()
        blacklist_marker.ns = "blacklist"
        blacklist_marker.id = marker_id
        blacklist_marker.type = Marker.SPHERE
        blacklist_marker.action = Marker.ADD
        blacklist_marker.pose.position = _point_from_world(goal_xy)
        blacklist_marker.pose.orientation.w = 1.0
        blacklist_marker.scale.x = max(blacklist_radius_m, 0.05)
        blacklist_marker.scale.y = max(blacklist_radius_m, 0.05)
        blacklist_marker.scale.z = 0.05
        blacklist_marker.color.r = 1.0
        blacklist_marker.color.g = 0.0
        blacklist_marker.color.b = 0.0
        blacklist_marker.color.a = 0.35
        markers.markers.append(blacklist_marker)

    if validated_path is not None and validated_path.poses:
        path_marker = Marker()
        path_marker.header.frame_id = frame_id
        path_marker.header.stamp = clock.now().to_msg()
        path_marker.ns = "planner_valid_path"
        path_marker.id = 300000
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.035
        path_marker.color.r = 0.1
        path_marker.color.g = 1.0
        path_marker.color.b = 0.1
        path_marker.color.a = 0.85
        path_marker.points = [
            _point_from_world((pose.pose.position.x, pose.pose.position.y))
            for pose in validated_path.poses
        ]
        markers.markers.append(path_marker)

    for marker_id, candidate in enumerate(planner_rejected_candidates[:20], start=320000):
        rejected_marker = Marker()
        rejected_marker.header.frame_id = frame_id
        rejected_marker.header.stamp = clock.now().to_msg()
        rejected_marker.ns = "planner_rejected_candidates"
        rejected_marker.id = marker_id
        rejected_marker.type = Marker.CUBE
        rejected_marker.action = Marker.ADD
        rejected_marker.pose.position = _point_from_world(candidate.point_world)
        rejected_marker.pose.orientation.w = 1.0
        rejected_marker.scale.x = 0.12
        rejected_marker.scale.y = 0.12
        rejected_marker.scale.z = 0.08
        rejected_marker.color.r = 1.0
        rejected_marker.color.g = 0.0
        rejected_marker.color.b = 0.8
        rejected_marker.color.a = 0.55
        markers.markers.append(rejected_marker)

    for marker_id, (point_xy, _, _, _) in enumerate(planner_reject_entries[:20], start=340000):
        cache_marker = Marker()
        cache_marker.header.frame_id = frame_id
        cache_marker.header.stamp = clock.now().to_msg()
        cache_marker.ns = "planner_reject_blacklist"
        cache_marker.id = marker_id
        cache_marker.type = Marker.SPHERE
        cache_marker.action = Marker.ADD
        cache_marker.pose.position = _point_from_world(point_xy)
        cache_marker.pose.orientation.w = 1.0
        cache_marker.scale.x = 0.22
        cache_marker.scale.y = 0.22
        cache_marker.scale.z = 0.06
        cache_marker.color.r = 0.9
        cache_marker.color.g = 0.0
        cache_marker.color.b = 1.0
        cache_marker.color.a = 0.35
        markers.markers.append(cache_marker)

    return markers


def _delete_all_marker(frame_id: str, clock: Clock) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = clock.now().to_msg()
    marker.ns = "active_slam"
    marker.id = 0
    marker.action = Marker.DELETEALL
    return marker


def _point_from_world(point_xy: Point2D) -> Point:
    point = Point()
    point.x = float(point_xy[0])
    point.y = float(point_xy[1])
    point.z = 0.05
    return point
