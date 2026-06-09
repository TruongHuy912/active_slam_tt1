from dataclasses import dataclass
from typing import Optional, Tuple


Point2D = Tuple[float, float]
MapPoint2D = Tuple[float, float]


@dataclass(frozen=True)
class NavigationCandidate:
    cluster_id: int
    cluster_size: int
    point_world: Point2D
    point_map: MapPoint2D
    distance: float
    source: str
    frontier_world: Point2D
    score: float = 0.0
    information_gain: float = 0.0
    cost: Optional[int] = None
    cost_penalty: float = 0.0
    safety_radius_used: float = 0.0
    relaxed_clearance: bool = False
    utility_score: float = 0.0
    path_entropy: float = 0.0
    mean_path_entropy: float = 0.0
    unknown_ratio_along_path: float = 0.0
    local_entropy: float = 0.0
    uncertainty_proxy: float = 0.0
    region_diversity: float = 1.0


@dataclass(frozen=True)
class CandidateSelectionStats:
    total_frontier_clusters: int = 0
    sampled_frontier_cells: int = 0
    sampled_viewpoints: int = 0
    rejected_by_distance: int = 0
    rejected_by_costmap: int = 0
    rejected_by_clearance: int = 0
    rejected_by_blacklist: int = 0
    rejected_by_progress: int = 0
    rejected_by_cluster_size: int = 0
    local_candidates: int = 0
    utility_candidates: int = 0
    global_reposition_attempted: bool = False
    global_candidates: int = 0
    selected_mode: str = "none"
    fallback_mode: str = "none"
    relaxed_clearance: bool = False
    safety_radius_used: float = 0.0
    skip_reason: str = "none"


@dataclass(frozen=True)
class GoalSelectorConfig:
    goal_candidate_mode: str
    scoring_mode: str
    use_costmap_filter: bool
    max_allowed_cost: int
    reject_unknown_cost: bool
    safety_radius_m: float
    viewpoint_sample_radius_m: float
    viewpoint_num_samples: int
    min_goal_separation_m: float
    min_candidate_distance_m: float
    min_viewpoint_distance_m: float
    max_candidate_distance_m: float
    max_viewpoint_distance_m: float
    min_goal_progress_distance_m: float
    min_cluster_size_for_navigation: int
    max_cells_sampled_per_cluster: int
    prefer_nearest_valid_candidate: bool
    w_cluster_size: float
    w_distance: float
    w_information_gain: float
    w_cost_penalty: float
    w_goal_switching: float
    information_radius_m: float
    prefer_farther_than_current: bool
    blacklist_radius_m: float
    min_information_gain_for_goal: float
    enable_global_reposition: bool
    global_reposition_min_distance_m: float
    global_reposition_max_distance_m: float
    global_reposition_step_m: float
    global_reposition_sample_count: int
    fallback_relax_clearance: bool
    fallback_safety_radius_m: float
    fallback_reject_unknown_cost: bool
    max_candidates_per_cluster_for_validation: int = 5
    candidate_spatial_separation_m: float = 0.5
    enable_efficient_utility: bool = False
    max_utility_frontier_clusters: int = 20
    max_utility_candidates: int = 60
    max_candidates_per_region: int = 5
    region_grid_size_m: float = 1.0
    enable_region_diversity: bool = True
    recent_goal_region_penalty: float = 0.4
    rejected_region_penalty: float = 0.7
    region_memory_timeout_sec: float = 120.0
    w_path_entropy: float = 1.0
    w_region_diversity: float = 0.5
    w_path_length_penalty: float = 0.4
    w_uncertainty_penalty: float = 0.3
    entropy_window_radius_m: float = 0.4
    path_entropy_sample_step_m: float = 0.1
    normalize_utility_scores: bool = True
    max_utility_candidate_markers: int = 20
    enable_utility_fallback_to_baseline: bool = True
    utility_fallback_when_no_candidates: bool = True
    utility_min_safe_candidates_before_ranking: int = 5
    utility_bootstrap_min_frontier_clusters: int = 5
    utility_bootstrap_min_robot_travel_m: float = 0.5
    enable_bootstrap_exploration: bool = True
    bootstrap_max_cycles_without_goal: int = 3
    bootstrap_min_goal_distance_m: float = 0.4
    bootstrap_max_goal_distance_m: float = 1.2
    bootstrap_allow_relaxed_clearance: bool = True
    bootstrap_safety_radius_m: float = 0.15
    bootstrap_use_known_free_space: bool = True
    progress_gate_enabled: bool = True
    progress_gate_min_distance_m: float = 0.25
    progress_gate_timeout_sec: float = 8.0
    progress_gate_max_skip_cycles: int = 2
    progress_gate_disable_after_escape: bool = True
    post_escape_resume_delay_sec: float = 1.0
    post_escape_force_selection_cycles: int = 2
    global_reposition_max_consecutive_goals: int = 2
    global_reposition_cooldown_sec: float = 20.0
    global_reposition_recent_goal_radius_m: float = 1.2
    global_reposition_recent_region_penalty: float = 0.8
    global_reposition_min_information_gain: float = 0.02
    global_reposition_allow_zero_gain_only_if_no_alternative: bool = True
    global_reposition_blacklist_after_success_sec: float = 45.0
    global_reposition_pingpong_window: int = 6
    global_reposition_pingpong_radius_m: float = 1.0
    recent_goal_region_timeout_sec: float = 120.0
    recent_goal_region_radius_m: float = 1.0
    enable_goal_usefulness_gate: bool = True
    min_goal_information_gain: float = 0.01
    min_goal_frontier_distance_from_recent_m: float = 1.0
    min_expected_frontier_reduction: int = 0
    allow_low_gain_recovery_goal: bool = True
    post_global_reposition_prefer_frontier_cycles: int = 2
    post_global_reposition_wait_for_map_update_sec: float = 1.0
    enable_frontier_bridge_reposition: bool = False
    frontier_bridge_min_frontier_cells: int = 300
    frontier_bridge_min_best_cluster_size: int = 100
    frontier_bridge_min_best_distance_m: float = 8.0
    frontier_bridge_step_distances_m: Tuple[float, ...] = (2.0, 3.0, 4.0, 5.0)
    frontier_bridge_lateral_offsets_m: Tuple[float, ...] = (0.0, -0.5, 0.5, -1.0, 1.0)
    frontier_bridge_require_planner_validation: bool = True
    frontier_bridge_max_goal_cost: int = 40
    frontier_bridge_max_near_cost: int = 70


@dataclass(frozen=True)
class HighCostEscapeConfig:
    enabled: bool
    robot_cost_threshold: int
    sample_radius_m: float
    sample_count: int
    max_allowed_cost: int
    reject_unknown_cost: bool
    safety_radius_m: float
    validation_mode: bool
    ignore_start_radius_m: float
    path_clearance_radius_m: float
    allow_initial_high_cost: bool
    max_goal_distance_m: float
    min_cost_drop: int
    require_cost_decrease: bool
    max_attempts_per_cycle: int
