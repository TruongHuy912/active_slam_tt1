from bumperbot_active_slam.models import GoalSelectorConfig, HighCostEscapeConfig
from bumperbot_active_slam.planner_validator import PlannerValidationConfig


def declare_active_slam_parameters(node) -> None:
    node.declare_parameter("map_topic", "/map")
    node.declare_parameter("global_frame", "map")
    node.declare_parameter("robot_frame", "base_link")
    node.declare_parameter("marker_topic", "/active_slam/markers")
    node.declare_parameter("update_period_sec", 2.0)
    node.declare_parameter("frontier_connectivity", 8)
    node.declare_parameter("min_cluster_size", 5)
    node.declare_parameter("max_frontier_markers", 100)
    node.declare_parameter("max_candidate_markers", 40)
    node.declare_parameter("max_rejected_markers", 40)
    node.declare_parameter("enable_debug_markers", True)
    node.declare_parameter("tf_lookup_timeout_sec", 0.5)
    node.declare_parameter("log_period_sec", 5.0)
    node.declare_parameter("publish_empty_markers", True)
    node.declare_parameter("enable_navigation", False)
    node.declare_parameter("navigate_action_name", "/navigate_to_pose")
    node.declare_parameter("goal_update_period_sec", 5.0)
    node.declare_parameter("goal_timeout_sec", 60.0)
    node.declare_parameter("goal_candidate_mode", "frontier_cell")
    node.declare_parameter("scoring_mode", "safe_viewpoint")
    node.declare_parameter("use_costmap_filter", True)
    node.declare_parameter("costmap_topic", "/global_costmap/costmap")
    node.declare_parameter("max_allowed_cost", 70)
    node.declare_parameter("reject_unknown_cost", True)
    node.declare_parameter("safety_radius_m", 0.25)
    node.declare_parameter("viewpoint_sample_radius_m", 0.45)
    node.declare_parameter("viewpoint_num_samples", 16)
    node.declare_parameter("min_goal_separation_m", 0.5)
    node.declare_parameter("min_candidate_distance_m", 0.6)
    node.declare_parameter("min_viewpoint_distance_m", 0.8)
    node.declare_parameter("max_candidate_distance_m", 6.0)
    node.declare_parameter("max_viewpoint_distance_m", 6.0)
    node.declare_parameter("min_goal_progress_distance_m", 0.4)
    node.declare_parameter("min_cluster_size_for_navigation", 5)
    node.declare_parameter("max_cells_sampled_per_cluster", 100)
    node.declare_parameter("prefer_nearest_valid_candidate", True)
    node.declare_parameter("w_cluster_size", 1.0)
    node.declare_parameter("w_distance", 0.5)
    node.declare_parameter("w_information_gain", 1.0)
    node.declare_parameter("w_cost_penalty", 1.0)
    node.declare_parameter("w_goal_switching", 0.5)
    node.declare_parameter("information_radius_m", 0.6)
    node.declare_parameter("prefer_farther_than_current", True)
    node.declare_parameter("min_information_gain_for_goal", 0.0)
    node.declare_parameter("enable_global_reposition", True)
    node.declare_parameter("global_reposition_min_distance_m", 4.0)
    node.declare_parameter("global_reposition_max_distance_m", 12.0)
    node.declare_parameter("global_reposition_step_m", 2.5)
    node.declare_parameter("global_reposition_sample_count", 24)
    node.declare_parameter("fallback_relax_clearance", True)
    node.declare_parameter("fallback_safety_radius_m", 0.18)
    node.declare_parameter("fallback_reject_unknown_cost", False)
    node.declare_parameter("goal_reached_distance_m", 0.35)
    node.declare_parameter("blacklist_radius_m", 0.6)
    node.declare_parameter("blacklist_timeout_sec", 90.0)
    node.declare_parameter("max_retries_per_frontier", 2)
    node.declare_parameter("send_goal_on_startup", False)
    node.declare_parameter("use_planner_validation", True)
    node.declare_parameter("planner_action_name", "/compute_path_to_pose")
    node.declare_parameter("planner_id", "GridBased")
    node.declare_parameter("planner_validation_timeout_sec", 3.0)
    node.declare_parameter("min_valid_path_length_m", 0.2)
    node.declare_parameter("max_valid_path_length_m", 15.0)
    node.declare_parameter("max_path_cost", 70)
    node.declare_parameter("reject_path_unknown", True)
    node.declare_parameter("path_check_step_m", 0.05)
    node.declare_parameter("path_clearance_radius_m", 0.22)
    node.declare_parameter("path_clearance_max_near_cost", 70)
    node.declare_parameter("path_clearance_lethal_cost", 90)
    node.declare_parameter("allow_low_inflation_near_path", True)
    node.declare_parameter("low_inflation_cost_threshold", 40)
    node.declare_parameter("normal_path_ignore_start_radius_m", 0.25)
    node.declare_parameter("max_planner_validation_candidates", 20)
    node.declare_parameter("planner_validation_required_for_navigation", True)
    node.declare_parameter("planner_validation_retry_next_best", True)
    node.declare_parameter("planner_validation_max_batches", 4)
    node.declare_parameter("planner_validation_batch_size", 20)
    node.declare_parameter("max_candidates_per_cluster_for_validation", 5)
    node.declare_parameter("candidate_spatial_separation_m", 0.5)
    node.declare_parameter("blacklist_on_planner_reject", True)
    node.declare_parameter("planner_reject_blacklist_timeout_sec", 45.0)
    node.declare_parameter("planner_reject_blacklist_radius_m", 0.8)
    node.declare_parameter("planner_reject_cluster_fail_threshold", 3)
    node.declare_parameter("planner_reject_cluster_timeout_sec", 45.0)
    node.declare_parameter("fallback_relax_path_clearance", True)
    node.declare_parameter("fallback_path_clearance_radius_m", 0.15)
    node.declare_parameter("enable_global_reposition_after_planner_fail", True)
    node.declare_parameter("max_planner_rejected_markers", 20)
    node.declare_parameter("planner_reject_reasons_to_cache", ["clearance", "no_path", "cost", "unknown"])
    node.declare_parameter("no_path_blacklist_timeout_sec", 60.0)
    node.declare_parameter("no_path_blacklist_radius_m", 1.0)
    node.declare_parameter("no_path_cluster_fail_threshold", 2)
    node.declare_parameter("no_path_cluster_timeout_sec", 90.0)
    node.declare_parameter("log_individual_blacklist_hits", False)
    node.declare_parameter("max_blacklist_hit_logs_per_cycle", 5)
    node.declare_parameter("selection_tier_mode", "progressive")
    node.declare_parameter("enable_medium_reposition", True)
    node.declare_parameter("medium_reposition_min_distance_m", 1.0)
    node.declare_parameter("medium_reposition_max_distance_m", 3.0)
    node.declare_parameter("medium_reposition_sample_count", 24)
    node.declare_parameter("enable_rotate_recovery_goal", True)
    node.declare_parameter("rotate_recovery_when_no_valid_goal", True)
    node.declare_parameter("progress_check_period_sec", 3.0)
    node.declare_parameter("min_progress_distance_m", 0.15)
    node.declare_parameter("stuck_timeout_sec", 12.0)
    node.declare_parameter("cancel_goal_on_stuck", True)
    node.declare_parameter("blacklist_goal_on_stuck", True)
    node.declare_parameter("high_cost_robot_threshold", 70)
    node.declare_parameter("high_cost_escape_enabled", True)
    node.declare_parameter("high_cost_escape_radius_m", 0.8)
    node.declare_parameter("high_cost_escape_samples", 24)
    node.declare_parameter("high_cost_escape_validation_mode", True)
    node.declare_parameter("high_cost_escape_ignore_start_radius_m", 0.35)
    node.declare_parameter("high_cost_escape_path_clearance_radius_m", 0.10)
    node.declare_parameter("high_cost_escape_allow_initial_high_cost", True)
    node.declare_parameter("high_cost_escape_max_goal_distance_m", 1.2)
    node.declare_parameter("high_cost_escape_min_cost_drop", 20)
    node.declare_parameter("high_cost_escape_require_cost_decrease", True)
    node.declare_parameter("high_cost_escape_max_attempts_per_cycle", 8)
    node.declare_parameter("recovery_wait_for_costmap_update_sec", 2.0)
    node.declare_parameter("recovery_clear_reject_cache_when_robot_high_cost", True)
    node.declare_parameter("enable_efficient_utility", False)
    node.declare_parameter("max_utility_frontier_clusters", 20)
    node.declare_parameter("max_utility_candidates", 60)
    node.declare_parameter("max_candidates_per_region", 5)
    node.declare_parameter("region_grid_size_m", 1.0)
    node.declare_parameter("enable_region_diversity", True)
    node.declare_parameter("recent_goal_region_penalty", 0.4)
    node.declare_parameter("rejected_region_penalty", 0.7)
    node.declare_parameter("region_memory_timeout_sec", 120.0)
    node.declare_parameter("w_path_entropy", 1.0)
    node.declare_parameter("w_region_diversity", 0.5)
    node.declare_parameter("w_path_length_penalty", 0.4)
    node.declare_parameter("w_uncertainty_penalty", 0.3)
    node.declare_parameter("entropy_window_radius_m", 0.4)
    node.declare_parameter("path_entropy_sample_step_m", 0.1)
    node.declare_parameter("normalize_utility_scores", True)
    node.declare_parameter("max_utility_candidate_markers", 20)
    node.declare_parameter("enable_utility_fallback_to_baseline", True)
    node.declare_parameter("utility_fallback_when_no_candidates", True)
    node.declare_parameter("utility_min_safe_candidates_before_ranking", 5)
    node.declare_parameter("utility_bootstrap_min_frontier_clusters", 5)
    node.declare_parameter("utility_bootstrap_min_robot_travel_m", 0.5)
    node.declare_parameter("enable_bootstrap_exploration", True)
    node.declare_parameter("bootstrap_max_cycles_without_goal", 3)
    node.declare_parameter("bootstrap_min_goal_distance_m", 0.4)
    node.declare_parameter("bootstrap_max_goal_distance_m", 1.2)
    node.declare_parameter("bootstrap_allow_relaxed_clearance", True)
    node.declare_parameter("bootstrap_safety_radius_m", 0.15)
    node.declare_parameter("bootstrap_use_known_free_space", True)
    node.declare_parameter("progress_gate_enabled", True)
    node.declare_parameter("progress_gate_min_distance_m", 0.25)
    node.declare_parameter("progress_gate_timeout_sec", 8.0)
    node.declare_parameter("progress_gate_max_skip_cycles", 2)
    node.declare_parameter("progress_gate_disable_after_escape", True)
    node.declare_parameter("post_escape_resume_delay_sec", 1.0)
    node.declare_parameter("post_escape_force_selection_cycles", 2)
    node.declare_parameter("global_reposition_max_consecutive_goals", 2)
    node.declare_parameter("global_reposition_cooldown_sec", 20.0)
    node.declare_parameter("global_reposition_recent_goal_radius_m", 1.2)
    node.declare_parameter("global_reposition_recent_region_penalty", 0.8)
    node.declare_parameter("global_reposition_min_information_gain", 0.02)
    node.declare_parameter("global_reposition_allow_zero_gain_only_if_no_alternative", True)
    node.declare_parameter("global_reposition_blacklist_after_success_sec", 45.0)
    node.declare_parameter("global_reposition_pingpong_window", 6)
    node.declare_parameter("global_reposition_pingpong_radius_m", 1.0)
    node.declare_parameter("recent_goal_region_timeout_sec", 120.0)
    node.declare_parameter("recent_goal_region_radius_m", 1.0)
    node.declare_parameter("enable_goal_usefulness_gate", True)
    node.declare_parameter("min_goal_information_gain", 0.01)
    node.declare_parameter("min_goal_frontier_distance_from_recent_m", 1.0)
    node.declare_parameter("min_expected_frontier_reduction", 0)
    node.declare_parameter("allow_low_gain_recovery_goal", True)
    node.declare_parameter("post_global_reposition_prefer_frontier_cycles", 2)
    node.declare_parameter("post_global_reposition_wait_for_map_update_sec", 1.0)
    node.declare_parameter("enable_frontier_bridge_reposition", False)
    node.declare_parameter("frontier_bridge_min_frontier_cells", 300)
    node.declare_parameter("frontier_bridge_min_best_cluster_size", 100)
    node.declare_parameter("frontier_bridge_min_best_distance_m", 8.0)
    node.declare_parameter("frontier_bridge_step_distances_m", [2.0, 3.0, 4.0, 5.0])
    node.declare_parameter("frontier_bridge_lateral_offsets_m", [0.0, -0.5, 0.5, -1.0, 1.0])
    node.declare_parameter("frontier_bridge_require_planner_validation", True)
    node.declare_parameter("frontier_bridge_max_goal_cost", 40)
    node.declare_parameter("frontier_bridge_max_near_cost", 70)
    node.declare_parameter("enable_unreachable_frontier_cooldown", False)
    node.declare_parameter("unreachable_frontier_no_safe_cycles", 3)
    node.declare_parameter("unreachable_frontier_cooldown_sec", 90.0)
    node.declare_parameter("near_frontier_max_best_distance_m", 4.0)
    node.declare_parameter("near_frontier_min_frontier_cells", 400)
    node.declare_parameter("near_frontier_min_best_cluster_size", 80)
    node.declare_parameter("enable_final_no_safe_viewpoint_stop", False)
    node.declare_parameter("final_no_safe_viewpoint_cycles", 6)
    node.declare_parameter("enable_stale_frontier_suppression", False)
    node.declare_parameter("stale_frontier_region_size_m", 1.0)
    node.declare_parameter("stale_frontier_max_repeats", 3)
    node.declare_parameter("stale_frontier_cooldown_sec", 60.0)
    node.declare_parameter("stale_frontier_min_frontier_reduction", 20)
    node.declare_parameter("stale_frontier_min_distance_change_m", 0.30)
    node.declare_parameter("enable_high_cost_failure_stop", False)
    node.declare_parameter("max_consecutive_high_cost_escape_failures", 3)


def read_active_slam_parameters(node) -> None:
    node.map_topic = node.get_parameter("map_topic").value
    node.global_frame = node.get_parameter("global_frame").value
    node.robot_frame = node.get_parameter("robot_frame").value
    node.marker_topic = node.get_parameter("marker_topic").value
    node.update_period_sec = max(0.1, float(node.get_parameter("update_period_sec").value))
    node.frontier_connectivity = int(node.get_parameter("frontier_connectivity").value)
    node.min_cluster_size = max(1, int(node.get_parameter("min_cluster_size").value))
    node.max_frontier_markers = max(0, int(node.get_parameter("max_frontier_markers").value))
    node.max_candidate_markers = max(0, int(node.get_parameter("max_candidate_markers").value))
    node.max_rejected_markers = max(0, int(node.get_parameter("max_rejected_markers").value))
    node.enable_debug_markers = _as_bool(node.get_parameter("enable_debug_markers").value)
    node.tf_lookup_timeout_sec = max(0.0, float(node.get_parameter("tf_lookup_timeout_sec").value))
    node.log_period_sec = max(0.5, float(node.get_parameter("log_period_sec").value))
    node.publish_empty_markers = _as_bool(node.get_parameter("publish_empty_markers").value)
    node.enable_navigation = _as_bool(node.get_parameter("enable_navigation").value)
    node.navigate_action_name = node.get_parameter("navigate_action_name").value
    node.goal_update_period_sec = max(0.5, float(node.get_parameter("goal_update_period_sec").value))
    node.goal_timeout_sec = max(1.0, float(node.get_parameter("goal_timeout_sec").value))
    node.goal_candidate_mode = str(node.get_parameter("goal_candidate_mode").value)
    node.scoring_mode = str(node.get_parameter("scoring_mode").value)
    node.use_costmap_filter = _as_bool(node.get_parameter("use_costmap_filter").value)
    node.costmap_topic = str(node.get_parameter("costmap_topic").value)
    node.max_allowed_cost = max(0, min(100, int(node.get_parameter("max_allowed_cost").value)))
    node.reject_unknown_cost = _as_bool(node.get_parameter("reject_unknown_cost").value)
    node.safety_radius_m = max(0.0, float(node.get_parameter("safety_radius_m").value))
    node.viewpoint_sample_radius_m = max(0.05, float(node.get_parameter("viewpoint_sample_radius_m").value))
    node.viewpoint_num_samples = max(1, int(node.get_parameter("viewpoint_num_samples").value))
    node.min_goal_separation_m = max(0.0, float(node.get_parameter("min_goal_separation_m").value))
    node.min_candidate_distance_m = max(0.0, float(node.get_parameter("min_candidate_distance_m").value))
    node.min_viewpoint_distance_m = max(
        node.min_candidate_distance_m,
        float(node.get_parameter("min_viewpoint_distance_m").value),
    )
    node.max_candidate_distance_m = max(
        node.min_candidate_distance_m,
        float(node.get_parameter("max_candidate_distance_m").value),
    )
    node.max_viewpoint_distance_m = max(
        node.min_viewpoint_distance_m,
        float(node.get_parameter("max_viewpoint_distance_m").value),
    )
    node.min_goal_progress_distance_m = max(0.0, float(node.get_parameter("min_goal_progress_distance_m").value))
    node.min_cluster_size_for_navigation = max(1, int(node.get_parameter("min_cluster_size_for_navigation").value))
    node.max_cells_sampled_per_cluster = max(1, int(node.get_parameter("max_cells_sampled_per_cluster").value))
    node.prefer_nearest_valid_candidate = _as_bool(node.get_parameter("prefer_nearest_valid_candidate").value)
    node.w_cluster_size = float(node.get_parameter("w_cluster_size").value)
    node.w_distance = float(node.get_parameter("w_distance").value)
    node.w_information_gain = float(node.get_parameter("w_information_gain").value)
    node.w_cost_penalty = float(node.get_parameter("w_cost_penalty").value)
    node.w_goal_switching = float(node.get_parameter("w_goal_switching").value)
    node.information_radius_m = max(0.05, float(node.get_parameter("information_radius_m").value))
    node.prefer_farther_than_current = _as_bool(node.get_parameter("prefer_farther_than_current").value)
    node.min_information_gain_for_goal = max(0.0, float(node.get_parameter("min_information_gain_for_goal").value))
    node.enable_global_reposition = _as_bool(node.get_parameter("enable_global_reposition").value)
    node.global_reposition_min_distance_m = max(
        0.0,
        float(node.get_parameter("global_reposition_min_distance_m").value),
    )
    node.global_reposition_max_distance_m = max(
        node.global_reposition_min_distance_m,
        float(node.get_parameter("global_reposition_max_distance_m").value),
    )
    node.global_reposition_step_m = max(0.2, float(node.get_parameter("global_reposition_step_m").value))
    node.global_reposition_sample_count = max(1, int(node.get_parameter("global_reposition_sample_count").value))
    node.fallback_relax_clearance = _as_bool(node.get_parameter("fallback_relax_clearance").value)
    node.fallback_safety_radius_m = max(0.0, float(node.get_parameter("fallback_safety_radius_m").value))
    node.fallback_reject_unknown_cost = _as_bool(node.get_parameter("fallback_reject_unknown_cost").value)
    node.goal_reached_distance_m = max(0.0, float(node.get_parameter("goal_reached_distance_m").value))
    node.blacklist_radius_m = max(0.0, float(node.get_parameter("blacklist_radius_m").value))
    node.blacklist_timeout_sec = max(1.0, float(node.get_parameter("blacklist_timeout_sec").value))
    node.max_retries_per_frontier = max(1, int(node.get_parameter("max_retries_per_frontier").value))
    node.send_goal_on_startup = _as_bool(node.get_parameter("send_goal_on_startup").value)
    node.use_planner_validation = _as_bool(node.get_parameter("use_planner_validation").value)
    node.planner_action_name = str(node.get_parameter("planner_action_name").value)
    node.planner_id = str(node.get_parameter("planner_id").value)
    node.planner_validation_timeout_sec = max(0.2, float(node.get_parameter("planner_validation_timeout_sec").value))
    node.min_valid_path_length_m = max(0.0, float(node.get_parameter("min_valid_path_length_m").value))
    node.max_valid_path_length_m = max(
        node.min_valid_path_length_m,
        float(node.get_parameter("max_valid_path_length_m").value),
    )
    node.max_path_cost = max(0, min(100, int(node.get_parameter("max_path_cost").value)))
    node.reject_path_unknown = _as_bool(node.get_parameter("reject_path_unknown").value)
    node.path_check_step_m = max(0.01, float(node.get_parameter("path_check_step_m").value))
    node.path_clearance_radius_m = max(0.0, float(node.get_parameter("path_clearance_radius_m").value))
    node.path_clearance_max_near_cost = max(
        0,
        min(100, int(node.get_parameter("path_clearance_max_near_cost").value)),
    )
    node.path_clearance_lethal_cost = max(
        0,
        min(100, int(node.get_parameter("path_clearance_lethal_cost").value)),
    )
    node.allow_low_inflation_near_path = _as_bool(
        node.get_parameter("allow_low_inflation_near_path").value
    )
    node.low_inflation_cost_threshold = max(
        0,
        min(100, int(node.get_parameter("low_inflation_cost_threshold").value)),
    )
    node.normal_path_ignore_start_radius_m = max(
        0.0,
        float(node.get_parameter("normal_path_ignore_start_radius_m").value),
    )
    node.max_planner_validation_candidates = max(1, int(node.get_parameter("max_planner_validation_candidates").value))
    node.planner_validation_required_for_navigation = _as_bool(
        node.get_parameter("planner_validation_required_for_navigation").value
    )
    node.planner_validation_retry_next_best = _as_bool(
        node.get_parameter("planner_validation_retry_next_best").value
    )
    node.planner_validation_max_batches = max(1, int(node.get_parameter("planner_validation_max_batches").value))
    node.planner_validation_batch_size = max(1, int(node.get_parameter("planner_validation_batch_size").value))
    node.max_candidates_per_cluster_for_validation = max(
        1,
        int(node.get_parameter("max_candidates_per_cluster_for_validation").value),
    )
    node.candidate_spatial_separation_m = max(
        0.0,
        float(node.get_parameter("candidate_spatial_separation_m").value),
    )
    node.blacklist_on_planner_reject = _as_bool(node.get_parameter("blacklist_on_planner_reject").value)
    node.planner_reject_blacklist_timeout_sec = max(
        1.0,
        float(node.get_parameter("planner_reject_blacklist_timeout_sec").value),
    )
    node.planner_reject_blacklist_radius_m = max(
        0.0,
        float(node.get_parameter("planner_reject_blacklist_radius_m").value),
    )
    node.planner_reject_cluster_fail_threshold = max(
        1,
        int(node.get_parameter("planner_reject_cluster_fail_threshold").value),
    )
    node.planner_reject_cluster_timeout_sec = max(
        1.0,
        float(node.get_parameter("planner_reject_cluster_timeout_sec").value),
    )
    node.fallback_relax_path_clearance = _as_bool(node.get_parameter("fallback_relax_path_clearance").value)
    node.fallback_path_clearance_radius_m = max(
        0.0,
        float(node.get_parameter("fallback_path_clearance_radius_m").value),
    )
    node.enable_global_reposition_after_planner_fail = _as_bool(
        node.get_parameter("enable_global_reposition_after_planner_fail").value
    )
    node.max_planner_rejected_markers = max(0, int(node.get_parameter("max_planner_rejected_markers").value))
    node.planner_reject_reasons_to_cache = list(node.get_parameter("planner_reject_reasons_to_cache").value)
    node.no_path_blacklist_timeout_sec = max(1.0, float(node.get_parameter("no_path_blacklist_timeout_sec").value))
    node.no_path_blacklist_radius_m = max(0.0, float(node.get_parameter("no_path_blacklist_radius_m").value))
    node.no_path_cluster_fail_threshold = max(1, int(node.get_parameter("no_path_cluster_fail_threshold").value))
    node.no_path_cluster_timeout_sec = max(1.0, float(node.get_parameter("no_path_cluster_timeout_sec").value))
    node.log_individual_blacklist_hits = _as_bool(node.get_parameter("log_individual_blacklist_hits").value)
    node.max_blacklist_hit_logs_per_cycle = max(0, int(node.get_parameter("max_blacklist_hit_logs_per_cycle").value))
    node.selection_tier_mode = str(node.get_parameter("selection_tier_mode").value)
    node.enable_medium_reposition = _as_bool(node.get_parameter("enable_medium_reposition").value)
    node.medium_reposition_min_distance_m = max(0.0, float(node.get_parameter("medium_reposition_min_distance_m").value))
    node.medium_reposition_max_distance_m = max(
        node.medium_reposition_min_distance_m,
        float(node.get_parameter("medium_reposition_max_distance_m").value),
    )
    node.medium_reposition_sample_count = max(1, int(node.get_parameter("medium_reposition_sample_count").value))
    node.enable_rotate_recovery_goal = _as_bool(node.get_parameter("enable_rotate_recovery_goal").value)
    node.rotate_recovery_when_no_valid_goal = _as_bool(node.get_parameter("rotate_recovery_when_no_valid_goal").value)
    node.progress_check_period_sec = max(0.5, float(node.get_parameter("progress_check_period_sec").value))
    node.min_progress_distance_m = max(0.0, float(node.get_parameter("min_progress_distance_m").value))
    node.stuck_timeout_sec = max(1.0, float(node.get_parameter("stuck_timeout_sec").value))
    node.cancel_goal_on_stuck = _as_bool(node.get_parameter("cancel_goal_on_stuck").value)
    node.blacklist_goal_on_stuck = _as_bool(node.get_parameter("blacklist_goal_on_stuck").value)
    node.high_cost_robot_threshold = max(0, min(100, int(node.get_parameter("high_cost_robot_threshold").value)))
    node.high_cost_escape_enabled = _as_bool(node.get_parameter("high_cost_escape_enabled").value)
    node.high_cost_escape_radius_m = max(0.1, float(node.get_parameter("high_cost_escape_radius_m").value))
    node.high_cost_escape_samples = max(1, int(node.get_parameter("high_cost_escape_samples").value))
    node.high_cost_escape_validation_mode = _as_bool(
        node.get_parameter("high_cost_escape_validation_mode").value
    )
    node.high_cost_escape_ignore_start_radius_m = max(
        0.0,
        float(node.get_parameter("high_cost_escape_ignore_start_radius_m").value),
    )
    node.high_cost_escape_path_clearance_radius_m = max(
        0.0,
        float(node.get_parameter("high_cost_escape_path_clearance_radius_m").value),
    )
    node.high_cost_escape_allow_initial_high_cost = _as_bool(
        node.get_parameter("high_cost_escape_allow_initial_high_cost").value
    )
    node.high_cost_escape_max_goal_distance_m = max(
        0.2,
        float(node.get_parameter("high_cost_escape_max_goal_distance_m").value),
    )
    node.high_cost_escape_min_cost_drop = max(0, int(node.get_parameter("high_cost_escape_min_cost_drop").value))
    node.high_cost_escape_require_cost_decrease = _as_bool(
        node.get_parameter("high_cost_escape_require_cost_decrease").value
    )
    node.high_cost_escape_max_attempts_per_cycle = max(
        1,
        int(node.get_parameter("high_cost_escape_max_attempts_per_cycle").value),
    )
    node.recovery_wait_for_costmap_update_sec = max(
        0.0,
        float(node.get_parameter("recovery_wait_for_costmap_update_sec").value),
    )
    node.recovery_clear_reject_cache_when_robot_high_cost = _as_bool(
        node.get_parameter("recovery_clear_reject_cache_when_robot_high_cost").value
    )
    node.enable_efficient_utility = _as_bool(node.get_parameter("enable_efficient_utility").value)
    node.max_utility_frontier_clusters = max(1, int(node.get_parameter("max_utility_frontier_clusters").value))
    node.max_utility_candidates = max(1, int(node.get_parameter("max_utility_candidates").value))
    node.max_candidates_per_region = max(1, int(node.get_parameter("max_candidates_per_region").value))
    node.region_grid_size_m = max(0.1, float(node.get_parameter("region_grid_size_m").value))
    node.enable_region_diversity = _as_bool(node.get_parameter("enable_region_diversity").value)
    node.recent_goal_region_penalty = max(0.0, float(node.get_parameter("recent_goal_region_penalty").value))
    node.rejected_region_penalty = max(0.0, float(node.get_parameter("rejected_region_penalty").value))
    node.region_memory_timeout_sec = max(1.0, float(node.get_parameter("region_memory_timeout_sec").value))
    node.w_path_entropy = float(node.get_parameter("w_path_entropy").value)
    node.w_region_diversity = float(node.get_parameter("w_region_diversity").value)
    node.w_path_length_penalty = float(node.get_parameter("w_path_length_penalty").value)
    node.w_uncertainty_penalty = float(node.get_parameter("w_uncertainty_penalty").value)
    node.entropy_window_radius_m = max(0.05, float(node.get_parameter("entropy_window_radius_m").value))
    node.path_entropy_sample_step_m = max(0.01, float(node.get_parameter("path_entropy_sample_step_m").value))
    node.normalize_utility_scores = _as_bool(node.get_parameter("normalize_utility_scores").value)
    node.max_utility_candidate_markers = max(0, int(node.get_parameter("max_utility_candidate_markers").value))
    node.enable_utility_fallback_to_baseline = _as_bool(
        node.get_parameter("enable_utility_fallback_to_baseline").value
    )
    node.utility_fallback_when_no_candidates = _as_bool(
        node.get_parameter("utility_fallback_when_no_candidates").value
    )
    node.utility_min_safe_candidates_before_ranking = max(
        1,
        int(node.get_parameter("utility_min_safe_candidates_before_ranking").value),
    )
    node.utility_bootstrap_min_frontier_clusters = max(
        1,
        int(node.get_parameter("utility_bootstrap_min_frontier_clusters").value),
    )
    node.utility_bootstrap_min_robot_travel_m = max(
        0.0,
        float(node.get_parameter("utility_bootstrap_min_robot_travel_m").value),
    )
    node.enable_bootstrap_exploration = _as_bool(node.get_parameter("enable_bootstrap_exploration").value)
    node.bootstrap_max_cycles_without_goal = max(
        1,
        int(node.get_parameter("bootstrap_max_cycles_without_goal").value),
    )
    node.bootstrap_min_goal_distance_m = max(
        0.0,
        float(node.get_parameter("bootstrap_min_goal_distance_m").value),
    )
    node.bootstrap_max_goal_distance_m = max(
        node.bootstrap_min_goal_distance_m,
        float(node.get_parameter("bootstrap_max_goal_distance_m").value),
    )
    node.bootstrap_allow_relaxed_clearance = _as_bool(
        node.get_parameter("bootstrap_allow_relaxed_clearance").value
    )
    node.bootstrap_safety_radius_m = max(0.0, float(node.get_parameter("bootstrap_safety_radius_m").value))
    node.bootstrap_use_known_free_space = _as_bool(
        node.get_parameter("bootstrap_use_known_free_space").value
    )
    node.progress_gate_enabled = _as_bool(node.get_parameter("progress_gate_enabled").value)
    node.progress_gate_min_distance_m = max(
        0.0,
        float(node.get_parameter("progress_gate_min_distance_m").value),
    )
    node.progress_gate_timeout_sec = max(0.0, float(node.get_parameter("progress_gate_timeout_sec").value))
    node.progress_gate_max_skip_cycles = max(0, int(node.get_parameter("progress_gate_max_skip_cycles").value))
    node.progress_gate_disable_after_escape = _as_bool(
        node.get_parameter("progress_gate_disable_after_escape").value
    )
    node.post_escape_resume_delay_sec = max(
        0.0,
        float(node.get_parameter("post_escape_resume_delay_sec").value),
    )
    node.post_escape_force_selection_cycles = max(
        0,
        int(node.get_parameter("post_escape_force_selection_cycles").value),
    )
    node.global_reposition_max_consecutive_goals = max(
        0,
        int(node.get_parameter("global_reposition_max_consecutive_goals").value),
    )
    node.global_reposition_cooldown_sec = max(
        0.0,
        float(node.get_parameter("global_reposition_cooldown_sec").value),
    )
    node.global_reposition_recent_goal_radius_m = max(
        0.0,
        float(node.get_parameter("global_reposition_recent_goal_radius_m").value),
    )
    node.global_reposition_recent_region_penalty = max(
        0.0,
        min(1.0, float(node.get_parameter("global_reposition_recent_region_penalty").value)),
    )
    node.global_reposition_min_information_gain = max(
        0.0,
        float(node.get_parameter("global_reposition_min_information_gain").value),
    )
    node.global_reposition_allow_zero_gain_only_if_no_alternative = _as_bool(
        node.get_parameter("global_reposition_allow_zero_gain_only_if_no_alternative").value
    )
    node.global_reposition_blacklist_after_success_sec = max(
        0.0,
        float(node.get_parameter("global_reposition_blacklist_after_success_sec").value),
    )
    node.global_reposition_pingpong_window = max(
        2,
        int(node.get_parameter("global_reposition_pingpong_window").value),
    )
    node.global_reposition_pingpong_radius_m = max(
        0.0,
        float(node.get_parameter("global_reposition_pingpong_radius_m").value),
    )
    node.recent_goal_region_timeout_sec = max(
        0.0,
        float(node.get_parameter("recent_goal_region_timeout_sec").value),
    )
    node.recent_goal_region_radius_m = max(
        0.0,
        float(node.get_parameter("recent_goal_region_radius_m").value),
    )
    node.enable_goal_usefulness_gate = _as_bool(node.get_parameter("enable_goal_usefulness_gate").value)
    node.min_goal_information_gain = max(0.0, float(node.get_parameter("min_goal_information_gain").value))
    node.min_goal_frontier_distance_from_recent_m = max(
        0.0,
        float(node.get_parameter("min_goal_frontier_distance_from_recent_m").value),
    )
    node.min_expected_frontier_reduction = max(0, int(node.get_parameter("min_expected_frontier_reduction").value))
    node.allow_low_gain_recovery_goal = _as_bool(node.get_parameter("allow_low_gain_recovery_goal").value)
    node.post_global_reposition_prefer_frontier_cycles = max(
        0,
        int(node.get_parameter("post_global_reposition_prefer_frontier_cycles").value),
    )
    node.post_global_reposition_wait_for_map_update_sec = max(
        0.0,
        float(node.get_parameter("post_global_reposition_wait_for_map_update_sec").value),
    )
    node.enable_frontier_bridge_reposition = _as_bool(
        node.get_parameter("enable_frontier_bridge_reposition").value
    )
    node.frontier_bridge_min_frontier_cells = max(
        0,
        int(node.get_parameter("frontier_bridge_min_frontier_cells").value),
    )
    node.frontier_bridge_min_best_cluster_size = max(
        1,
        int(node.get_parameter("frontier_bridge_min_best_cluster_size").value),
    )
    node.frontier_bridge_min_best_distance_m = max(
        0.0,
        float(node.get_parameter("frontier_bridge_min_best_distance_m").value),
    )
    node.frontier_bridge_step_distances_m = tuple(
        max(0.1, float(value))
        for value in node.get_parameter("frontier_bridge_step_distances_m").value
    )
    node.frontier_bridge_lateral_offsets_m = tuple(
        float(value)
        for value in node.get_parameter("frontier_bridge_lateral_offsets_m").value
    )
    node.frontier_bridge_require_planner_validation = _as_bool(
        node.get_parameter("frontier_bridge_require_planner_validation").value
    )
    node.frontier_bridge_max_goal_cost = max(
        0,
        min(100, int(node.get_parameter("frontier_bridge_max_goal_cost").value)),
    )
    node.frontier_bridge_max_near_cost = max(
        0,
        min(100, int(node.get_parameter("frontier_bridge_max_near_cost").value)),
    )
    node.enable_unreachable_frontier_cooldown = _as_bool(
        node.get_parameter("enable_unreachable_frontier_cooldown").value
    )
    node.unreachable_frontier_no_safe_cycles = max(
        1,
        int(node.get_parameter("unreachable_frontier_no_safe_cycles").value),
    )
    node.unreachable_frontier_cooldown_sec = max(
        1.0,
        float(node.get_parameter("unreachable_frontier_cooldown_sec").value),
    )
    node.near_frontier_max_best_distance_m = max(
        0.0,
        float(node.get_parameter("near_frontier_max_best_distance_m").value),
    )
    node.near_frontier_min_frontier_cells = max(
        0,
        int(node.get_parameter("near_frontier_min_frontier_cells").value),
    )
    node.near_frontier_min_best_cluster_size = max(
        1,
        int(node.get_parameter("near_frontier_min_best_cluster_size").value),
    )
    node.enable_final_no_safe_viewpoint_stop = _as_bool(
        node.get_parameter("enable_final_no_safe_viewpoint_stop").value
    )
    node.final_no_safe_viewpoint_cycles = max(
        1,
        int(node.get_parameter("final_no_safe_viewpoint_cycles").value),
    )
    node.enable_stale_frontier_suppression = _as_bool(
        node.get_parameter("enable_stale_frontier_suppression").value
    )
    node.stale_frontier_region_size_m = max(
        0.1,
        float(node.get_parameter("stale_frontier_region_size_m").value),
    )
    node.stale_frontier_max_repeats = max(
        1,
        int(node.get_parameter("stale_frontier_max_repeats").value),
    )
    node.stale_frontier_cooldown_sec = max(
        1.0,
        float(node.get_parameter("stale_frontier_cooldown_sec").value),
    )
    node.stale_frontier_min_frontier_reduction = max(
        0,
        int(node.get_parameter("stale_frontier_min_frontier_reduction").value),
    )
    node.stale_frontier_min_distance_change_m = max(
        0.0,
        float(node.get_parameter("stale_frontier_min_distance_change_m").value),
    )
    node.enable_high_cost_failure_stop = _as_bool(
        node.get_parameter("enable_high_cost_failure_stop").value
    )
    node.max_consecutive_high_cost_escape_failures = max(
        1,
        int(node.get_parameter("max_consecutive_high_cost_escape_failures").value),
    )

    if node.frontier_connectivity not in (4, 8):
        node.get_logger().warn("Invalid frontier_connectivity=%d; using 8" % node.frontier_connectivity)
        node.frontier_connectivity = 8
    if node.goal_candidate_mode not in ("centroid", "frontier_cell"):
        node.get_logger().warn(
            "Invalid goal_candidate_mode='%s'; using frontier_cell" % node.goal_candidate_mode
        )
        node.goal_candidate_mode = "frontier_cell"


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def make_goal_selector_config(node) -> GoalSelectorConfig:
    return GoalSelectorConfig(
        goal_candidate_mode=node.goal_candidate_mode,
        scoring_mode=node.scoring_mode,
        use_costmap_filter=node.use_costmap_filter,
        max_allowed_cost=node.max_allowed_cost,
        reject_unknown_cost=node.reject_unknown_cost,
        safety_radius_m=node.safety_radius_m,
        viewpoint_sample_radius_m=node.viewpoint_sample_radius_m,
        viewpoint_num_samples=node.viewpoint_num_samples,
        min_goal_separation_m=node.min_goal_separation_m,
        min_candidate_distance_m=node.min_candidate_distance_m,
        min_viewpoint_distance_m=node.min_viewpoint_distance_m,
        max_candidate_distance_m=node.max_candidate_distance_m,
        max_viewpoint_distance_m=node.max_viewpoint_distance_m,
        min_goal_progress_distance_m=node.min_goal_progress_distance_m,
        min_cluster_size_for_navigation=node.min_cluster_size_for_navigation,
        max_cells_sampled_per_cluster=node.max_cells_sampled_per_cluster,
        prefer_nearest_valid_candidate=node.prefer_nearest_valid_candidate,
        w_cluster_size=node.w_cluster_size,
        w_distance=node.w_distance,
        w_information_gain=node.w_information_gain,
        w_cost_penalty=node.w_cost_penalty,
        w_goal_switching=node.w_goal_switching,
        information_radius_m=node.information_radius_m,
        prefer_farther_than_current=node.prefer_farther_than_current,
        blacklist_radius_m=node.blacklist_radius_m,
        min_information_gain_for_goal=node.min_information_gain_for_goal,
        enable_global_reposition=node.enable_global_reposition,
        global_reposition_min_distance_m=node.global_reposition_min_distance_m,
        global_reposition_max_distance_m=node.global_reposition_max_distance_m,
        global_reposition_step_m=node.global_reposition_step_m,
        global_reposition_sample_count=node.global_reposition_sample_count,
        fallback_relax_clearance=node.fallback_relax_clearance,
        fallback_safety_radius_m=node.fallback_safety_radius_m,
        fallback_reject_unknown_cost=node.fallback_reject_unknown_cost,
        max_candidates_per_cluster_for_validation=node.max_candidates_per_cluster_for_validation,
        candidate_spatial_separation_m=node.candidate_spatial_separation_m,
        enable_efficient_utility=node.enable_efficient_utility,
        max_utility_frontier_clusters=node.max_utility_frontier_clusters,
        max_utility_candidates=node.max_utility_candidates,
        max_candidates_per_region=node.max_candidates_per_region,
        region_grid_size_m=node.region_grid_size_m,
        enable_region_diversity=node.enable_region_diversity,
        recent_goal_region_penalty=node.recent_goal_region_penalty,
        rejected_region_penalty=node.rejected_region_penalty,
        region_memory_timeout_sec=node.region_memory_timeout_sec,
        w_path_entropy=node.w_path_entropy,
        w_region_diversity=node.w_region_diversity,
        w_path_length_penalty=node.w_path_length_penalty,
        w_uncertainty_penalty=node.w_uncertainty_penalty,
        entropy_window_radius_m=node.entropy_window_radius_m,
        path_entropy_sample_step_m=node.path_entropy_sample_step_m,
        normalize_utility_scores=node.normalize_utility_scores,
        max_utility_candidate_markers=node.max_utility_candidate_markers,
        enable_utility_fallback_to_baseline=node.enable_utility_fallback_to_baseline,
        utility_fallback_when_no_candidates=node.utility_fallback_when_no_candidates,
        utility_min_safe_candidates_before_ranking=node.utility_min_safe_candidates_before_ranking,
        utility_bootstrap_min_frontier_clusters=node.utility_bootstrap_min_frontier_clusters,
        utility_bootstrap_min_robot_travel_m=node.utility_bootstrap_min_robot_travel_m,
        enable_bootstrap_exploration=node.enable_bootstrap_exploration,
        bootstrap_max_cycles_without_goal=node.bootstrap_max_cycles_without_goal,
        bootstrap_min_goal_distance_m=node.bootstrap_min_goal_distance_m,
        bootstrap_max_goal_distance_m=node.bootstrap_max_goal_distance_m,
        bootstrap_allow_relaxed_clearance=node.bootstrap_allow_relaxed_clearance,
        bootstrap_safety_radius_m=node.bootstrap_safety_radius_m,
        bootstrap_use_known_free_space=node.bootstrap_use_known_free_space,
        progress_gate_enabled=node.progress_gate_enabled,
        progress_gate_min_distance_m=node.progress_gate_min_distance_m,
        progress_gate_timeout_sec=node.progress_gate_timeout_sec,
        progress_gate_max_skip_cycles=node.progress_gate_max_skip_cycles,
        progress_gate_disable_after_escape=node.progress_gate_disable_after_escape,
        post_escape_resume_delay_sec=node.post_escape_resume_delay_sec,
        post_escape_force_selection_cycles=node.post_escape_force_selection_cycles,
        global_reposition_max_consecutive_goals=node.global_reposition_max_consecutive_goals,
        global_reposition_cooldown_sec=node.global_reposition_cooldown_sec,
        global_reposition_recent_goal_radius_m=node.global_reposition_recent_goal_radius_m,
        global_reposition_recent_region_penalty=node.global_reposition_recent_region_penalty,
        global_reposition_min_information_gain=node.global_reposition_min_information_gain,
        global_reposition_allow_zero_gain_only_if_no_alternative=node.global_reposition_allow_zero_gain_only_if_no_alternative,
        global_reposition_blacklist_after_success_sec=node.global_reposition_blacklist_after_success_sec,
        global_reposition_pingpong_window=node.global_reposition_pingpong_window,
        global_reposition_pingpong_radius_m=node.global_reposition_pingpong_radius_m,
        recent_goal_region_timeout_sec=node.recent_goal_region_timeout_sec,
        recent_goal_region_radius_m=node.recent_goal_region_radius_m,
        enable_goal_usefulness_gate=node.enable_goal_usefulness_gate,
        min_goal_information_gain=node.min_goal_information_gain,
        min_goal_frontier_distance_from_recent_m=node.min_goal_frontier_distance_from_recent_m,
        min_expected_frontier_reduction=node.min_expected_frontier_reduction,
        allow_low_gain_recovery_goal=node.allow_low_gain_recovery_goal,
        post_global_reposition_prefer_frontier_cycles=node.post_global_reposition_prefer_frontier_cycles,
        post_global_reposition_wait_for_map_update_sec=node.post_global_reposition_wait_for_map_update_sec,
        enable_frontier_bridge_reposition=node.enable_frontier_bridge_reposition,
        frontier_bridge_min_frontier_cells=node.frontier_bridge_min_frontier_cells,
        frontier_bridge_min_best_cluster_size=node.frontier_bridge_min_best_cluster_size,
        frontier_bridge_min_best_distance_m=node.frontier_bridge_min_best_distance_m,
        frontier_bridge_step_distances_m=node.frontier_bridge_step_distances_m,
        frontier_bridge_lateral_offsets_m=node.frontier_bridge_lateral_offsets_m,
        frontier_bridge_require_planner_validation=node.frontier_bridge_require_planner_validation,
        frontier_bridge_max_goal_cost=node.frontier_bridge_max_goal_cost,
        frontier_bridge_max_near_cost=node.frontier_bridge_max_near_cost,
    )


def make_planner_validation_config(node) -> PlannerValidationConfig:
    return PlannerValidationConfig(
        use_planner_validation=node.use_planner_validation,
        planner_action_name=node.planner_action_name,
        planner_id=node.planner_id,
        planner_validation_timeout_sec=node.planner_validation_timeout_sec,
        min_valid_path_length_m=node.min_valid_path_length_m,
        max_valid_path_length_m=node.max_valid_path_length_m,
        max_path_cost=node.max_path_cost,
        reject_path_unknown=node.reject_path_unknown,
        path_check_step_m=node.path_check_step_m,
        path_clearance_radius_m=node.path_clearance_radius_m,
        max_planner_validation_candidates=node.max_planner_validation_candidates,
        planner_validation_required_for_navigation=node.planner_validation_required_for_navigation,
        planner_validation_retry_next_best=node.planner_validation_retry_next_best,
        planner_validation_max_batches=node.planner_validation_max_batches,
        planner_validation_batch_size=node.planner_validation_batch_size,
        fallback_relax_path_clearance=node.fallback_relax_path_clearance,
        fallback_path_clearance_radius_m=node.fallback_path_clearance_radius_m,
        high_cost_escape_validation_mode=node.high_cost_escape_validation_mode,
        high_cost_escape_ignore_start_radius_m=node.high_cost_escape_ignore_start_radius_m,
        high_cost_escape_path_clearance_radius_m=node.high_cost_escape_path_clearance_radius_m,
        high_cost_escape_allow_initial_high_cost=node.high_cost_escape_allow_initial_high_cost,
        enable_efficient_utility=node.enable_efficient_utility,
        path_entropy_sample_step_m=node.path_entropy_sample_step_m,
        path_clearance_max_near_cost=node.path_clearance_max_near_cost,
        path_clearance_lethal_cost=node.path_clearance_lethal_cost,
        allow_low_inflation_near_path=node.allow_low_inflation_near_path,
        low_inflation_cost_threshold=node.low_inflation_cost_threshold,
        normal_path_ignore_start_radius_m=node.normal_path_ignore_start_radius_m,
    )


def make_high_cost_escape_config(node) -> HighCostEscapeConfig:
    return HighCostEscapeConfig(
        enabled=node.high_cost_escape_enabled,
        robot_cost_threshold=node.high_cost_robot_threshold,
        sample_radius_m=node.high_cost_escape_radius_m,
        sample_count=node.high_cost_escape_samples,
        max_allowed_cost=node.max_allowed_cost,
        reject_unknown_cost=node.reject_unknown_cost,
        safety_radius_m=node.safety_radius_m,
        validation_mode=node.high_cost_escape_validation_mode,
        ignore_start_radius_m=node.high_cost_escape_ignore_start_radius_m,
        path_clearance_radius_m=node.high_cost_escape_path_clearance_radius_m,
        allow_initial_high_cost=node.high_cost_escape_allow_initial_high_cost,
        max_goal_distance_m=node.high_cost_escape_max_goal_distance_m,
        min_cost_drop=node.high_cost_escape_min_cost_drop,
        require_cost_decrease=node.high_cost_escape_require_cost_decrease,
        max_attempts_per_cycle=node.high_cost_escape_max_attempts_per_cycle,
    )
