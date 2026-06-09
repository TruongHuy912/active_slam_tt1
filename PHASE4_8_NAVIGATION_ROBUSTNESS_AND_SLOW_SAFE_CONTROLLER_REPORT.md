# Phase 4.8 Navigation Robustness And Slow Safe Controller Report

## 1. Summary

Phase 4.8 improves robustness before moving to entropy scoring:

- treats `no_path` as an unreachable planner reject, not just a transient failure
- caches planner rejects by reason and cluster
- suppresses per-candidate blacklist-hit spam and emits cycle summaries
- adds medium reposition between local goals and global reposition
- adds progress/stuck monitoring with optional goal cancel/blacklist
- adds high-cost robot diagnostics and a local escape candidate
- slows the current Regulated Pure Pursuit controller slightly for safer narrow-space behavior

No Phase 5 path entropy, SLAM uncertainty, MPPI, roadmap/TSP, or copied reference code was added.

## 2. Why Not Phase 5 Path Entropy Yet

The current issue is robustness: repeated `no_path`, stale bad clusters, high-cost stuck states, and controller speed in narrow regions. Path entropy would rank information value but would not fix unreachable candidates or controller tracking. Entropy should come after the navigation loop stops repeating invalid candidates.

## 3. Why MPPI Was Not Ported

MPPI is useful only after goals and paths are already valid. It does not solve:

- `no_path`
- unreachable frontier clusters
- bad candidate selection
- missing costmap obstacles
- global paths through unknown/high cost

This phase keeps the current RPP controller and only applies a conservative speed tune. A later dedicated phase can compare current RPP vs RotationShim+MPPI.

## 4. Root Cause From Test 2/3

Observed symptoms:

- many `planner_reject_blacklist_hit` lines
- `NavigateToPose result: FAILED status=6`
- repeated `no_path` candidates/clusters
- local and global reposition sometimes both failed with `rejected_no_path`
- robot sometimes entered high-cost regions and got stuck

Root causes addressed:

- reject cache did not handle `no_path` strongly enough
- blacklist hit logging was per-candidate, causing spam
- local candidates could remain dominant even after planner failure
- no progress monitor existed to cancel a goal that was accepted but not making progress
- controller speed was still relatively aggressive for tight cluttered areas

## 5. Files Changed

Updated:

- `bumperbot_active_slam/planner_reject_cache.py`
- `bumperbot_active_slam/planner_validator.py`
- `bumperbot_active_slam/goal_selector.py`
- `bumperbot_active_slam/navigation_dispatcher.py`
- `bumperbot_active_slam/costmap_utils.py`
- `bumperbot_active_slam/node_params.py`
- `bumperbot_active_slam/active_slam_node.py`
- `bumperbot_active_slam/config/active_slam.yaml`
- `bumperbot_navigation/config/controller_server.yaml`

New:

- `bumperbot_active_slam/progress_monitor.py`

## 6. Module Boundaries

`active_slam_node.py` still acts as orchestration:

- calls selector/validator/dispatcher
- checks progress monitor
- chooses fallback tier calls
- publishes markers

New policy logic is in modules:

- `planner_reject_cache.py`: reason-aware reject cache and hit summaries
- `planner_validator.py`: no-path/cost/unknown/clearance reject callback
- `goal_selector.py`: medium reposition candidate generation
- `progress_monitor.py`: stuck detection
- `costmap_utils.py`: high-cost diagnostics and safe escape sampling
- `navigation_dispatcher.py`: active goal cancel helper

The node grew modestly because it wires the new modules together, but large logic remains outside the node.

## 7. no_path Reject Cache Logic

`no_path` is now cached like clearance/cost/unknown:

```yaml
planner_reject_reasons_to_cache: ["clearance", "no_path", "cost", "unknown"]
no_path_blacklist_timeout_sec: 60.0
no_path_blacklist_radius_m: 1.0
no_path_cluster_fail_threshold: 2
no_path_cluster_timeout_sec: 90.0
```

Expected logs:

```text
planner_reject_cache_added: reason=no_path cluster_id=...
planner_reject_cluster_blacklisted: reason=no_path cluster_id=...
planner_reject_cache_summary: blacklist_hits_total=... hits_by_reason={...}
```

## 8. Blacklist Log Throttling

Per-candidate hit logs are disabled by default:

```yaml
log_individual_blacklist_hits: false
max_blacklist_hit_logs_per_cycle: 5
```

Instead, each selection cycle can emit:

```text
planner_reject_cache_summary: blacklist_hits_total=... hits_by_reason={...} top_blacklisted_clusters={...}
```

This avoids flooding logs when hundreds of candidates are skipped.

## 9. Progressive Selection Tiers

Current tier order:

1. local safe viewpoint candidates
2. medium reposition candidates in nearby known free space
3. global reposition toward farther frontiers
4. high-cost escape candidate if robot is already in a high-cost area

New config:

```yaml
selection_tier_mode: progressive
enable_medium_reposition: true
medium_reposition_min_distance_m: 1.0
medium_reposition_max_distance_m: 3.0
medium_reposition_sample_count: 24
enable_rotate_recovery_goal: true
rotate_recovery_when_no_valid_goal: true
```

Rotate recovery is configured for the policy layer but not implemented as a Nav2 behavior dispatch in this phase.

## 10. Progress And Stuck Handling

New monitor:

```yaml
progress_check_period_sec: 3.0
min_progress_distance_m: 0.15
stuck_timeout_sec: 12.0
cancel_goal_on_stuck: true
blacklist_goal_on_stuck: true
```

Expected logs:

```text
progress_monitor: moved=... in ... sec
stuck_detected: canceling current goal
Canceling active NavigateToPose goal: reason=stuck_detected
```

When stuck, the active goal is canceled and optionally blacklisted.

## 11. High-Cost Region Handling

New diagnostics and escape sampling:

```yaml
high_cost_robot_threshold: 70
high_cost_escape_enabled: true
high_cost_escape_radius_m: 0.8
high_cost_escape_samples: 24
```

Expected logs:

```text
costmap_robot_status: global_max_cost_near_robot=... threshold=70
high_cost_escape: robot_cost=... selected_escape=(x, y)
```

This does not replace planner validation. The escape candidate still goes through planner validation before dispatch.

## 12. Controller Speed Tuning

Controller audited:

- file: `Bumper-Bot-main/bumperbot_navigation/config/controller_server.yaml`
- plugin: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

Changed values:

```yaml
desired_linear_vel: 0.3 -> 0.22
rotate_to_heading_angular_vel: 1.8 -> 1.2
min_approach_linear_velocity: 0.05 -> 0.04
regulated_linear_scaling_min_speed: 0.25 -> 0.18
max_angular_accel: 3.2 -> 2.4
```

Reason:

- reduce wall/door clipping risk in narrow areas
- reduce aggressive turns
- keep the same controller and costmaps

Rollback:

- restore the old numeric values above in `controller_server.yaml`
- rebuild/source if needed

No planner/costmap parameters were changed.

## 13. MPPI Audit

MPPI should be considered later only if:

1. candidate is planner-valid
2. path avoids high cost/unknown/low-clearance cells
3. `/scan` and costmaps show obstacles correctly
4. robot still collides while tracking a valid path

Suggested later phase:

```text
Phase Controller Experimental: compare current RPP vs RotationShim+MPPI
```

No MPPI code was ported in Phase 4.8.

## 14. Ideas From References

From `m-explore-ros2`:

- blacklist unreachable/failed goals
- avoid retrying invalid frontiers indefinitely
- use progress timeout/recovery after failed goal

From `roadmap-explorer`:

- avoid greedy local selection
- use intermediate/global reposition ideas
- diversify candidates instead of validating one tight area

No source code was copied.

## 15. Test Commands

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_navigation
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Runtime:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
```

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

## 16. Expected Logs

Reduced blacklist spam:

```text
planner_reject_cache_summary: blacklist_hits_total=... hits_by_reason={...}
```

No-path cache:

```text
planner_reject_cache_added: reason=no_path cluster_id=...
planner_reject_cluster_blacklisted: reason=no_path cluster_id=...
```

Progressive fallback:

```text
local planner validation failed, trying medium_reposition
local planner validation failed, trying global_reposition
```

Stuck handling:

```text
progress_monitor: moved=... in ... sec
stuck_detected: canceling current goal
```

High-cost escape:

```text
costmap_robot_status: global_max_cost_near_robot=...
high_cost_escape: robot_cost=... selected_escape=(...)
```

## 17. Known Limitations

- No path entropy yet.
- No SLAM uncertainty yet.
- No full roadmap/TSP.
- Rotate recovery is configured but not dispatched as a Nav2 behavior yet.
- Medium/global reposition still depends on Nav2 planner and costmap quality.
- Controller tuning is conservative but not a replacement for MPPI experiments.

## 18. Verification

Build result:

```text
Summary: 2 packages finished
```

Syntax/import smoke test:

```text
phase48 imports ok
```

Runtime was not executed in this turn; validate in `small_house` with `enable_navigation:=true`.
