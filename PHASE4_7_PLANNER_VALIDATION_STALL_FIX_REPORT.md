# Phase 4.7 Planner Validation Stall Fix Report

## 1. Summary

Phase 4.7 fixes the stall where Active SLAM repeatedly selected the same local candidate, Nav2 planner validation rejected all top candidates by clearance, and no new goal was sent.

The fix adds:

- planner validation batching across multiple candidate groups
- candidate diversity before planner validation
- planner reject blacklist/cache
- cluster-level short timeout after repeated planner rejects
- relaxed path-clearance fallback
- global reposition validation after local planner validation fails
- detailed diagnostics for clearance rejects
- RViz markers for planner rejected candidates and planner reject cache

No path entropy, SLAM uncertainty, MPPI, controller tuning, planner tuning, or copied reference code was added.

## 2. Root Cause Of Repeated Goal Calculation

The runtime log showed:

```text
local_candidates=538
candidates_before_planner_validation=20
validated=20
rejected_clearance=20
selected=false
```

Then the same candidate came back:

```text
selected_cluster_id=48
selected_world=(3.23, 2.15)
```

Root cause:

- `goal_selector.py` generated many local candidates, but planner validation only considered the top group.
- Those top candidates were spatially/cluster-wise similar.
- `planner_validator.py` reported clearance failure but did not feed the rejection back into selection.
- The next timer cycle therefore selected the same cluster/candidate again.
- `global_reposition` did not run because local candidates existed, even though all local candidates failed planner validation.

Phase 4.7 feeds planner rejects back into selection and tries broader candidate batches plus global reposition before giving up.

## 3. Files Changed

New files:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/planner_reject_cache.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/candidate_diversity.py`

Updated files:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/goal_selector.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/planner_validator.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/path_safety.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/marker_utils.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/models.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/node_params.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`

## 4. Module Boundaries

`active_slam_node.py` remains orchestration-only:

- receives map/costmap
- calls frontier detection
- calls `GoalSelector`
- calls `PlannerValidator`
- calls `NavigationDispatcher`
- publishes markers

New logic lives in:

- `planner_validator.py`
  - batched validation
  - relaxed clearance retry
  - rejection summaries
  - rejection callbacks
- `path_safety.py`
  - first rejected path sample diagnostics
  - path sample count
  - max cost near path
- `planner_reject_cache.py`
  - candidate and cluster short blacklist
- `candidate_diversity.py`
  - per-cluster and spatial diversity for validation candidates
- `goal_selector.py`
  - skips candidates/clusters hit by planner reject cache
  - exposes explicit global reposition selection for planner-fail fallback
- `marker_utils.py`
  - planner rejected candidate and planner reject cache markers

`active_slam_node.py` is currently about 739 lines, but parameter declaration/loading is already split into `node_params.py`; the new behavior is not embedded in the node.

## 5. Clearance Reject Diagnostics

For the first few rejected candidates per validation run, logs now include:

- candidate x/y
- `path_clearance_radius_m`
- `fallback_path_clearance_radius_m`
- path length
- number of path samples checked
- first rejected world coordinate
- first rejected map cell
- detailed reason:
  - `obstacle_cost`
  - `inflated_cost`
  - `unknown`
  - `out_of_bounds`
  - `clearance_radius_hit`
- max cost on path
- max cost near path
- costmap frame, resolution, and size
- path frame

This is intended to explain whether `rejected_clearance=20` is caused by inflated obstacles, unknown cells, out-of-bounds costmap coverage, or strict clearance radius.

## 6. Planner Validation Batching And Fallback

New behavior:

- candidates are diversified before validation
- validation runs through multiple batches
- each batch is logged
- top candidates are not allowed to come only from one tight cluster region

New config:

```yaml
planner_validation_retry_next_best: true
planner_validation_max_batches: 4
planner_validation_batch_size: 20
max_candidates_per_cluster_for_validation: 5
candidate_spatial_separation_m: 0.5
```

With defaults, validation can inspect up to 80 diversified candidates in a cycle.

Expected batch log:

```text
Planner validation batch: source=local planner_validation_batch_index=1 candidates_in_batch=20
Planner validation batch: source=local planner_validation_batch_index=2 candidates_in_batch=20
```

## 7. Planner Reject Blacklist/Cache

If a candidate path fails planner validation due to clearance, cost, or unknown:

- candidate point is added to `planner_reject_cache`
- nearby candidates in the same cluster are skipped for a short timeout
- repeated cluster failures temporarily blacklist that cluster

New config:

```yaml
blacklist_on_planner_reject: true
planner_reject_blacklist_timeout_sec: 45.0
planner_reject_blacklist_radius_m: 0.8
planner_reject_cluster_fail_threshold: 3
planner_reject_cluster_timeout_sec: 45.0
```

Expected logs:

```text
planner_reject_blacklist_added: cluster_id=48 point=(3.23, 2.15) radius=0.80 reason=path_clearance
planner_reject_blacklist_hit: cluster_id=48 point=(...)
planner_reject_cluster_blacklisted: cluster_id=48 failures=3 timeout=45.0
expired planner reject blacklist: cluster_id=48 point=(...)
```

## 8. Global Reposition After Planner Fail

Previously `global_reposition` only ran when no local candidates existed. That missed the important case:

```text
local candidates exist, but every local path is planner-invalid
```

Now:

1. Local candidates are generated.
2. Planner validation checks diversified local batches.
3. If local validation fails and `enable_global_reposition_after_planner_fail=true`, the node explicitly generates global reposition candidates.
4. Global reposition candidates are planner-validated.
5. If one is valid, it is sent.
6. If none are valid, the node logs no planner-valid candidate and waits before retrying.

New config:

```yaml
enable_global_reposition_after_planner_fail: true
```

Expected log:

```text
local planner validation failed, trying global_reposition
Planner validation started: source=global_reposition ...
```

## 9. Relaxed Clearance Fallback

If strict path clearance rejects a candidate, validator retries the same returned path with a smaller clearance radius:

```yaml
fallback_relax_path_clearance: true
fallback_path_clearance_radius_m: 0.15
```

This fallback does not ignore lethal costs or unknown cells. It only relaxes the clearance radius. If the path crosses high cost, unknown, or out-of-bounds cells, it is still rejected.

Expected logs:

```text
strict_validation_failed=True
trying_relaxed_path_clearance=True
relaxed_selected=True
safety_radius_used=0.15
```

## 10. Marker Debug

New marker namespaces:

- `planner_valid_path`
- `planner_rejected_candidates`
- `planner_reject_blacklist`

Limits:

```yaml
max_planner_rejected_markers: 20
```

Markers are capped to avoid RViz lag.

## 11. Test Commands

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam
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

## 12. Expected Logs

When a local batch fails:

```text
Planner validation batch: source=local planner_validation_batch_index=1 candidates_in_batch=20
Planner candidate rejected: source=local cluster_id=... candidate=(...) path_clearance_radius_m=0.22 ...
planner_reject_blacklist_added: cluster_id=... point=(...) radius=0.80 reason=path_clearance
Planner validation batch: source=local planner_validation_batch_index=2 candidates_in_batch=20
```

If local validation fails completely:

```text
Planner validation finished: source=local ... rejected_clearance=... selected=false
local planner validation failed, trying global_reposition
Planner validation started: source=global_reposition ...
```

If no valid path exists:

```text
Navigation skip: state=... enable_navigation=True reason=planner validation failed; no planner-valid candidate; waiting before retry
```

The same `selected_world=(3.23, 2.15)` should not be selected indefinitely while its planner reject cache is active.

## 13. When To Consider MPPI

Do not use MPPI to mask invalid global goals or paths. When planner validation rejects every candidate by clearance, this is still a goal/path feasibility issue, not a controller issue.

Consider RotationShim + MPPI only after:

1. a candidate is planner-valid
2. the returned path does not cross unknown/high-cost/low-clearance cells
3. `/scan` and costmaps correctly show walls/doors
4. the robot still collides while tracking the valid path

## 14. Why Phase 5 Path Entropy Was Not Added

Path entropy would rank exploration value, but this bug is candidate/path feasibility. The selector must first avoid repeatedly proposing planner-invalid candidates. Entropy can be added later once the candidate set and validation loop are stable.

## 15. Verification

Build:

```text
Summary: 1 package finished
```

Syntax and import smoke test:

```text
phase47 imports ok
```

Runtime was not executed in this turn; the expected runtime logs above should be checked in `small_house`.
