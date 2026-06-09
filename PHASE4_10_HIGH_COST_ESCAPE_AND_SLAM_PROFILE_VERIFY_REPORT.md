# Phase 4.10 High-Cost Escape And SLAM Profile Verify Report

## 1. Summary

Phase 4.10 fixes the case where the robot is already inside an inflated/high-cost region and every escape path is rejected immediately at the start pose.

Changes made:

- Added a dedicated `high_cost_escape.py` policy module.
- Added high-cost escape validation parameters.
- Added path safety support for `ignore_start_radius_m`.
- Made planner validation use a smaller clearance radius for `high_cost_escape` candidates.
- Prevented heavy planner reject cache/cluster blacklist behavior for synthetic cluster `-2`.
- Extended `slam_turning_diagnostics.sh` to report whether runtime SLAM params match the `turning_stable` profile.

No Phase 5 path entropy, MPPI, roadmap/TSP, SLAM uncertainty scoring, or copied reference code was added.

## 2. Why Phase 5 Is Still Blocked

The current failure is recovery feasibility, not exploration value ranking. The robot can enter a high-cost region, and the validator can reject every possible escape because the path starts inside inflated cost. Entropy would not solve that. Phase 5 should wait until high-cost recovery works and the SLAM profile being used at runtime is confirmed.

## 3. SLAM Profile Verification

Launch audit:

- `bumperbot_bringup/launch/simulated_robot.launch.py` exposes `slam_config`.
- It passes `slam_config` into `bumperbot_mapping/launch/slam.launch.py`.
- `slam.launch.py` passes that YAML path directly to `sync_slam_toolbox_node`.

Use the turning-stable profile explicitly:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Runtime verification:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh
```

The script now reads these live params from `/slam_toolbox`:

- `map_update_interval`
- `minimum_time_interval`
- `minimum_travel_distance`
- `minimum_travel_heading`
- `throttle_scans`
- `transform_publish_period`
- `scan_topic`
- `base_frame`
- `odom_frame`
- `map_frame`

If runtime params are:

```yaml
map_update_interval: 1.0
minimum_time_interval: 0.2
minimum_travel_distance: 0.2
minimum_travel_heading: 0.15
```

the script prints:

```text
OK: SLAM profile appears to be turning_stable
```

If runtime params are still default:

```yaml
map_update_interval: 2.0
minimum_time_interval: 0.5
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
```

the script warns that the default profile is active. The default remains unchanged for Raspberry Pi safety.

## 4. High-Cost Stuck Root Cause

The observed log:

```text
costmap_robot_status: global_max_cost_near_robot=88 threshold=70
high_cost_escape: robot_cost=88 selected_escape=(...)
Planner candidate rejected ... reason=path_clearance max_cost_near_path=96
```

means the robot starts inside inflated/high cost. Normal path safety checks then reject the returned path at or near the first sample, before the path has a chance to leave the inflated region. This is too strict for escape behavior.

The fix is not to ignore obstacles globally. It is to ignore only the initial high-cost radius around the robot for escape paths, then keep normal lethal/unknown/high-cost checks after that radius.

## 5. Files Changed

New:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/high_cost_escape.py`

Updated:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/path_safety.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/planner_validator.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/models.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/node_params.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/planner_reject_cache.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh`

## 6. active_slam_node.py Size

Before Phase 4.10:

```text
846 lines
```

After Phase 4.10:

```text
854 lines
```

Increase:

```text
8 lines
```

The added lines only wire the new `HighCostEscapePolicy` into the existing orchestration flow. The escape algorithm, sampling, cost checks, path ignore-start logic, and planner validation behavior are not implemented inside `active_slam_node.py`.

## 7. Logic Placement

New logic locations:

- `high_cost_escape.py`
  - detects high-cost robot state
  - samples short lower-cost escape candidates
  - enforces cost drop and max escape distance
- `path_safety.py`
  - adds `ignore_start_radius_m`
  - reports `first_checked_world`, `robot_cost`, `goal_cost`, and ignored start samples
- `planner_validator.py`
  - applies high-cost escape validation mode only for `candidate.source == "high_cost_escape"`
  - uses smaller escape clearance radius
  - logs ignore-start diagnostics
- `planner_reject_cache.py`
  - adds cache clear helper for high-cost recovery
- `models.py`
  - adds `HighCostEscapeConfig`
- `node_params.py` and `active_slam.yaml`
  - declare/load Phase 4.10 parameters
- `slam_turning_diagnostics.sh`
  - verifies live SLAM profile params

This keeps `active_slam_node.py` as orchestration: params, subscribers, TF, frontier detection, selector, validator, dispatcher, markers, and high-level logging.

## 8. High-Cost Escape Validation

New config:

```yaml
high_cost_escape_validation_mode: true
high_cost_escape_ignore_start_radius_m: 0.35
high_cost_escape_path_clearance_radius_m: 0.10
high_cost_escape_allow_initial_high_cost: true
high_cost_escape_max_goal_distance_m: 1.2
high_cost_escape_min_cost_drop: 20
high_cost_escape_require_cost_decrease: true
high_cost_escape_max_attempts_per_cycle: 8
recovery_wait_for_costmap_update_sec: 2.0
recovery_clear_reject_cache_when_robot_high_cost: true
```

Behavior:

- If robot max nearby cost is above `high_cost_robot_threshold`, high-cost escape overrides frontier goals.
- Escape candidates are short, at most `1.2 m`.
- The selected escape must reduce cost by at least `20` when cost data is available.
- Escape endpoints still need to be safe.
- Planner validation ignores only the first `0.35 m` around the robot for high-cost escape paths.
- After the ignored start radius, path cost/unknown/lethal/clearance checks still apply.
- Synthetic high-cost escape candidates use `cluster_id=-2`; failures are not fed into the frontier cluster reject cache.

Expected logs:

```text
costmap_robot_status: global_max_cost_near_robot=88 threshold=70
high_cost_escape: robot_cost=88 goal_cost=... selected_escape=(...) distance=...
Planner validation started: source=high_cost_escape ...
Planner candidate rejected: ... ignore_start_radius_m=0.35 ... robot_cost=88 goal_cost=...
```

If valid:

```text
Planner validation accepted: source=high_cost_escape ...
Sending NavigateToPose goal ...
```

If not valid:

```text
high_cost_escape_failed: no lower-cost planner-valid escape reason=...
Navigation skip: ... reason=planner validation failed; no planner-valid candidate
```

## 9. ignore_start_radius_m Path Safety

Normal paths still use:

```yaml
ignore_start_radius_m: 0.0
```

High-cost escape paths use:

```yaml
ignore_start_radius_m: 0.35
path_clearance_radius_m: 0.10
```

This prevents false rejection caused only by the robot's current inflated-cost start cell. It does not allow driving through lethal obstacles farther along the path.

## 10. Test Commands

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_mapping bumperbot_bringup
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Runtime with stable SLAM profile:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Verify SLAM params:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh
```

Start exploration:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

## 11. Expected Logs

Stable profile:

```text
OK: SLAM profile appears to be turning_stable
```

High-cost escape:

```text
costmap_robot_status: global_max_cost_near_robot=... threshold=70
high_cost_escape: robot_cost=... goal_cost=... selected_escape=(...)
Planner validation started: source=high_cost_escape
```

Path safety diagnostics:

```text
ignore_start_radius_m=0.35
ignored_start_samples=...
first_checked_pose_after_start=(...)
robot_cost=...
goal_cost=...
```

Success:

```text
Planner validation accepted: source=high_cost_escape ...
NavigateToPose goal accepted ...
```

Failure:

```text
high_cost_escape_failed: no lower-cost planner-valid escape
```

There should be no repeated `planner_reject_cluster_blacklisted` spam for `cluster_id=-2`.

## 12. Phase 5 Gate

Move to Phase 5 only after:

- runtime diagnostics confirm whether `turning_stable` or default SLAM profile is active
- high-cost escape can find a planner-valid lower-cost escape in typical stuck cases
- robot does not remain stuck in inflated/high-cost regions for long periods
- map updates are not visibly stale around recently visited regions
- active_slam can complete many goals without repeated `no_path`/clearance loops
- `active_slam_node.py` does not grow further with policy logic

## 13. Verification

Build:

```text
Summary: 3 packages finished
```

Syntax/import smoke:

```text
phase410 imports ok
```

Runtime Gazebo test was not executed in this turn. Use the runtime commands above to verify actual high-cost escape behavior in `small_house`.
