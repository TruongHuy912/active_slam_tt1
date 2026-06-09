# Phase 4 Costmap Safe Viewpoint And Map Update Report

## 1. Summary

Phase 4 adds costmap-aware safe viewpoint sampling to `bumperbot_active_slam`.

The node no longer sends Nav2 goals directly to raw frontier cells. It now:

- subscribes to the Nav2 global costmap
- samples safe viewpoints around sampled frontier cells
- rejects candidates by map free-space, costmap cost, clearance radius,
  blacklist, distance, and progress gates
- scores valid viewpoints with a simple weighted utility
- publishes debug markers for selected, valid, rejected, and blacklisted
  candidates

No path entropy, SLAM uncertainty, full roadmap/TSP scheduling, or reference
repo code was ported.

## 2. Current Baseline Before Phase

Before this phase:

- Nav2 lifecycle was active.
- `/navigate_to_pose` had `Action servers: 1 /bt_navigator`.
- `active_slam_explorer enable_navigation:=true` could send goals.
- Nav2 accepted goals and returned `SUCCEEDED`.
- The selected goal was still too local because it came from frontier-cell
  candidates near the robot.

Example weakness:

```text
selected_distance=0.64
NavigateToPose goal accepted
NavigateToPose result: SUCCEEDED
selected_distance=0.62
```

This proved the pipeline worked but also showed goal selection was too
short-horizon and not costmap-aware.

## 3. Ideas Taken From `m-explore-ros2`

Only ideas were used, not code.

Useful ideas:

- Use Nav2 costmap data as an exploration planning source when available.
- Score frontiers with a distance term and a gain term.
- Track progress toward a goal and avoid repeatedly sending the same goal.
- Blacklist failed or unreachable frontier goals.
- Keep debug markers for frontiers and chosen goals.

Simplification for Bumper-Bot:

- Kept our Python ROS 2 node and existing state machine.
- Kept `NavigateToPose` action handling already implemented.
- Used a lightweight local scoring model instead of the full C++ frontier
  search/costmap client implementation.

## 4. Ideas Taken From `roadmap-explorer`

Only ideas were used, not code.

Useful ideas:

- Avoid purely greedy nearest-frontier behavior.
- Prefer meaningful viewpoints instead of raw frontier centroids/cells.
- Include local information gain in candidate scoring.
- Keep session/state concepts such as failed frontier memory and progress gates.
- Use markers and explicit feedback to make exploration behavior inspectable.

Simplification for Bumper-Bot:

- No roadmap graph.
- No TSP/global scheduling.
- No pose-graph alignment.
- No 3D/camera FOV model.
- Local unknown-cell counting is used as a lightweight information-gain proxy.

## 5. What Was Intentionally Not Ported

- No code copied from `m-explore-ros2`.
- No code copied from `roadmap-explorer`.
- No custom roadmap planner.
- No TSP solver.
- No Fisher information or pose-graph uncertainty.
- No path entropy utility from the paper.
- No Nav2 planner/controller changes.

## 6. Files Changed

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/costmap_utils.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/README.md`
- `Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox.yaml`

## 7. New Config Parameters

```yaml
use_costmap_filter: true
costmap_topic: /global_costmap/costmap
max_allowed_cost: 70
reject_unknown_cost: true
safety_radius_m: 0.25
viewpoint_sample_radius_m: 0.45
viewpoint_num_samples: 16
min_viewpoint_distance_m: 0.8
max_viewpoint_distance_m: 4.0
min_goal_progress_distance_m: 0.4
min_cluster_size_for_navigation: 5
scoring_mode: safe_viewpoint
w_cluster_size: 1.0
w_distance: 0.5
w_information_gain: 1.0
w_cost_penalty: 1.0
w_goal_switching: 0.5
information_radius_m: 0.6
prefer_farther_than_current: true
max_candidate_markers: 40
max_rejected_markers: 40
```

Existing candidate params remain:

```yaml
goal_candidate_mode: frontier_cell
min_candidate_distance_m: 0.6
max_candidate_distance_m: 6.0
max_cells_sampled_per_cluster: 50
prefer_nearest_valid_candidate: true
```

## 8. Costmap Safety Checks

`costmap_utils.py` provides:

- `world_to_costmap()`
- `get_cost_at_world()`
- `is_cost_safe()`
- `is_pose_safe()`
- `check_clearance_radius()`
- `summarize_costmap_status()`

Safety behavior:

- Reject unknown cost when `reject_unknown_cost=true`.
- Reject costs greater than `max_allowed_cost`.
- Reject poses whose `safety_radius_m` neighborhood contains unsafe cells.
- If the costmap has not arrived, the node logs a warning and falls back to the
  occupancy map free-space check.

## 9. Safe Viewpoint Sampling Algorithm

For each frontier cluster:

1. Ignore clusters smaller than `min_cluster_size_for_navigation`.
2. Sample up to `max_cells_sampled_per_cluster` frontier cells.
3. For each frontier cell, create viewpoints:
   - one point stepped from frontier toward the robot
   - `viewpoint_num_samples` circular samples around the frontier cell
4. Reject viewpoints outside `[min_viewpoint_distance_m, max_viewpoint_distance_m]`.
5. Reject viewpoints not free in `/map`.
6. Reject viewpoints unsafe in `/global_costmap/costmap`.
7. Reject blacklisted or too-close-to-last-goal viewpoints.
8. Score all remaining viewpoints and select the highest score.

The selected goal sent to Nav2 is now the safe viewpoint, not the frontier cell
or cluster centroid.

## 10. Scoring Formula

Implemented scoring:

```text
score =
  w_cluster_size * normalized_cluster_size
  + w_distance * distance_score
  + w_information_gain * local_unknown_gain
  - w_cost_penalty * cost_penalty
  - w_goal_switching * goal_switch_penalty
```

Notes:

- `local_unknown_gain` is a count-based unknown-cell ratio inside
  `information_radius_m`.
- `distance_score` avoids always choosing the nearest candidate by preferring
  useful mid-range/farther goals inside the allowed range.
- `goal_switch_penalty` discourages goals close to the previous goal.
- `min_goal_progress_distance_m` prevents immediate goal churn after success if
  the robot has not moved meaningfully.

## 11. SLAM Toolbox Map Update Tuning

Changed:

```yaml
map_update_interval: 5.0 -> 2.0
```

File:

```text
Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox.yaml
```

Why:

- Simulation already showed active exploration and map growth.
- A 5 second map update interval makes frontier updates feel stale.
- 2 seconds is a moderate laptop/simulation setting and should improve feedback
  without being extreme.

Not changed:

- `throttle_scans: 1`
- `minimum_time_interval: 0.5`
- `transform_publish_period: 0.02`
- `resolution: 0.05`
- scan matching and loop closure parameters

Risk:

- More frequent map publication can increase CPU and memory pressure.
- On Raspberry Pi or low-power hardware, this may be too aggressive.

Rollback:

```yaml
map_update_interval: 5.0
```

Suggested profiles:

- `simulation_fast_map`: `map_update_interval: 2.0`
- `real_robot_safe_map`: `map_update_interval: 5.0`

## 12. Exact Commands To Test

Terminal 1, bringup:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Terminal 2, Nav2 lifecycle check:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
```

Terminal 3, marker-only:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Terminal 4, navigation enabled:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

## 13. Expected Logs

Costmap:

```text
Received costmap: frame=map size=... res=...
```

Safe viewpoint selection:

```text
Goal selection: mode=safe_viewpoint total_frontier_clusters=...
sampled_frontier_cells=... sampled_viewpoints=...
rejected_by_distance=... rejected_by_costmap=...
rejected_by_clearance=... rejected_by_blacklist=...
selected_cluster_id=... selected_source=frontier_cell
selected_distance=... selected_score=...
selected_cost=... selected_world=(..., ...)
```

Goal dispatch:

```text
Sending NavigateToPose goal: state=WAITING_FOR_SERVER frame=map x=... y=... cluster_id=... source=frontier_cell distance=... score=... cost=...
NavigateToPose goal accepted: state=NAVIGATING goal=(..., ...)
```

No goal spam:

```text
Navigation skip: state=NAVIGATING enable_navigation=True reason=currently navigating
Navigation skip: ... reason=waiting for goal_update_period_sec
Navigation skip: ... reason=robot has not progressed enough since previous goal
```

## 14. Markers

Marker topic:

```text
/active_slam/markers
```

Namespaces:

- `frontier_clusters`
- `cluster_centroid`
- `best_frontier`
- `selected_goal`
- `safe_viewpoint_candidates`
- `rejected_viewpoint_candidates`
- `blacklist`

Marker count caps:

```yaml
max_frontier_markers: 100
max_candidate_markers: 40
max_rejected_markers: 40
```

## 15. Verification

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_mapping bumperbot_bringup
```

Result:

```text
Summary: 3 packages finished
```

Syntax:

```bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

Default launch smoke test:

```bash
source install/setup.bash
timeout --signal=INT 6 ros2 launch bumperbot_active_slam active_slam.launch.py
```

Observed:

```text
Navigation dispatch: enable_navigation=False
Safe viewpoint config: mode=safe_viewpoint costmap_filter=True costmap_topic=/global_costmap/costmap
process has finished cleanly
```

## 16. Known Limitations

- No path entropy.
- No SLAM pose-graph uncertainty.
- No full roadmap/TSP scheduling.
- Viewpoints are sampled locally around frontier cells, not globally optimized.
- Costmap filtering can reject all candidates near narrow frontiers; tune
  `safety_radius_m`, `max_allowed_cost`, or `reject_unknown_cost` if needed.
