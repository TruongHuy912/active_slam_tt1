# Phase 4.5 Refactor, Global Reposition, And SLAM Profile Report

## 1. Summary

Phase 4.5 keeps the working Phase 4 pipeline and addresses the late-run stall where many far frontiers remain but local safe viewpoint sampling returns no candidate.

Changes made:

- Refactored `active_slam_node.py` into smaller modules.
- Added a two-tier goal selector:
  - `local_safe_viewpoint` for normal nearby frontier viewpoint goals.
  - `global_reposition` fallback for short, safe intermediate goals toward distant frontier clusters.
- Added debug/log fields for local/global candidate counts, fallback mode, relaxed clearance, score, cost, information gain, and safety radius.
- Added a separate SLAM Toolbox simulation-fast profile without changing scan matcher or loop closure parameters.
- Added `slam_config` passthrough to the bringup launch so the SLAM profile can be selected from one command.

No path entropy, SLAM uncertainty, roadmap graph, or TSP scheduling was added.

## 2. Why Phase 5 Path Entropy Was Not Added

The current blocker is not entropy scoring. The robot already accepts and succeeds Nav2 goals, but the selector can exhaust local safe viewpoints while distant frontier clusters remain. Adding path entropy now would score a weak candidate set rather than fix candidate generation. Phase 4.5 therefore improves candidate generation and global fallback first.

## 3. Refactor Result

`active_slam_node.py` now focuses on ROS orchestration:

- parameter declaration and loading
- map/costmap subscriptions
- TF lookup
- periodic frontier detection
- goal selector invocation
- navigation dispatcher invocation
- marker publication
- runtime logging

New modules:

- `models.py`
  - `NavigationCandidate`
  - `CandidateSelectionStats`
  - `GoalSelectorConfig`
- `navigation_dispatcher.py`
  - Nav2 `NavigateToPose` `ActionClient`
  - navigation state machine
  - goal timeout/cancel
  - result callbacks
  - blacklist and retry tracking
- `goal_selector.py`
  - frontier cell sampling
  - local safe viewpoint selection
  - costmap/map safety checks
  - information gain and score calculation
  - global reposition fallback
- `viewpoint_sampler.py`
  - cluster cell sampling
  - circular viewpoint sampling around frontier cells
  - short intermediate global reposition point sampling
- `marker_utils.py`
  - clear marker array
  - frontier cluster markers
  - centroid/debug markers
  - selected goal markers
  - safe/rejected candidate markers
  - blacklist markers
- `slam_status_utils.py`
  - small helper for documenting SLAM profile values

## 4. File Responsibilities

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
  - ROS node named `active_slam_explorer`; no package API or launch name changed.
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/goal_selector.py`
  - All local/global candidate selection logic.
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/navigation_dispatcher.py`
  - Nav2 action lifecycle and blacklist ownership.
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/viewpoint_sampler.py`
  - Sampling primitives.
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/marker_utils.py`
  - RViz `MarkerArray` construction.
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/models.py`
  - Shared dataclasses.
- `Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox_sim_fast.yaml`
  - Optional simulation fast-map SLAM profile.
- `Bumper-Bot-main/bumperbot_bringup/launch/simulated_robot.launch.py`
  - Added `slam_config` launch argument passthrough.

## 5. Root Cause Of `no safe viewpoint candidate`

The Phase 4 selector was local-only:

- It sampled viewpoints around frontier cells.
- It rejected frontier sources beyond `max_candidate_distance_m`.
- It rejected viewpoints beyond `max_viewpoint_distance_m`.
- It had no fallback for distant global frontiers after nearby frontiers were exhausted.

With a late-run frontier centroid around `11.8 m`, a local-only selector with a `4.0 m` viewpoint limit can legitimately return no candidate even while thousands of frontier cells remain. The fix is not to drive directly to a far frontier cell, but to choose a short, safe intermediate goal in known free space toward a promising far cluster.

## 6. Local Safe Viewpoint Logic After Fix

The local selector still does the Phase 4 behavior:

- sample up to `max_cells_sampled_per_cluster` frontier cells per cluster
- create viewpoints around each frontier cell
- require distance in `[min_viewpoint_distance_m, max_viewpoint_distance_m]`
- reject blacklisted or too-close-to-last-goal candidates
- require free occupancy map cells
- require costmap safety when costmap filtering is enabled
- score candidates with cluster size, distance, local unknown gain, cost penalty, and goal switching penalty

Defaults were broadened:

- `max_viewpoint_distance_m: 6.0`
- `max_cells_sampled_per_cluster: 100`
- `min_information_gain_for_goal: 0.0`

## 7. Global Reposition Fallback Logic

If the local selector finds no valid safe viewpoint and `enable_global_reposition` is true:

1. Find frontier clusters with centroid distance in:
   - `global_reposition_min_distance_m: 4.0`
   - `global_reposition_max_distance_m: 12.0`
2. Rank candidate clusters by local information gain, cluster size, and distance.
3. Sample a short fan of intermediate goals from the robot toward the selected far frontier.
4. Keep only candidates in known free map space.
5. Keep only costmap-safe candidates.
6. If strict clearance rejects all candidates and `fallback_relax_clearance` is true, retry with:
   - `fallback_safety_radius_m: 0.18`
   - `fallback_reject_unknown_cost: false`
7. Send the best short intermediate goal, not the far frontier centroid.

Expected selected source values:

- `global_reposition`
- `global_reposition_relaxed`

This is a simple global frontier reposition step inspired by roadmap scheduling, not a roadmap/TSP planner.

## 8. New Config

Added or changed in `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`:

```yaml
max_viewpoint_distance_m: 6.0
max_cells_sampled_per_cluster: 100
min_information_gain_for_goal: 0.0
enable_global_reposition: true
global_reposition_min_distance_m: 4.0
global_reposition_max_distance_m: 12.0
global_reposition_step_m: 2.5
global_reposition_sample_count: 24
fallback_relax_clearance: true
fallback_safety_radius_m: 0.18
fallback_reject_unknown_cost: false
```

## 9. Ideas From References

From `m-explore-ros2`:

- Keep a blacklist/progress mechanism so failed goals do not get retried forever.
- Avoid repeatedly sending the same goal.
- Combine distance and frontier size/gain instead of choosing only the nearest point.
- Treat the costmap as a safety source before goal dispatch.

From `roadmap-explorer`:

- Avoid pure greedy local exploration when local candidates are exhausted.
- Use a global/frontier-level target to reposition the robot toward a more useful region.
- Keep state/session concepts conceptually separate from raw frontier detection.

Intentionally not ported:

- full roadmap graph construction
- TSP/scheduler implementation
- raytraced information gain engine
- lifecycle/plugin architecture
- package code from either reference repo

## 10. SLAM Toolbox Profile/Tuning

Default profile remains:

```yaml
map_update_interval: 2.0
minimum_time_interval: 0.5
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
```

Added simulation profile:

```text
Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox_sim_fast.yaml
```

Fast-map values:

```yaml
map_update_interval: 1.0
minimum_time_interval: 0.2
minimum_travel_distance: 0.2
minimum_travel_heading: 0.2
```

No scan matcher or loop closure tuning was changed.

Use fast profile only for laptop simulation until CPU/RAM load is measured. On Raspberry Pi, use the default profile or a slower real-robot profile. Rollback is simply launching without `slam_config:=...slam_toolbox_sim_fast.yaml`.

## 11. Commands Test

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_mapping bumperbot_bringup
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Terminal 1, normal bringup:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Terminal 1, fast SLAM simulation profile:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_sim_fast.yaml
```

Terminal 2, Nav2 lifecycle check:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
```

Terminal 3, marker-only:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Terminal 4, navigation enabled:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

## 12. Expected Logs

Local mode:

```text
Goal selection: mode=safe_viewpoint selected_mode=local_safe_viewpoint ...
selected_source=frontier_cell selected_distance=... selected_score=...
```

Global fallback mode:

```text
Goal selection: mode=safe_viewpoint selected_mode=global_reposition fallback_mode=global_reposition ...
global_attempted=True global_candidates=... selected_source=global_reposition...
```

Relaxed fallback mode:

```text
selected_source=global_reposition_relaxed relaxed_clearance=True safety_radius=0.18
```

Nav2 dispatch:

```text
Sending NavigateToPose goal: state=WAITING_FOR_SERVER ...
NavigateToPose goal accepted: state=NAVIGATING ...
NavigateToPose result: SUCCEEDED
```

No goal spam:

```text
Navigation skip: state=NAVIGATING enable_navigation=True reason=currently navigating
```

## 13. Verification Performed

Build:

```text
Summary: 3 packages finished
```

Syntax:

```bash
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

Launch argument check with sandbox-safe log directory confirmed `slam_config` is exposed by `simulated_robot.launch.py`.

## 14. Known Limitations

- No path entropy yet.
- No SLAM pose graph uncertainty yet.
- No full roadmap/TSP scheduling.
- `global_reposition` is a simple intermediate-goal fallback inspired by roadmap-style global reasoning, not a full roadmap planner.
- Costmap rejection is still grid-based and does not replace Nav2 planning validation.
