# Frontier Goal Candidate Selection Fix Report

## Root Cause

The previous navigation goal used only `FrontierCluster.centroid_world`.

That fails when one large frontier cluster wraps around the robot. The centroid
of the whole cluster can land near `base_link` even though many frontier cells
in that same cluster are valid exploration targets farther away.

Observed runtime:

```text
frontier_clusters=1
best_centroid_world=(0.01, 0.06)
best_distance=0.06 m
Navigation skip: all frontier candidates are closer than min_candidate_distance_m
```

The issue was not Nav2 lifecycle and not frontier detection. The cluster had
frontier cells, but the navigation candidate was collapsed to a centroid.

## Files Changed

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/README.md`

No Nav2 config, controller, planner, costmap, path entropy, or roadmap logic was
changed.

## New Config

```yaml
goal_candidate_mode: frontier_cell
min_candidate_distance_m: 0.6
max_candidate_distance_m: 6.0
max_cells_sampled_per_cluster: 50
prefer_nearest_valid_candidate: true
```

Supported candidate modes:

- `frontier_cell`: sample cells from each frontier cluster and use valid sampled
  cells as navigation candidates.
- `centroid`: old centroid behavior, kept only for debugging.

Default is `frontier_cell`.

## Candidate Selection Algorithm

For each frontier cluster:

1. Sample up to `max_cells_sampled_per_cluster` cells from `cluster.cells`.
2. Convert each sampled map cell to world coordinates with `map_to_world()`.
3. Compute distance from robot pose to candidate.
4. Reject candidates closer than `min_candidate_distance_m`.
5. Reject candidates farther than `max_candidate_distance_m`.
6. Reject candidates inside the active blacklist radius.
7. If `prefer_nearest_valid_candidate=true`, select the nearest valid candidate
   to make Nav2 testing conservative.
8. If `prefer_nearest_valid_candidate=false`, prefer larger clusters and farther
   candidates.

The node no longer sets `SUCCEEDED` just because a centroid is near the robot.
`goal_reached_distance_m` is only used while a sent goal is active.

## Logging

Goal selection logs now include:

- candidate mode
- total frontier clusters
- total sampled candidates
- rejected too close
- rejected too far
- rejected blacklisted
- selected candidate world/map coordinate
- selected source cluster id
- selected candidate distance

Expected selected-candidate log:

```text
Goal selection: mode=frontier_cell total_frontier_clusters=1 total_sampled_candidates=50 rejected_too_close=... rejected_too_far=... rejected_blacklisted=0 selected_cluster_id=0 selected_source=frontier_cell selected_distance=... selected_world=(..., ...) selected_map=(..., ...) skip_reason=none
```

Expected send log:

```text
Sending NavigateToPose goal: state=WAITING_FOR_SERVER frame=map x=... y=... cluster_id=0 source=frontier_cell distance=...
```

If no valid candidate exists:

```text
Goal selection: mode=frontier_cell total_frontier_clusters=... total_sampled_candidates=... rejected_too_close=... rejected_too_far=... rejected_blacklisted=... selected=none skip_reason=...
```

## Markers

Existing marker topic remains:

```text
/active_slam/markers
```

Marker namespaces:

- `frontier_clusters`: existing centroid point markers.
- `cluster_centroid`: explicit centroid debug spheres.
- `best_frontier`: best cluster centroid debug marker.
- `selected_goal`: selected navigation candidate, which is a sampled frontier
  cell in `frontier_cell` mode.
- `blacklist`: failed/blacklisted candidate regions.

## Test With `enable_navigation=false`

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Expected:

```text
Navigation dispatch: enable_navigation=False
Navigation skip: state=IDLE enable_navigation=False reason=enable_navigation=false
```

No Nav2 goal should be sent.

## Test With `enable_navigation=true`

Start clean simulation + SLAM + Nav2:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Verify Nav2:

```bash
ros2 action info /navigate_to_pose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /smoother_server
```

Expected:

```text
Action servers: 1
active [3]
```

Launch Active SLAM navigation:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

Expected when there is a valid sampled frontier-cell candidate:

```text
Goal selection: mode=frontier_cell ... selected_cluster_id=... selected_distance=... selected_world=(..., ...)
Sending NavigateToPose goal: state=WAITING_FOR_SERVER frame=map x=... y=...
NavigateToPose goal accepted: state=NAVIGATING goal=(..., ...)
```

## Verification

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam
```

Result:

```text
Summary: 1 package finished
```

Python syntax:

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
Runtime: map=373x221 ... frontier_cells=578 frontier_clusters=1 ...
process has finished cleanly
```

## Known Limitations

- No costmap rejection yet.
- No safe viewpoint sampling yet.
- No path entropy scoring yet.
- Candidate is still a raw frontier cell, not a costmap-validated reachable
  viewpoint.
