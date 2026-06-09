# bumperbot_active_slam

Active SLAM package for Bumper-Bot on ROS 2 Humble.

This package performs frontier detection and can optionally dispatch conservative Nav2 goals. It subscribes to the SLAM map and Nav2 global costmap, looks up the robot pose with TF, clusters frontier cells, samples safe viewpoints, logs selected candidates, and publishes RViz debug markers.

Navigation is disabled by default. With default config it does not send Nav2 goals.

## Build

From `/home/hlq017912/Downloads/bumper_bot_active_slam_new`:

```bash
colcon build --symlink-install --packages-select bumperbot_active_slam
source install/setup.bash
```

## Launch

Start the existing Bumper-Bot simulation with SLAM first:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

`world_name:=small_house` or `world_name:=small_warehouse` is important for
SLAM testing. The launch default is `empty`, where `/scan` may contain only
infinite ranges and SLAM Toolbox can keep publishing an empty `0x0` map.

Then launch this package:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

For faster SLAM map updates in laptop simulation, use the optional simulation
profile:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_sim_fast.yaml
```

Use the default `slam_toolbox.yaml` profile on the real robot unless CPU/RAM
headroom has been measured.

This default launch is marker-only:

```yaml
enable_navigation: false
```

To explicitly enable Nav2 `NavigateToPose` dispatch:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

`simulated_robot.launch.py` already includes `bumperbot_navigation/navigation.launch.py`.
Do not launch `navigation.launch.py` a second time in another terminal unless
you are intentionally running the simulation from lower-level component
launches. Duplicate Nav2 node names can leave lifecycle state and action server
checks ambiguous.

Before enabling navigation, verify Nav2 is active:

```bash
ros2 action info /navigate_to_pose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
```

Expected Nav2 state is one `/navigate_to_pose` action server and lifecycle
state `active [3]` for the navigation lifecycle nodes.

To disable navigation immediately, stop the active_slam node with `Ctrl+C` and
restart without the override:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

## Phase 1 Runtime Check

After launching simulation/SLAM, check the sensor, map, TF, and marker pipeline:

```bash
ros2 run bumperbot_active_slam active_slam_explorer
```

Or with the launch file:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

In another terminal:

```bash
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/phase1_runtime_check.sh
```

For a SLAM-specific diagnosis:

```bash
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_map_runtime_check.sh
```

The important checks are:

```bash
ros2 topic echo /scan --once
ros2 topic echo /clock --once
ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
```

If `/map` reports `width: 0` and `height: 0`, frontier detection cannot produce
frontiers yet. Check `/scan`, `/clock`, Gazebo pause state, and whether SLAM
Toolbox was launched through `use_slam:=true`.

## Required Topics And Frames

Required topics:

- `/map` (`nav_msgs/msg/OccupancyGrid`)
- `/tf`
- `/tf_static`

Required frames:

- `map`
- `base_link`

The frame names are configurable in `config/active_slam.yaml`.

The `/map` subscription uses reliable transient-local QoS to match the usual
SLAM Toolbox/map-server map publisher behavior.

## Debug Markers

The node publishes `visualization_msgs/msg/MarkerArray` on:

```text
/active_slam/markers
```

RViz can display this with a `MarkerArray` display.

To avoid RViz lag, the number of frontier cluster markers is capped by:

```yaml
max_frontier_markers: 100
```

The `frontier_clusters` and `cluster_centroid` marker namespaces show cluster
centroids for debugging. The selected best frontier centroid is shown as a green
sphere.

When navigation is enabled, the selected Nav2 goal candidate is shown in the
`selected_goal` marker namespace. In the default `frontier_cell` candidate mode,
this marker is a sampled safe viewpoint, not the cluster centroid. If local
safe viewpoints run out, the node can use `global_reposition` to send a short
safe intermediate goal toward a distant frontier. Failed frontiers may appear in
the `blacklist` namespace.

When planner validation is enabled, a valid `ComputePathToPose` result is shown
in the `path_validated` marker namespace. The node does not send
`NavigateToPose` unless a planner-validated path exists when
`planner_validation_required_for_navigation: true`.

Additional marker namespaces used during navigation debugging:

- `candidate_goals`: sampled candidate viewpoints before planner validation.
- `rejected_candidates`: candidates rejected by distance, costmap, blacklist, or clearance checks.
- `planner_rejected_candidates`: candidates rejected after `ComputePathToPose` or path safety checks.
- `planner_reject_blacklist`: temporary planner reject cache entries.
- `blacklist`: failed Nav2 goals, shown with the configured blacklist radius.

If RViz shows a circle or disk near the robot, it is usually a `blacklist` or
`planner_reject_blacklist` safety/debug marker, not a new controller command.
These markers are capped by the `max_*_markers` parameters to avoid RViz lag.

When the robot is in high/inflated cost, `high_cost_escape` temporarily has
priority over efficient utility scoring. The visible candidate points around the
robot are short escape candidates sampled in nearby known free space. They still
must pass planner validation before a `NavigateToPose` goal is sent. After a
successful high-cost escape, the progress gate is relaxed briefly so normal
frontier sampling and efficient utility ranking can resume.

## Runtime Logs

The node logs:

- first map receipt and map dimensions
- robot pose in the configured global frame
- raw frontier cell count
- filtered frontier cluster count
- best frontier centroid in map and world coordinates
- sampled navigation candidate counts and selected candidate world coordinate

The log period is controlled by:

```yaml
log_period_sec: 5.0
```

When no valid map/frontier exists, the node publishes a clear `MarkerArray` so
RViz can still discover `/active_slam/markers`:

```yaml
publish_empty_markers: true
```

## RViz

Add or edit a `MarkerArray` display:

- Fixed Frame: `map`
- Topic: `/active_slam/markers`

If an existing `MarkerArray` display is set to `/waypoints`, change it to
`/active_slam/markers`.

## Phase 1 Limits

- `NavigateToPose` dispatch is optional and disabled by default.
- No path entropy scoring beyond utility functions.
- Uses the published Nav2 global costmap for optional viewpoint filtering.
- Safe viewpoint sampling is local around frontier cells with a simple global
  reposition fallback; it is not a full roadmap or global planner replacement.
- No path entropy, SLAM uncertainty, or full roadmap/TSP scheduling yet.

## Navigation Parameters

Phase 5 efficient utility is optional. It ranks the same safe candidates used by
the Phase 4 pipeline and does not replace planner validation:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=true \
  scoring_mode:=efficient_entropy_utility
```

During early/bootstrap mapping, efficient mode may fall back to baseline
`safe_viewpoint` selection or a short known-free-space bootstrap goal so the
robot does not stand still before there are enough candidates to rank.

`global_reposition` is a fallback, not the main exploration policy. Recent
global reposition goals are remembered for a cooldown window so the robot does
not ping-pong between the same two regions. If a repeated global reposition is
attempted, the selector logs the reject reason such as `recent_region`,
`zero_gain`, `cooldown`, or `pingpong` and prefers local/frontier candidates or
waits for a different region.

Default navigation safety parameters:

```yaml
enable_navigation: false
navigate_action_name: /navigate_to_pose
goal_update_period_sec: 5.0
goal_timeout_sec: 60.0
goal_candidate_mode: frontier_cell
scoring_mode: safe_viewpoint
use_costmap_filter: true
costmap_topic: /global_costmap/costmap
max_allowed_cost: 70
reject_unknown_cost: true
safety_radius_m: 0.25
viewpoint_sample_radius_m: 0.45
viewpoint_num_samples: 16
min_goal_separation_m: 0.5
min_candidate_distance_m: 0.6
min_viewpoint_distance_m: 0.8
max_candidate_distance_m: 6.0
max_viewpoint_distance_m: 6.0
min_goal_progress_distance_m: 0.4
min_cluster_size_for_navigation: 5
max_cells_sampled_per_cluster: 100
prefer_nearest_valid_candidate: true
w_cluster_size: 1.0
w_distance: 0.5
w_information_gain: 1.0
w_cost_penalty: 1.0
w_goal_switching: 0.5
information_radius_m: 0.6
prefer_farther_than_current: true
min_information_gain_for_goal: 0.0
enable_global_reposition: true
global_reposition_min_distance_m: 4.0
global_reposition_max_distance_m: 12.0
global_reposition_step_m: 2.5
global_reposition_sample_count: 24
fallback_relax_clearance: true
fallback_safety_radius_m: 0.18
fallback_reject_unknown_cost: false
goal_reached_distance_m: 0.35
blacklist_radius_m: 0.6
blacklist_timeout_sec: 90.0
max_retries_per_frontier: 2
send_goal_on_startup: false
use_planner_validation: true
planner_action_name: /compute_path_to_pose
planner_id: GridBased
planner_validation_timeout_sec: 3.0
min_valid_path_length_m: 0.2
max_valid_path_length_m: 15.0
max_path_cost: 70
reject_path_unknown: true
path_check_step_m: 0.05
path_clearance_radius_m: 0.22
max_planner_validation_candidates: 20
planner_validation_required_for_navigation: true
```

The node will not send a goal while another goal is pending or navigating.
Cluster centroids are useful for debugging, but a large frontier cluster can
wrap around the robot and place its centroid near `base_link`. For navigation,
the default `goal_candidate_mode: frontier_cell` samples frontier cells inside
each cluster and filters candidates by distance:

```yaml
min_candidate_distance_m: 0.6
max_candidate_distance_m: 6.0
```

`goal_candidate_mode: centroid` is still available for debug, but it is not the
recommended navigation mode when frontiers surround the robot.

When the local selector reports `selected=none skip_reason=no safe viewpoint
candidate` while many far frontier clusters remain, check whether
`enable_global_reposition` is enabled. The fallback does not drive directly into
unknown space; it samples short intermediate goals in known free space toward a
large distant frontier.

If the robot appears to drive into walls or doors, run:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/collision_diagnostics_check.sh
```

This checks planner actions, `/scan`, global/local costmap topics, and Nav2
lifecycle state. If a door or wall is missing from `/scan` or the costmaps,
planner validation cannot reliably reject that collision; fix the world sensor
or Nav2 costmap first.
