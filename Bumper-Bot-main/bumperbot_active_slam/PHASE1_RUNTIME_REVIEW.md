# Phase 1 Runtime Review

Scope: runtime logic review for `bumperbot_active_slam` only. No Nav2 goal sending was added.

## Reviewed Files

- `bumperbot_active_slam/active_slam_node.py`
- `bumperbot_active_slam/frontier_detector.py`
- `bumperbot_active_slam/entropy_utils.py`
- `config/active_slam.yaml`
- `README.md`

## Checks

- `/map` QoS:
  - The node uses `RELIABLE + TRANSIENT_LOCAL` QoS for `nav_msgs/msg/OccupancyGrid`.
  - This matches the usual SLAM Toolbox/map-server map publisher behavior.
- TF lookup:
  - Lookup remains `map -> base_link`.
  - Timeout is now configurable with `tf_lookup_timeout_sec`, default `0.5`.
  - TF failures are throttled to avoid log spam.
- Marker frame:
  - Marker frame now uses the incoming map message frame when available.
  - Falls back to configured `global_frame`.
  - Warns if `/map` frame differs from configured `global_frame`.
- Frontier centroid conversion:
  - `map_to_world()` converts cell centers using map origin, resolution, and yaw.
  - Cluster world centroid is the average of cluster cell-center world coordinates.
- RViz marker load:
  - Marker publication is capped by `max_frontier_markers`.
  - Markers represent cluster centroids, not every frontier cell, to avoid RViz lag.

## Runtime Logging Added

The node now logs:

- First map receipt and map dimensions.
- Map frame, resolution, and origin.
- Robot pose in the configured global frame.
- Raw frontier cell count before cluster filtering.
- Filtered frontier cluster count.
- Best frontier centroid in map and world coordinates.
- Best frontier distance from robot when TF is available.

## Files Changed

- `bumperbot_active_slam/active_slam_node.py`
  - Added map receipt logging.
  - Added throttled waiting/TF warnings.
  - Added configurable TF timeout and log period.
  - Added detailed periodic runtime summary.
  - Changed marker frame to prefer `/map` message frame.
- `bumperbot_active_slam/frontier_detector.py`
  - Added `FrontierDetection` result with `frontier_cell_count`.
  - Kept existing `detect_frontiers()` API.
  - Added `detect_frontiers_with_stats()` for runtime logging.
- `config/active_slam.yaml`
  - Added `tf_lookup_timeout_sec`.
  - Added `log_period_sec`.
- `README.md`
  - Documented map QoS, marker cap, selected marker semantics, and runtime logs.

## Verification

Syntax check:

```bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Build check:

```bash
colcon build --symlink-install --packages-select bumperbot_active_slam
```

Result:

```text
Summary: 1 package finished
```

## Phase 1 Boundary

Still not implemented:

- No `NavigateToPose` action client.
- No Nav2 goal sending.
- No costmap usage.
- No blacklist/progress timeout.

