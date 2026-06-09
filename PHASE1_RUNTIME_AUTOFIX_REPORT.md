# Phase 1 Runtime Autofix Report

## 1. Summary

`bumperbot_active_slam` was updated to handle the observed Phase 1 runtime state cleanly:

- `/map` can be received as `0x0` from SLAM Toolbox before a valid map exists.
- Frontier detection is now skipped for empty/invalid maps.
- The marker topic is still published with a clear `MarkerArray` while waiting.
- TF startup delay is handled with throttled warnings.
- Ctrl+C shutdown now checks the rclpy context before calling `rclpy.shutdown()`.
- A runtime diagnostic script was added.

No Nav2 `NavigateToPose` action client, goal dispatch, costmap subscriber, or advanced scoring was added.

## 2. What Was Tested

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam
```

Result:

```text
Summary: 1 package finished
```

Syntax:

```bash
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Runtime check script:

```bash
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/phase1_runtime_check.sh
```

In the current shell session, simulation/SLAM was not running, so the script correctly reported missing `/scan`, `/clock`, `/map`, TF, and active_slam node.

Shutdown smoke test:

```bash
source install/setup.bash
timeout --signal=INT 6 ros2 run bumperbot_active_slam active_slam_explorer
```

No `rcl_shutdown already called` traceback was observed.

## 3. Root Cause Analysis

The active_slam code was not the root cause of `map=0x0`.

The observed log:

```text
Received map: frame=map size=0x0 resolution=0.050
Runtime: map=0x0 robot=(-0.00, -0.00) frontier_cells=0 frontier_clusters=0 best=none
```

means SLAM Toolbox has published an `OccupancyGrid`, but it has no cells yet. With `width=0`, `height=0`, and `data=[]`, no frontier can mathematically exist. The correct behavior is to wait and diagnose the upstream SLAM/simulation pipeline.

The Ctrl+C error was local to `active_slam_node.py`: `main()` always called `rclpy.shutdown()`, even if the context had already been shut down.

## 4. What Was Fixed

Files changed under `Bumper-Bot-main/bumperbot_active_slam`:

- `bumperbot_active_slam/active_slam_node.py`
  - Skips frontier detection on empty/invalid maps.
  - Logs:
    - `Received empty map 0x0 from /map; waiting for SLAM Toolbox to publish a valid map. Check /scan, /clock, and SLAM launch.`
  - Publishes clear/empty `MarkerArray` while waiting for map/frontiers.
  - Keeps `/active_slam/markers` active when the node is running.
  - Keeps TF warnings throttled.
  - Logs `no frontier clusters found` when map is valid but no clusters pass filtering.
  - Fixes shutdown by checking `rclpy.get_default_context().ok()` before shutdown.
- `config/active_slam.yaml`
  - Added `publish_empty_markers: true`.
- `setup.py`
  - Installs shell scripts into package share.
- `scripts/phase1_runtime_check.sh`
  - Added automated Phase 1 runtime diagnostics.
- `README.md`
  - Added runtime check, `/map` validity checks, and RViz marker instructions.

## 5. What Was Not Fixed Because It Is Outside `bumperbot_active_slam`

No changes were made to:

- SLAM Toolbox launch/config.
- Gazebo/simulation launch.
- `/scan` bridge or sensor config.
- `/clock` simulation clock.
- Bumper-Bot controller/planner/Nav2 config.
- `m-explore-ros2`.
- `roadmap-explorer`.

If `/map` remains `0x0`, the likely issue is upstream of `bumperbot_active_slam`.

## 6. Exact Commands For The User To Run Next

Terminal 1:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

Terminal 2:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Terminal 3:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/phase1_runtime_check.sh
```

Optional direct checks:

```bash
ros2 topic echo /scan --once
ros2 topic echo /clock --once
ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /active_slam/markers -v
```

## 7. Expected Good Output

The runtime check script should eventually report:

```text
OK: /scan produced at least one LaserScan sample.
OK: /clock produced at least one sample.
OK: /map produced an OccupancyGrid sample.
OK: /map width/height are greater than zero (...x...).
OK: TF map -> base_link is available.
OK: /active_slam/markers has a publisher.
OK: /active_slam_explorer node is running.
```

The active_slam node should log something like:

```text
Received map: frame=map size=<nonzero>x<nonzero> resolution=0.050
Runtime: map=<nonzero>x<nonzero> res=0.050 robot=(x, y) frontier_cells=N frontier_clusters=M ...
```

If a valid map exists but there are no frontiers, the node should log:

```text
no frontier clusters found
```

## 8. If `/map` Is Still `0x0`, Likely Causes

- `/scan` is not publishing data.
- Gazebo is paused.
- `/clock` is not running.
- SLAM Toolbox is not receiving scan data.
- The simulation was launched without the SLAM path, for example missing `use_slam:=true`.
- The robot has not moved or scan/map initialization has not completed.
- TF was not available early enough for SLAM to initialize.

With `map=0x0`, frontier detection cannot produce frontier markers. The active_slam node should only publish clear/empty markers and wait.

## 9. RViz Instruction

In RViz:

- Set `Fixed Frame` to `map`.
- Add a `MarkerArray` display.
- Set the MarkerArray topic to:

```text
/active_slam/markers
```

If an existing MarkerArray display is set to `/waypoints`, change it to `/active_slam/markers`.

