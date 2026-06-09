# SLAM Map 0x0 Root Cause Report

## 1. Root Cause

`bumperbot_active_slam` is not the cause of `/map` being `0x0`.

The active launch command that was being used:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

does start `slam_toolbox`, but the Gazebo world argument defaults to:

```text
world_name:=empty
```

The current runtime evidence shows:

- `/slam_toolbox` exists.
- `/scan` exists and is subscribed by `slam_toolbox`.
- `/clock` is running.
- `/map` is published by `slam_toolbox` with `RELIABLE + TRANSIENT_LOCAL`.
- TF is available:
  - `map -> base_link`
  - `odom -> base_footprint`
  - `base_link -> laser_link`
- `/scan.header.frame_id` is `laser_link`.
- `slam_toolbox` parameters are correct:
  - `mode: mapping`
  - `scan_topic: /scan`
  - `base_frame: base_footprint`
  - `odom_frame: odom`
  - `map_frame: map`
  - `use_sim_time: True`
- `/map` remains:
  - `width: 0`
  - `height: 0`
  - `data: []`

The default `empty.world` only contains a ground plane and no useful nearby 2D LiDAR obstacles. In this state SLAM Toolbox can publish an empty map metadata message but never grow a valid occupancy grid. A brief rotation command was also tested in the current empty-world session; odometry changed, but `/map` stayed `0x0`.

Conclusion: the practical root cause is launching SLAM in the default empty Gazebo world. Use a non-empty world for SLAM, such as `small_house` or `small_warehouse`.

## 2. Correct Launch Command

Use:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Alternative:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_warehouse
```

Then launch active_slam:

```bash
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

`ros2 launch bumperbot_bringup simulated_robot.launch.py --show-args` exposes `world_name`, and its current default is `empty`.

## 3. Required Topics And Frames

Topics:

- `/scan`
  - Type: `sensor_msgs/msg/LaserScan`
  - Publisher: `ros_gz_bridge`
  - Subscriber: `slam_toolbox`
  - Expected frame: `laser_link`
- `/clock`
  - Must have measurable rate while Gazebo is running.
- `/map`
  - Type: `nav_msgs/msg/OccupancyGrid`
  - Publisher: `slam_toolbox`
  - QoS: `RELIABLE + TRANSIENT_LOCAL`
- `/active_slam/markers`
  - Type: `visualization_msgs/msg/MarkerArray`
  - Publisher: `active_slam_explorer`

Frames:

- `map -> odom`
  - Published by SLAM Toolbox once mapping is active.
- `odom -> base_footprint`
  - Published by the diff-drive controller.
- `base_footprint -> base_link`
  - Static URDF transform.
- `base_link -> laser_link`
  - Static URDF transform.

The full scan chain required by SLAM is:

```text
odom -> base_footprint -> base_link -> laser_link
```

## 4. Files Changed

No Bumper-Bot core launch/config files were changed.

Files changed only in `bumperbot_active_slam`:

- `Bumper-Bot-main/bumperbot_active_slam/scripts/slam_map_runtime_check.sh`
  - New SLAM-specific runtime diagnostic script.
- `Bumper-Bot-main/bumperbot_active_slam/README.md`
  - Updated launch command to include `world_name:=small_house`.
  - Added SLAM map diagnostic script instructions.

Reason: the issue is launch usage/default world selection, not a proven defect in SLAM Toolbox config, robot description, controller, or active_slam frontier code.

## 5. How To Test Again

Stop the current simulation first, then run:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

In a second terminal:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

In a third terminal:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_map_runtime_check.sh
```

If the robot starts in a visually open area, rotate it in place for a few seconds:

```bash
timeout 8 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

Then run the check script again.

## 6. Expected Good Output

From `slam_map_runtime_check.sh`:

```text
OK: /scan exists
OK: /scan hz OK
OK: /clock hz OK
OK: slam_toolbox node exists
OK: /map width/height > 0
OK: TF map -> base_link OK
OK: TF odom -> base_footprint OK
OK: TF base_link -> laser_link OK
```

From direct `/map` echo:

```text
info:
  resolution: 0.05000000074505806
  width: <greater than 0>
  height: <greater than 0>
data:
  - ...
```

From `active_slam_explorer`, once a valid map with unknown/free boundaries exists:

```text
Runtime: map=<nonzero>x<nonzero> res=0.050 robot=(x, y) frontier_cells=<greater than 0> frontier_clusters=<greater than 0> ...
```

If the valid map is fully known or the robot is still in a poor initial area, `frontier_cells` can temporarily be `0`. That is different from the current `map=0x0` failure.

## 7. RViz

In RViz:

- Fixed Frame: `map`
- Add a `MarkerArray` display.
- Set topic:

```text
/active_slam/markers
```

If an existing MarkerArray display is set to `/waypoints`, change it to `/active_slam/markers`.

## 8. What Not To Do Yet

Do not proceed to:

- Nav2 `NavigateToPose` dispatch.
- Goal state machine.
- Costmap rejection.
- Path entropy scoring.

Those phases should wait until `/map` has nonzero width/height and frontier detection produces meaningful candidates.

