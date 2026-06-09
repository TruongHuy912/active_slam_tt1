# Phase 4.9 SLAM Turning Stability Report

## 1. Summary

Phase 4.9 focuses on reducing map skew during sharp turns before adding entropy scoring.

Changes made:

- Audited SLAM Toolbox frames, scan topic, odometry frame, TF chain, and Nav2 controller settings.
- Added `slam_turning_diagnostics.sh` for runtime TF/scan/odom/SLAM/controller diagnostics.
- Added a separate SLAM Toolbox profile for turning stability:
  - `slam_toolbox_turning_stable.yaml`
- Tuned the existing Regulated Pure Pursuit controller to reduce aggressive turning.

No path entropy, SLAM uncertainty scoring, MPPI, roadmap/TSP logic, or copied reference code was added.

## 2. Why Phase 5 Was Not Started

The current blocker is map quality during turning. If the map skews while the robot rotates, entropy scoring would rank goals on top of a distorted map. Phase 5 should wait until SLAM, TF, odometry, scan, and controller behavior are stable enough during repeated turns.

## 3. Root Cause Hypothesis For Map Skew

The most likely causes are a combination of:

- sharp angular commands during Nav2 tracking
- SLAM Toolbox accepting scans too sparsely during rotation
- odometry drift or jumps while rotating
- scan-to-base TF timing issues
- LaserScan data dropouts or poor scan geometry during fast turns

The default SLAM profile had:

```yaml
map_update_interval: 2.0
minimum_time_interval: 0.5
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
```

Those values are conservative and can be too sparse for simulation tests where the robot turns frequently in small rooms. The fix is to test a separate turning-stable SLAM profile and slow the controller turn behavior without changing scan matcher or loop closure internals.

## 4. TF/Odom/Scan/SLAM Audit

Audited configuration:

- SLAM Toolbox:
  - `odom_frame: odom`
  - `map_frame: map`
  - `base_frame: base_footprint`
  - `scan_topic: /scan`
- Diff drive controller:
  - publishes odom on `/bumperbot_controller/odom`
  - odom message frame: `odom`
  - child frame: `base_footprint`
  - publishes TF `odom -> base_footprint`
- Robot description:
  - fixed chain includes `base_footprint -> base_link -> laser_link`
  - Gazebo LiDAR frame is `laser_link`
- Nav2 controller:
  - uses odom topic `/bumperbot_controller/odom`
  - local costmap base frame is `base_link`

Runtime checks still need to be done while the robot is turning:

- `/scan` rate and `frame_id`
- `/bumperbot_controller/odom` rate and frame IDs
- `map -> base_link`
- `odom -> base_link`
- `base_link -> laser_link`
- SLAM Toolbox runtime params

The new script prints these as OK/WARN/FAIL:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh
```

## 5. Controller Tuning

File changed:

```text
Bumper-Bot-main/bumperbot_navigation/config/controller_server.yaml
```

Controller remains:

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

Changed values:

```yaml
desired_linear_vel: 0.22 -> 0.20
lookahead_dist: 0.6 -> 0.75
min_lookahead_dist: 0.3 -> 0.4
max_lookahead_dist: 0.9 -> 1.1
rotate_to_heading_angular_vel: 1.2 -> 0.9
rotate_to_heading_min_angle: 0.785 -> 1.0
max_angular_accel: 2.4 -> 1.8
```

Reason:

- reduce sharp turn commands
- smooth path tracking by looking slightly farther ahead
- reduce sudden heading corrections
- keep the robot moving, but less aggressively in narrow areas

Rollback:

- restore the old numeric values above in `controller_server.yaml`
- rebuild/source `bumperbot_navigation`

No controller plugin was changed, and MPPI was not ported.

## 6. SLAM Toolbox Turning-Stable Profile

New file:

```text
Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Default profile remains unchanged:

```yaml
map_update_interval: 2.0
minimum_time_interval: 0.5
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
throttle_scans: 1
transform_publish_period: 0.02
```

Turning-stable profile:

```yaml
map_update_interval: 1.0
minimum_time_interval: 0.2
minimum_travel_distance: 0.2
minimum_travel_heading: 0.15
throttle_scans: 1
transform_publish_period: 0.02
```

Scan matcher and loop closure parameters were intentionally left unchanged:

- `use_scan_matching`
- `minimum_angle_penalty`
- `angle_variance_penalty`
- `coarse_angle_resolution`
- `fine_search_angle_offset`
- loop closure response thresholds

Use this profile for laptop simulation testing first. Do not make it the Raspberry Pi default until CPU/RAM usage is measured. The rollback is launching without `slam_config:=...slam_toolbox_turning_stable.yaml`.

## 7. Files Changed

Updated:

- `Bumper-Bot-main/bumperbot_navigation/config/controller_server.yaml`

Added:

- `Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh`

No `active_slam_node.py` code was changed in Phase 4.9.

## 8. Active SLAM Node Size

`active_slam_node.py` was not modified in this phase.

Current line count:

```text
846 Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py
```

The Phase 4.9 changes stay in controller config, SLAM config, and diagnostics script.

## 9. Test Commands

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_navigation bumperbot_mapping bumperbot_bringup
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Test 1, baseline SLAM config:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Then:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

Run diagnostics while the robot is turning:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh
```

Test 2, turning-stable SLAM config:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Then:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

Run diagnostics again:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/slam_turning_diagnostics.sh
```

## 10. Expected Result

Expected runtime behavior:

- map skew during sharp turns is reduced
- robot turns less aggressively
- `/scan` keeps a stable measured rate
- `/bumperbot_controller/odom` keeps a stable measured rate
- TF checks pass:
  - `map -> base_link`
  - `odom -> base_link`
  - `base_link -> laser_link`
- SLAM Toolbox publishes non-empty `/map`
- active_slam can continue sending multiple planner-valid goals
- map coverage remains around 80% or better without major skew

If the map still skews badly, compare the diagnostic output between baseline and turning-stable runs. The next likely causes would be odometry drift while turning, scan dropouts, or a TF timing issue rather than frontier scoring.

## 11. Phase 5 Gate Checklist

Only move to Phase 5 path entropy when:

- map no longer skews heavily during turns
- robot can complete many goals in a row
- Nav2 does not spam `FAILED` or `no_path`
- map coverage reaches about 80% or more consistently
- `active_slam_node.py` does not grow further with policy logic
- costmap, TF, scan, odom, and SLAM logs have no major recurring errors

## 12. MPPI Audit

MPPI should not be ported yet.

Consider RotationShim + MPPI later only if:

1. map is stable
2. selected goals are planner-valid
3. paths do not cross unknown/high-cost/low-clearance cells
4. `/scan` and costmaps show obstacles correctly
5. the robot still collides or tracks poorly while following valid paths

That should be a separate controller experiment phase comparing current RPP against RotationShim+MPPI. It is not part of Phase 4.9.

## 13. Verification

Build:

```text
Summary: 4 packages finished
```

Python syntax:

```bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

Install check:

- `slam_toolbox_turning_stable.yaml` is available under the installed `bumperbot_mapping` share directory.
- `slam_turning_diagnostics.sh` is available under the installed `bumperbot_active_slam` share directory.

Runtime Gazebo test was not executed in this turn; use the test protocol above to compare baseline vs turning-stable mapping behavior in `small_house`.
